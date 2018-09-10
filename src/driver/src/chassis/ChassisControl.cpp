#include <robot_driver/chassis/ChassisControl.h>

namespace robot
{
ChassisControl::ChassisControl(double wheelRadius, double bias, double radPerTick)
	: wheelRadius(wheelRadius),
	  bias(bias),
	  radPerTick(radPerTick),
	  diffDrive(WheelRadius, bias, radPerTick),
	  leftWheel(2 * M_PI * wheelRadius, "/dev/port_leftwheel", 57600, 20, 10),
	  rightWheel(2 * M_PI * wheelRadius, "/dev/port_rightwheel", 57600, 20, 10)
{
}

ChassisControl::~ChassisControl()
{
}

/**
 * 初始化轮子及差速驱动
 */
void ChassisControl::Init(ros::NodeHandle &nh)
{
	odometry = new Odometry();

	chassisOperate = Forwarding;

	/** 左轮初始化 */
	if (leftWheel.HasErrorOccurred())
	{
		leftWheel.ReopenPort();
	}

	leftWheel.StartMotor();
	leftWheel.ClearFault();
	leftWheel.SetMode(Position);
	leftWheel.SetMode(RelPosition);
	leftWheel.SetLimitVelocity(LimitedVelocity); // 位置模式下的限速

	if (rightWheel.HasErrorOccurred())
	{
		rightWheel.ReopenPort();
	}

	/** 右轮初始化 */
	rightWheel.StartMotor();
	rightWheel.ClearFault();
	rightWheel.SetMode(Position);
	rightWheel.SetMode(RelPosition);
	rightWheel.SetLimitVelocity(LimitedVelocity); // 位置模式下的限速

	/** 里程计初始化 */
	odometry->init(nh, "base_link");

	/** 关节状态初始化 */
	string left_wheel_joint_name = "left_wheel_link";
	string right_wheel_joint_name = "right_wheel_link";
	string ommi_wheel_joint_name = "ommi_wheel_link";

	joint_states.name.push_back(left_wheel_joint_name);
	joint_states.name.push_back(right_wheel_joint_name);
	joint_states.name.push_back(ommi_wheel_joint_name);
	joint_states.position.resize(3, 0.0);
	joint_states.velocity.resize(3, 0.0);
	joint_states.effort.resize(3, 0.0);

	/** 发布关节状态 */
	joint_state_publisher = nh.advertise<sensor_msgs::JointState>("joint_states", 100);

	/** 发布运行状态 */
	run_state_publisher = nh.advertise<sensor_msgs::JointState>("run_state", 100);

	/** 没有移动 */
	UpdateRunState(runState);
}

/**
 * 读取编码器信息, 并更新姿态信息
 */
void ChassisControl::Update(WheelState &leftState, WheelState &rightState)
{
	unique_lock<recursive_mutex> lock(control_mutex);

	if (ros::ok() && odometry->commandTimeout()) // 速度命令是否超时, 若超时则机器人停止
		Velocity(0, 0);

	/** base_link, 底盘的姿态 */
	static ecl::LegacyPose2D<double> pose_update;
	static ecl::linear_algebra::Vector3d pose_update_rates;

	static u8 lBuf[32], rBuf[32];				// 左轮, 右轮, 数据缓存
	static u8 index = 0;						// 为了按一定频率打印轮子状态, 调试使用
	static u8 lDataReaded = 0, rDataReaded = 0; // 读到的左右轮数据的字节数

	static u8 errorNum = 0; // 数据连续出错次数, 若连续出错超过0.5秒底盘轮子将会自锁, 需要清除自锁

	static long errorStartTime; // 统计第一次出错的时间

	checkError();
	leftWheel.ReadStatus(lBuf, &lDataReaded); /** 读取左轮状态 */
	leftState = leftWheel.ParseStatus(lBuf, lDataReaded);

	checkError();
	rightWheel.ReadStatus(rBuf, &rDataReaded); /** 读取右轮状态 */
	rightState = rightWheel.ParseStatus(rBuf, rDataReaded);

	index++;

	if (!leftState.isParsedSuccess || !rightState.isParsedSuccess) // 数据错误
	{
		auto time_now = std::chrono::system_clock::now().time_since_epoch();
		long timestamp = chrono::duration_cast<chrono::milliseconds>(time_now).count();
		if (++errorNum == 1)
		{
			errorStartTime = timestamp;
		}
		else if (timestamp - errorStartTime > 500)
		{ // 轮子已自锁
			checkError();
			leftWheel.ClearFault();

			checkError();
			rightWheel.ClearFault();

			errorNum = 0;
		}
		return;
	}

	errorNum = 0;

	if (!leftState.velocity || !rightState.velocity) // 若某一个轮子的速度是0就认为, 没有移动
		UpdateRunState(IDLE);

	// LOG(INFO) << "leftState.feedback: " << leftState.feedback << ", " << "rightState.feedback: " << rightState.feedback;

	diffDrive.update(leftState.timestamp, leftState.feedback, -rightState.feedback, pose_update, pose_update_rates); // 更新姿态信息
	odometry->update(pose_update, pose_update_rates, 0, 0);

	diffDrive.getWheelJointStates(joint_states.position[0], joint_states.velocity[0], joint_states.position[1], joint_states.velocity[1]);

	joint_states.header.stamp = ros::Time::now();
	joint_state_publisher.publish(joint_states);

	/** 限制打印状态信息的频率 */
	if (index % 10 == 0)
	{
		leftWheel.PrintStatus();
		rightWheel.PrintStatus();

		index = 0;
	}
}

/** 关闭轮子 */
void ChassisControl::Close()
{
	HardwareSerial::isRunning = false;

	if (runState == CLOSED)
	{
		LOG(INFO) << "已关闭.";
		return;
	}

	unique_lock<recursive_mutex> lock(control_mutex);

	checkError();

	if (leftWheel.CloseMotor())
		LOG(INFO) << "左轮已停止";
	else
		LOG(INFO) << "左轮停止失败";

	checkError();
	if (rightWheel.CloseMotor())
		LOG(INFO) << "右轮已停止";
	else
		LOG(INFO) << "右轮停止失败";

	runState = CLOSED;

}

/** 重置里程计超时时间(超时, 则机器人速度就设置为0) */
void ChassisControl::ResetTimeout()
{
	unique_lock<recursive_mutex> lock(control_mutex);
	odometry->resetTimeout();
}

/**
 * 速度, 单位:m/s, rad/s
 */
void ChassisControl::Velocity(double linearVelocity, double angularVectlor)
{
	unique_lock<recursive_mutex> lock(control_mutex);

	double leftVelocity = linearVelocity - angularVectlor * bias / 2;
	double rightVelocity = linearVelocity + angularVectlor * bias / 2;

	if (chassisOperate != Velocitying)
	{
		checkError();
		leftWheel.SetMode(Speed);

		checkError();
		rightWheel.SetMode(Speed);

		chassisOperate = Velocitying;
	}

	checkError();
	leftWheel.SetVelocity(leftVelocity);

	checkError();
	rightWheel.SetVelocity(-rightVelocity);

	runState = BUSY;
}

/**
 * 前进, 单位米
 */
void ChassisControl::Forward(double distance)
{
	unique_lock<recursive_mutex> lock(control_mutex);

	if (chassisOperate != Forwarding)
	{
		checkError();
		leftWheel.SetMode(Mode::Position);
		leftWheel.SetLimitVelocity(LimitedVelocity); // 位置模式下的限速

		checkError();
		rightWheel.SetMode(Mode::Position);
		rightWheel.SetLimitVelocity(LimitedVelocity); // 位置模式下的限速

		chassisOperate = Forwarding;
	}

	checkError();
	leftWheel.SetPosition(distance);

	checkError();
	rightWheel.SetPosition(-distance);

	runState = BUSY;
}

/**
 * 旋转, 单位:°, 机器人自身的坐标系:前x 左y 上z, 以z为轴, 根据右手螺旋法则, 确定机器人的旋转方向(即向左为正, 向右为负)
 */
void ChassisControl::Rotate(double angle)
{
	unique_lock<recursive_mutex> lock(control_mutex);

	float distance = angle / 180 * M_PI * WheelDistance / 2;

	if (chassisOperate != Rotating)
	{
		double v = LimitedAngular * WheelDistance / 2;

		checkError();

		leftWheel.SetMode(Mode::Position);
		leftWheel.SetLimitVelocity(v);

		checkError();

		rightWheel.SetMode(Mode::Position);
		rightWheel.SetLimitVelocity(v);

		chassisOperate = Rotating;
	}

	checkError();
	leftWheel.SetPosition(-distance);

	checkError();
	rightWheel.SetPosition(-distance);

	runState = BUSY;
}

/** 停止 */
void ChassisControl::Stop()
{
	unique_lock<recursive_mutex> lock(control_mutex);

	checkError();

	if (chassisOperate == Forwarding)
		Forward(0);
	else if (chassisOperate == Velocitying)
		Velocity(0, 0);
	else if (chassisOperate == Rotating)
		Rotate(0);
}

/** 获取运行状态 */
RunState ChassisControl::GetRunState()
{
	unique_lock<recursive_mutex> lock(control_mutex);
	return runState;
}

/** 更新运行状态 */
void ChassisControl::UpdateRunState(RunState runState)
{
	unique_lock<recursive_mutex> lock(control_mutex);
	this->runState = runState;
}

/** 
 * 错误检查, 主要是为了解决端口重新插拔
 * 左轮端口有错误, 会重启右轮端口, 因为左轮有错误发生时, 会有等待解决时间, 直到解决完成, 才会执行到右轮, 此时的右轮可能是原来打开的句柄fd, 需要重新打开
 */
void ChassisControl::checkError()
{
	if (leftWheel.HasErrorOccurred())
	{
		rightWheel.ReopenPort();
		rightWheel.StartMotor();
	}
	if (rightWheel.HasErrorOccurred())
	{
		leftWheel.ReopenPort();
		leftWheel.StartMotor();
	}
}

} // namespace robot

#include <robot_driver/chassis/ChassisNode.h>

namespace robot
{

/** 运行状态 */
atomic<bool> ChassisNode::isRunning(false);

ChassisNode::ChassisNode() : chassisControl(WheelRadius, WheelDistance, RadPerTick)
{
	isRunning = true;
}

ChassisNode::~ChassisNode()
{
	minitorThread->join();
	delete minitorThread;
	chassisControl.Close();
}

/**
 * 初始化: ros节点, 轮子驱动, 及线程
 */
void ChassisNode::init(int argc, char **argv)
{
	ros::init(argc, argv, "chassis_node", ros::init_options::NoSigintHandler);

	nh = new ros::NodeHandle("/");

	chassisControl.Init(*nh);

	minitorThread = new thread(&ChassisNode::minitorLoop, this);
}

/**
 * ros消息处理
 */
void ChassisNode::run()
{
	/** 订阅调度命令 */
	ros::Subscriber subSchedule = nh->subscribe(string(TOPIC_NAME_CHASSIS_IN), 100, &ChassisNode::scheduleCallback, this);

	/** 订阅调度命令 */
	ros::Subscriber subVelocity = nh->subscribe(string("/velocity"), 100, &ChassisNode::velocityCallback, this);

	ros::Rate rate(20);
	while (isRunning && ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}
}

/**
 * 轮子状态监控线程函数
 */
void ChassisNode::minitorLoop()
{
	ros::Rate rate(10);
	WheelState leftState, rightWheel;

	while (isRunning && ros::ok())
	{
		chassisControl.Update(leftState, rightWheel);

		rate.sleep();
	}
}

/**
 * 调度请求, 回调函数
 */
void ChassisNode::scheduleCallback(const std_msgs::StringConstPtr &msg)
{
	Document doc;
	doc.Parse(msg->data.c_str());
	if (doc.HasParseError())
	{
		JsonHelper::printError(doc);
		return;
	}
	if (ActionChecker::isChassisAction(doc))
	{
		float linear = doc["linear"].GetFloat();	 /** 线速度 */
		float angular = doc["angular"].GetFloat();   /** 角速度 */
		float distance = doc["distance"].GetFloat(); /** 距离 */
		float angle = doc["angle"].GetFloat();		 /** 角度 */

		if (linear != 0 || angular != 0)
			chassisControl.Velocity(linear, angular);
		else if (distance != 0)
			chassisControl.Forward(distance);
		else if (angle != 0)
			chassisControl.Rotate(angle);
		else
			chassisControl.Velocity(0, 0);
	}
}

/** 速度请求, 回调函数 */
void ChassisNode::velocityCallback(const geometry_msgs::TwistConstPtr &msg)
{
	chassisControl.Velocity(msg->linear.x, msg->angular.z);
	chassisControl.ResetTimeout();
}

/** 超声波回调 */
void ChassisNode::sonarCallback(const std_msgs::StringConstPtr &msg)
{
	std::map<std::string, double> mapSonar;

	Document doc;
	doc.Parse(msg->data.c_str());

	if (doc.HasParseError())
	{
		JsonHelper::printError(doc);
		return;
	}

	if (!ActionChecker::isSonarInfo(doc))
		return;

	mapSonar["left"] = doc["left"].GetFloat();   // 左边声呐到障碍物的距离
	mapSonar["right"] = doc["right"].GetFloat(); // 右边声呐到障碍物的距离
	mapSonar["up"] = doc["up"].GetFloat();
	mapSonar["down"] = doc["down"].GetFloat();

	/** 下方声呐到障碍物的距离 小于0.1m */
	if (mapSonar["down"] < 0.1f)
	{
		chassisControl.Stop();
		return;
	}
}

/** 测试 */
void ChassisNode::test()
{
	chassisControl.Velocity(1, 2);
	LOG(INFO) << "速度(1m/s, 2rad/s)";
	this_thread::sleep_for(chrono::seconds(5));

	chassisControl.Forward(1);
	LOG(INFO) << "前进1m";
	this_thread::sleep_for(chrono::seconds(4));

	chassisControl.Forward(-1);
	LOG(INFO) << "前进-1m";
	this_thread::sleep_for(chrono::seconds(4));

	chassisControl.Rotate(360);
	LOG(INFO) << "向左旋转360°";

	this_thread::sleep_for(chrono::seconds(5));

	chassisControl.Rotate(-360);
	LOG(INFO) << "向右旋转360°";

	this_thread::sleep_for(chrono::seconds(5));
}

} // namespace robot

int main(int argc, char *argv[])
{
	LogHelper log(argv[0]);

	robot::ChassisNode chassisNode;
	chassisNode.init(argc, argv);

	signal(SIGINT, [](int sig) {
		robot::ChassisNode::isRunning = false;
	});

	// chassisNode.test();

	chassisNode.run();

	LOG(INFO) << "chassis_node 退出";

	return 0;
}

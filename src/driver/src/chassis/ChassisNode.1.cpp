#include <robot_driver/chassis/ChassisNode.1.h>

void signalHandler(int signum);

namespace robot
{

std::vector<bool> ChassisNode::m_isFault(MOTOR_NUM, false);

ChassisNode::ChassisNode() : m_motor(MOTOR_NUM, NULL),
							 m_port(MOTOR_NUM, ""),
							 m_posneg(MOTOR_NUM, 1),
							 m_currentencodedvalue(MOTOR_NUM, 0),
							 m_all_status(MOTOR_NUM),
							 m_distance(0),
							 m_isRunning(false),
							 m_motorspeed(MOTOR_NUM, 0.0),
							 m_hasObstacle(false)

{
	init();
}

ChassisNode::~ChassisNode()
{
	destroy();
}

void ChassisNode::ScanThread()
{
	LOG(INFO) << "motor ScanThread start...";
	while (ros::ok())
	{

		std::unique_lock<std::mutex> loc(m_mutex);
		while (m_middlestopflag)
			m_cv.wait(loc);

		usleep(2000);

		StatusCallback();
	}
}

void ChassisNode::run(int argc, char **argv)
{
	ros::init(argc, argv, "chassis_node", ros::init_options::NoSigintHandler);
	m_nh = new ros::NodeHandle();
	ros::Subscriber velocity_subscriber;
	ros::Subscriber sonar_subscriber;
	ros::Subscriber web_vel_subscriber;

	signal(SIGINT, signalHandler); //接收到SIGINT信号ctrl+c

	ParamServerIns.param("topic/sensor/sonar/topic_name", m_sonartopic, "robot/sensor/sonar");
	ParamServerIns.param("topic/chassis/in/topic_name", m_servotopic, "/robot/chassis/move/in");
	ParamServerIns.param("topic/webservo/in/topic_name", m_webservotopic, "/robot/chassis/move/web_in");

	bool flag = false;
	InitParams(m_nh);

	if (m_motor[0] != NULL && m_motor[1] != NULL)
	{
		velocity_subscriber = m_nh->subscribe(m_servotopic.c_str() /* TOPIC_NAME_FOOT_MOVE_IN */, 1, &ChassisNode::VelocityCallBack, this); //获取底盘控制指令
		web_vel_subscriber = m_nh->subscribe(m_webservotopic.c_str() /* TOPIC_REMOTE_CONTROL */, 10, &ChassisNode::ServoWebControlTopicCallback, this);
		// m_servo_publisher = pnh->advertise<std_msgs::String>(TOPIC_NAME_FOOT_MOVE_IN, 10);
		sonar_subscriber = m_nh->subscribe(m_sonartopic.c_str() /* TOPIC_NAME_SENSOR_SONAR */, 1, &ChassisNode::SonarTopicCallback, this);
	}

	m_scanthread = new std::thread(&ChassisNode::ScanThread, this);

	std_msgs::String msg;

	ros::Rate loop_rate(20);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
}

/************************************************************************************
 * Topic回调    START
 ************************************************************************************/

void ChassisNode::VelocityCallBack(const std_msgs::String::ConstPtr &msg)
{
	m_parvalue = ParseJson(msg->data); //解析指令值
	m_middlestopflag = true;		   //先确保运行线程处于休眠状态
	usleep(300);
	if (5 == m_parvalue.size() && 3 != m_remote_cmd.size())
	{
		m_distance = 0.0;						   //每次进入需要清零, 防止遗留值干扰下一步的操作
		LOG(INFO) << "底盘移动指令:" << msg->data; //调度下发的指令只是笼统的指令,需要整理判断再下发
		LOG(INFO) << "遥控: " << m_remote_cmd.size();
		// for (auto item : m_parvalue)
		// 	LOG(INFO) << "发布的: " << item;
		//距离不为零且角度为零时,
		if ((m_parvalue[em_DIS] > DOUBLE_ZERO) || (m_parvalue[em_DIS] < -DOUBLE_ZERO) && ((m_parvalue[em_ANGLE] > -DOUBLE_ZERO) && ((m_parvalue[em_ANGLE] < DOUBLE_ZERO))))
		{
			// LOG(INFO) << "dis===m_distance: " << m_distance;
			m_distance = m_parvalue[em_DIS];
		} //角度不为零且距离为零时,
		else if ((m_parvalue[em_DIS] > -DOUBLE_ZERO && m_parvalue[em_DIS] < DOUBLE_ZERO) && ((m_parvalue[em_ANGLE] > DOUBLE_ZERO) || (m_parvalue[em_ANGLE] < -DOUBLE_ZERO)))
		{
			m_distance = GETDISBY(m_parvalue[em_ANGLE]);
			// LOG(INFO) << "angle===m_distance: " << m_distance;
		}

		m_line_velocity = m_parvalue[em_LINE];
		m_angular_velocity = m_parvalue[em_REGULAR];
		m_diff_control.setVelocityCommands(m_line_velocity, m_angular_velocity);
		// LOG(INFO)<<"m_remote_cmd.size(): "<<m_remote_cmd.size();
		m_workmutex.lock();
		if (3 != m_remote_cmd.size())
		{
			//为了计算编码值,需要先关,后开
			StopMotorRun();
			StartMotorRun();
			LOG(INFO) << "m_remote_cmd";
		}
		m_workmutex.unlock();
		m_middlestopflag = false;
		m_cv.notify_one();
	}
}

void ChassisNode::ServoWebControlTopicCallback(const std_msgs::String::ConstPtr &msg)
{
	m_remote_cmd = ParseRemoteControl(msg->data); //解析远程监控指令集

	// for(auto item : m_remote_cmd){
	// 	LOG(INFO)<<"first: "<<item.first;
	// 	LOG(INFO)<<"second: "<<item.second;
	// }

	if (3 == m_remote_cmd.size() && 5 != m_parvalue.size())
	{
		LOG(INFO) << "web遥控: " << msg->data;
		LOG(INFO) << "口令: " << m_parvalue.size();
		//判断有无语音控制底盘转动, 无则启动电机
		if (5 != m_parvalue.size())
		{
			if (m_isornot_start_flag)
			{
				m_isornot_start_flag = false;
				m_workmutex.lock();
				StartMotorRun();
				m_workmutex.unlock();
			}
		}

		m_diff_control.setVelocityCommands(m_remote_cmd[KEY_LINE], m_remote_cmd[KEY_ANGULAR]);
		m_diff_control.leftrightVelocity(linear_velocity_left, linear_velocity_right);

		// printf("sonardis[SERVOKEY_DOWN]: %lf\n",sonardis[SERVOKEY_DOWN]);
		//遇到障碍物停止状态
		if (sonardis[SERVOKEY_DOWN] <= MAXDIS)
		{
			linear_velocity_right = 0;
			linear_velocity_left = 0;
		}

		//限速
		if (fabs(linear_velocity_left) >= ALLOW_MAXSPEED || fabs(linear_velocity_right) >= ALLOW_MAXSPEED)
		{
			linear_velocity_left = 0;
			linear_velocity_right = 0;
		}
		SetVel(linear_velocity_left, linear_velocity_right);
		LOG(INFO) << "linear_velocity_left: " << linear_velocity_left;
		LOG(INFO) << "linear_velocity_right: " << linear_velocity_right;
		if ((linear_velocity_left > -DOUBLE_ZERO && linear_velocity_left < DOUBLE_ZERO) && (linear_velocity_right > -DOUBLE_ZERO && linear_velocity_right < DOUBLE_ZERO))
		{
			if (m_remote_cmd.size() != 0)
			{
				LOG(INFO) << "before: " << m_remote_cmd.size();
				m_remote_cmd.clear();
				LOG(INFO) << "after: " << m_remote_cmd.size();
				m_isornot_start_flag = true;
				StopMotorRun();
			}
		}
		// LOG(INFO) <<"END...";
	}
}

void ChassisNode::SonarTopicCallback(const std_msgs::String::ConstPtr &msg)
{
	sonardis = ParseSonarJson(msg->data);
}

void ChassisNode::StatusCallback()
{
	// LOG(INFO)<<"StatusCallback: running";
	//如果电机处于停止状态不再循环
	std::vector<bool> status_flag(MOTOR_NUM, true); //电机状态
	// std::vector<unsigned short> motorvel(MOTOR_NUM, 0x00); //电机速度
	// std::vector<float> motorrotation_rate(MOTOR_NUM, 0.0); //电机转速rpm/min
	// // std::vector<float> m_motorspeed(MOTOR_NUM, 0.0);	   //电机速度m/s

	// std::vector<unsigned short> motorvol(MOTOR_NUM, 0x00);   //电机电压
	// std::vector<float> motorcur(MOTOR_NUM, 0x00);			 //电机电流
	// std::vector<unsigned short> pos_givenH(MOTOR_NUM, 0x00); //位置给定高16位
	// std::vector<unsigned short> pos_givenL(MOTOR_NUM, 0x00); //位置给定低16位

	// std::vector<unsigned short> pos_backH(MOTOR_NUM, 0x00); //位置回馈高16位
	// std::vector<unsigned short> pos_backL(MOTOR_NUM, 0x00); //位置回馈低16位

	// std::vector<unsigned int> position_given(MOTOR_NUM, 0x00); //H16+L16
	//std::vector<unsigned int> position_back(MOTOR_NUM, 0x00); //H16+L16
	long cha = 0;
	// std::vector<std::map<unsigned char, std::vector<unsigned char>>> all_status(MOTOR_NUM);
	// ros::Time begin = ros::Time::now();
	for (int i = 0; i < MOTOR_NUM; i++)
	{
		if (3 != m_remote_cmd.size() || 5 == m_parvalue.size())
		{
			m_all_status[i] = m_motor[i]->Parse_Status();
			if ((m_all_status[i].size()) != 8)
				continue;

			status_flag[i] = m_motor[i]->MotorStatus_Judge(m_all_status[i][STATUS(1)][1]);
			if (!status_flag[i])
			{
				m_isFault[i] = true;
			}
			m_currentencodedvalue[i] = H16_L16(H8_L8(m_all_status[i], POSITION_FEEDBACK_HIGH16(1)), H8_L8(m_all_status[i], POSITION_FEEDBACK_LOW16(1)));
		}

		//获取速度
		// motorvel[i] = H8_L8(m_all_status[i], ROTATION_RATE(1));
		// motorrotation_rate[i] = REAL_ROTATION_RATE(motorvel[i]);
		// m_motorspeed[i] = ROTATION_RATE2SPEED(motorvel[i]);
		// //判断电机的正反方向runningEncodevalue
		// if (i == 0 && !m_posneg[i] || i == 1 && m_posneg[i])
		// {
		// 	motorvel[i] = 24000 - motorvel[i];
		// 	m_motorspeed[i] = 212 - m_motorspeed[i];
		// }

		// //获取母线电压 V
		// motorvol[i] = H8_L8(m_all_status[i], BUSBAR_VOLTAGE(1));
		// //获取输出电流 A
		// motorcur[i] = H8_L8(m_all_status[i], OUTPUT_CURRENT(1)) / 100.0;
		// //位置给定获取
		// pos_givenH[i] = H8_L8(m_all_status[i], POSITION_GIVEN_HIGH16(1));
		// pos_givenL[i] = H8_L8(m_all_status[i], POSITION_GIVEN_LOW16(1));
		// position_given[i] = H16_L16(pos_givenH[i], pos_givenL[i]);
		// //位置反馈获取
		// pos_backH[i] = H8_L8(m_all_status[i], POSITION_FEEDBACK_HIGH16(1));
		// pos_backL[i] = H8_L8(m_all_status[i], POSITION_FEEDBACK_LOW16(1));
		// position_back[i] = H16_L16(pos_backH[i], pos_backL[i]);

		// H8_L8(m_all_status[i], POSITION_FEEDBACK_HIGH16(1));
		// H8_L8(m_all_status[i], POSITION_FEEDBACK_LOW16(1));

		//目标距离-电机反馈值=还需要运行的距离
		// LOG(INFO) << "位置反馈值: " << i << "   " << H16_L16(H8_L8(m_all_status[i], POSITION_FEEDBACK_HIGH16(1)), H8_L8(m_all_status[i], POSITION_FEEDBACK_LOW16(1)));
		//记录反馈的编码值
		// LOG(INFO) << "位置反馈值: " << i << m_currentencodedvalue[i];

		// LOG(INFO)<<"m_remote_cmd.size(): "<<m_remote_cmd.size();

		/* else  */ if (5 == m_parvalue.size())
		{
			m_diff_control.leftrightVelocity(linear_velocity_left, linear_velocity_right);

			if (!((m_parvalue[em_DIS] > -DOUBLE_ZERO && (m_parvalue[em_DIS] < DOUBLE_ZERO) && (m_parvalue[em_ANGLE] > -DOUBLE_ZERO && m_parvalue[em_ANGLE] < DOUBLE_ZERO))))
			{
				cha = (fabs(GETCODEDVALUEBY(m_distance)) - fabs(m_currentencodedvalue[i]));
				LOG(INFO) << "位置差值: " << cha;
				LOG(INFO) << "最小值: " << fabs(DECELDIS(GETCODEDVALUEBY(m_distance)));
			}

			//确保电机在距离的x%时, 做减速处理
			if (cha <= fabs(DECELDIS(GETCODEDVALUEBY(m_distance))) && cha > 0 && sonardis[SERVOKEY_DOWN] > MAXDIS && !((m_parvalue[em_DIS] > -DOUBLE_ZERO && (m_parvalue[em_DIS] < DOUBLE_ZERO) && (m_parvalue[em_ANGLE] > -DOUBLE_ZERO && m_parvalue[em_ANGLE] < DOUBLE_ZERO))))
			{
				LOG(INFO) << "减速处理:==============";
				SetVel(linear_velocity_left, linear_velocity_right);
				m_line_velocity *= REDUCTION_GEAR_RATIO;
				m_angular_velocity *= REDUCTION_GEAR_RATIO;
				m_diff_control.setVelocityCommands(m_line_velocity, m_angular_velocity);

				//速度减小到一定程度停止
				if (m_line_velocity <= 1 || m_angular_velocity <= 1)
				{
					m_middlestopflag = true;
					SetVel(0, 0);
					StopMotorRun();
					m_parvalue.clear();
				}
			}
			// // //cha<=0说明此时,距离已达到要求距离
			else if ((cha <= 0) && !((m_parvalue[em_DIS] > -DOUBLE_ZERO && (m_parvalue[em_DIS] < DOUBLE_ZERO) && (m_parvalue[em_ANGLE] > -DOUBLE_ZERO && m_parvalue[em_ANGLE] < DOUBLE_ZERO))))
			{
				LOG(INFO) << "(cha <= 0):==============";
				if (sonardis[SERVOKEY_DOWN] <= MAXDIS)
				{
					m_hasObstacle = true;
				}
				//停止
				m_middlestopflag = true;
				SetVel(0, 0);
				StopMotorRun();
				m_parvalue.clear();
			}
			else if (cha > DECELDIS(m_distance) && sonardis[SERVOKEY_DOWN] > MAXDIS && !((m_parvalue[em_DIS] > -DOUBLE_ZERO && (m_parvalue[em_DIS] < DOUBLE_ZERO) && (m_parvalue[em_ANGLE] > -DOUBLE_ZERO && m_parvalue[em_ANGLE] < DOUBLE_ZERO))))
			{
				SetVel(linear_velocity_left, linear_velocity_right);
			}
			//遇到障碍物停止状态
			if (sonardis[SERVOKEY_DOWN] <= MAXDIS)
			{
				SetVel(0, 0);
			}

			//无障碍物的情况下, 保持一种持续运行的状态
			if ((m_parvalue[em_DIS] > -DOUBLE_ZERO && (m_parvalue[em_DIS] < DOUBLE_ZERO) && (m_parvalue[em_ANGLE] > -DOUBLE_ZERO && m_parvalue[em_ANGLE] < DOUBLE_ZERO)) && sonardis[SERVOKEY_DOWN] > MAXDIS)
			{
				//printf("m_parvalue[em_SPEED]: %d\n", m_parvalue[em_SPEED]);
				SetVel(linear_velocity_left, linear_velocity_right);
				if ((m_parvalue[em_LINE] > -DOUBLE_ZERO && m_parvalue[em_LINE] < DOUBLE_ZERO) &&
					m_parvalue[em_REGULAR] > -DOUBLE_ZERO && m_parvalue[em_REGULAR] < DOUBLE_ZERO)
				{
					m_isRunning = false;
					m_middlestopflag = true;

					StopMotorRun();
					m_parvalue.clear();
					break;
				}
				else if (!(m_parvalue[em_LINE] > -DOUBLE_ZERO && m_parvalue[em_LINE] < DOUBLE_ZERO) || !(m_parvalue[em_REGULAR] > -DOUBLE_ZERO && m_parvalue[em_REGULAR] < DOUBLE_ZERO))
				{
					m_isRunning = true;
				}
			}
		}
	}
	// ros::Time after = ros::Time::now();
	// printf("时间差: %f\n ", after.toSec() - begin.toSec());
}

void ChassisNode::init()
{
	m_nh = NULL;
	m_scanthread = NULL;
	m_middlestopflag = true;
	m_webcontrol_flag = false;
	m_isornot_start_flag = true;
	linear_velocity_left = 0;
	linear_velocity_right = 0;

	// m_remote_cmd[KEY_TYPE] = 0;
	// m_remote_cmd[KEY_LINE] = 0;
	// m_remote_cmd[KEY_ANGULAR] = 0;
}

void ChassisNode::destroy()
{

	m_middlestopflag = false; //确保退出等待
	m_cv.notify_one();
	ros::shutdown();

	if (m_scanthread != NULL)
	{
		m_scanthread->join();
		delete m_scanthread;
		m_scanthread = NULL;
	}
	LOG(INFO) << "destroy...";

	for (int i = 0; i < MOTOR_NUM; i++)
	{
		if (m_motor[i] != NULL)
		{
			if (m_isFault[i]) //清除故障
				m_motor[i]->Clear_Fault();
			m_motor[i]->ShutDown_Motors(); //关闭电机

			delete m_motor[i];
			m_motor[i] = NULL;
		}
	}
}

void ChassisNode::InitParams(ros::NodeHandle *pnh)
{
	//开启串口
	m_port[0] = PORT_LEFTWHEEL;
	m_port[1] = PORT_RIGHTWHEEL;

	for (int i = 0; i < MOTOR_NUM; i++)
	{
		if (m_motor[i] == NULL)
		{
			m_motor[i] = new ChassisDriver(m_port[i]);
			m_motor[i]->Set_Velocity_Mode(); //设置速度模式
											 // m_motor[i]->Start_Motors();		 //启动电机
		}
		m_currentencodedvalue[i] = 0; //开始的编码值是0, 电机一启动就会清零
	}
}

void ChassisNode::RelativeSetDistanceSpeed(long distance, int speed, char dir)
{
	LOG(INFO) << "distance: ===================================" << distance;
	//设置距离cm, 针对位置模式, 距离有正负之分
	if (m_motor.size() < 0 || speed > MAXSPEED) //限制速度在100cm/s
		return;

	for (char i = 0; i < m_motor.size(); i++)
	{
		m_motor[i]->Set_Position_Mode(); //设置位置模式
		m_motor[i]->Set_Relative_Pos();  //设置相对位置
		m_motor[i]->Start_Motors();		 //启动电机
		m_motor[i]->Set_MaxH_Speed(speed);
		usleep(100);
		switch (dir)
		{
		case 's': //同向
			if (0 == i)
				m_motor[i]->Set_Distance(-distance);
			else if (1 == i)
				m_motor[i]->Set_Distance(distance);
			break;

		case 'r': //反向
			m_motor[i]->Set_Distance(-distance);
			break;
		}
	}
}

void ChassisNode::AbsoluteSetDistanceSpeed(long distance, int speed)
{
	//设置距离cm, 针对位置模式, 距离有正负之分
	if (m_motor.size() < 0 || speed > MAXSPEED) //限制速度在100cm/s
		return;

	for (char i = 0; i < m_motor.size(); i++)
	{
		m_motor[i]->Set_Position_Mode(); //设置位置模式
		m_motor[i]->Set_Absolute_Pos();  //设置绝对位置, 相对于电机开启的位置为零点值
		m_motor[i]->Set_MaxH_Speed(speed);
		m_motor[i]->Start_Motors(); //启动电机

		if (0 == i)
			m_motor[i]->Set_Distance(-distance);
		else if (1 == i)
			m_motor[i]->Set_Distance(distance);
	}
}

void ChassisNode::StartMotorRun()
{
	for (int i = 0; i < MOTOR_NUM; i++)
	{
		if (m_motor[i] != NULL)
		{
			m_motor[i]->Start_Motors(); //开启电机
			m_all_status[i] = m_motor[i]->Parse_Status();
			if ((m_all_status[i].size()) == 8)
			{
				while (m_all_status[i][STATUS(1)][1] == 0)
				{
					m_all_status[i] = m_motor[i]->Parse_Status();
					if ((m_all_status[i].size()) != 8)
						break;
					m_motor[i]->Start_Motors(); //开启电机
				}
			}
		}
	}
}

void ChassisNode::StopMotorRun()
{
	for (int i = 0; i < MOTOR_NUM; i++)
	{
		if (m_motor[i] != NULL)
		{
			// if (m_isFault[i]) //清除故障
			// 	m_motor[i]->Clear_Fault();
			m_motor[i]->ShutDown_Motors(); //关闭电机

			m_all_status[i] = m_motor[i]->Parse_Status();
			if ((m_all_status[i].size()) == 8)
			{
				while (m_all_status[i][STATUS(1)][1] != 0)
				{
					m_all_status[i] = m_motor[i]->Parse_Status();
					if ((m_all_status[i].size()) != 8)
						break;
					m_motor[i]->ShutDown_Motors(); //关闭电机
				}
			}
		}
	}
}

void ChassisNode::SetVel(double lvel, double rvel)
{
	// LOG(INFO)<<"lvel: "<<lvel;
	// LOG(INFO)<<"rvel: "<<rvel;
	if (m_motor[0] != NULL && m_motor[1] != NULL)
	{
		m_workmutex.lock();
		m_motor[0]->Set_Velocity(lvel);
		m_motor[1]->Set_Velocity(rvel);
		m_workmutex.unlock();
	}
}

std::map<std::string, double> ChassisNode::ParseSonarJson(std::string msg)
{
	std::map<std::string, double> val;
	Document document;
	document.Parse(msg.c_str());
	if (!document.HasParseError())
	{
		if (!ActionChecker::isSonarInfo(document))
			return val;

		val[KEY_LEFT] = document[KEY_LEFT].GetFloat();
		val[KEY_RIGHT] = document[KEY_RIGHT].GetFloat();
		val[KEY_UP] = document[KEY_UP].GetFloat();
		val[SERVOKEY_DOWN] = document[SERVOKEY_DOWN].GetFloat();
	}
	return val;
}

std::map<std::string, double> ChassisNode::ParseRemoteControl(std::string msg)
{

	std::map<std::string, double> val;
	Document document;
	document.Parse(msg.c_str());

	if (!document.HasParseError())
	{
		if (!ActionChecker::isRemoteControl(document))
			return val;

		val[KEY_LINE] = document[KEY_LINE].GetFloat();
		val[KEY_ANGULAR] = document[KEY_ANGULAR].GetFloat();
		val[KEY_PRIORITY] = document[KEY_PRIORITY].GetFloat();
	}
	return val;
}

std::vector<double> ChassisNode::ParseJson(std::string msg)
{
	std::vector<double> val;
	Document document;

	document.Parse(msg.c_str());

	if (!document.HasParseError())
	{
		if (!ActionChecker::isChassisAction(document))
			return val;

		val.push_back(document[KEY_DISTANCE].GetFloat());
		val.push_back(document[KEY_ANGLE].GetFloat());
		val.push_back(document[KEY_LINE].GetFloat());
		val.push_back(document[KEY_ANGULAR].GetFloat());
		val.push_back(document[KEY_PRIORITY].GetFloat());
	}
	return val;
}

/************************************************************************************
 * 定时回调函数     END
 * **********************************************************************************/

} // namespace robot

using namespace robot;

ChassisNode chassisNode;

void signalHandler(int signum)
{
	chassisNode.destroy();
	exit(signum);
}

int main(int argc, char **argv)
{
	LogHelper m_Log(argv[0]);
	chassisNode.run(argc, argv);
	return 0;
}

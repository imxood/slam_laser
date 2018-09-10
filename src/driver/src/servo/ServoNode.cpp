#include <robot_driver/servo/ServoNode.h>

namespace robot
{

atomic<bool> ServoNode::isRunning(false);

ServoNode::ServoNode() : servoDriver(NULL), statusThread(NULL), nh(NULL)
{
	isRunning = true;
}

ServoNode::~ServoNode()
{
	InitStatus(); // 回到初始状态

	if (ros::ok())
		ros::shutdown(); // 关闭当前节点

	if (servoDriver != NULL)
	{
		delete servoDriver;
		servoDriver = NULL;
	}

	if (statusThread != NULL) // 关闭相关线程
	{
		statusThread->join();
		delete statusThread;
		statusThread = NULL;
	}

	if (nh)
	{
		delete nh;
		nh = NULL;
	}
}

void ServoNode::init(int argc, char **argv)
{
	init_ros(argc, argv);

	init_devices();

	/** 初始化ros参数 */
	// ParamServerIns.param("/servo/head_ver_angle/max", servoDevices[HEAD_VER_ID]->maxAzimuth, 20);
	// ParamServerIns.param("/servo/head_ver_angle/min", servoDevices[HEAD_VER_ID]->minAzimuth, -20);

	// ParamServerIns.param("/servo/head_hor_angle/max", servoDevices[HEAD_HOR_ID]->maxAzimuth, 60);
	// ParamServerIns.param("/servo/head_hor_angle/min", servoDevices[HEAD_HOR_ID]->minAzimuth, -60);

	// ros::ServiceServer service = nh.advertiseService(SERVICE_SERVO, &ServoNode::ServiceCallback, this);

	/** 舵机驱动控制 */
	servoDriver = new ServoDriver(PORT_SERVO, 115200, 5, 5); // 读写5ms超时

	/** 检测在线舵机 */
	servoDriver->Pings(servoDevices);

	/** 初始化各舵机的位置 */
	InitStatus();

	/** 开启状态监控线程 */
	statusThread = new std::thread(&ServoNode::StatusScan, this);
}

/** 初始化ros节点 */
void ServoNode::init_ros(int argc, char **argv)
{
	ros::init(argc, argv, "ServoNode");
	nh = new ros::NodeHandle();

	std::string servoTopicName, sonarTopicName, resetTopicName;

	ParamServerIns.param("/topic/servo/in/topic_name", servoTopicName, "/robot/servo/in");
	ParamServerIns.param("/topic/sensor/sonar/topic_name", sonarTopicName, "/robot/sensor/sonar");
	ParamServerIns.param("/topic/servo/reset/topic_name", resetTopicName, "/robot/reset/servo");

	/** 消息订阅 */
	subSonar = nh->subscribe(sonarTopicName, 1, &ServoNode::SonarTopicCallback, this);
	subAction = nh->subscribe(servoTopicName, 30, &ServoNode::ActionCallback, this);
	subReset = nh->subscribe(resetTopicName, 30, &ServoNode::ResetCallback, this);
}

/** 初始化设备列表 */
void ServoNode::init_devices()
{
	map<string, double> numberParam;
	ros::param::get("/servo/head_hor", numberParam);

	int num;
	ros::param::get("/servo/head_hor/init", num);

	servoDevices[HEAD_HOR_ID] = new ServoDevice(HEAD_HOR_ID, 1700, 1300, 2100, "头部水平舵机HEAD_HOR", 1);
	servoDevices[HEAD_VER_ID] = new ServoDevice(HEAD_VER_ID, 1700, 1700, 1900, "头部竖直舵机HEAD_VER", 1);
	servoDevices[LEFTARM_1_ID] = new ServoDevice(LEFTARM_1_ID, LEFTARM_1_ZERO, 0, 0, "左臂舵机LEFTARM_1", 1);
	servoDevices[LEFTARM_2_ID] = new ServoDevice(LEFTARM_2_ID, LEFTARM_2_ZERO, 0, 0, "左臂舵机LEFTARM_2", 1);
	servoDevices[LEFTARM_3_ID] = new ServoDevice(LEFTARM_3_ID, LEFTARM_3_ZERO, 0, 0, "左臂舵机LEFTARM_3", 1);
	servoDevices[RIGHTARM_1_ID] = new ServoDevice(RIGHTARM_1_ID, RIGHTARM_1_ZERO, 0, 0, "右臂舵机RIGHTARM_1", 1);
	servoDevices[RIGHTARM_2_ID] = new ServoDevice(RIGHTARM_2_ID, RIGHTARM_2_ZERO, 0, 0, "右臂舵机RIGHTARM_2", 1);
	servoDevices[RIGHTARM_3_ID] = new ServoDevice(RIGHTARM_3_ID, RIGHTARM_3_ZERO, 0, 0, "右臂舵机RIGHTARM_3", 1);
	servoDevices[WAIST_ID] = new ServoDevice(WAIST_ID, WAIST_ZERO, 0, 0, "腰部舵机WAIST", 1);
}

/** 设置初始状态 */
void ServoNode::InitStatus()
{
	// Pings
	for (auto &d : servoDevices)
	{
		auto &servoDevice = *d.second;
		if (servoDevice.isOnline)
		{
			servoDriver->SetDeviceAzimuth(servoDevice.id, servoDevice.initAzimuth, 0, SERVO_SPEED);
		}
	}
}

void ServoNode::run()
{
	ros::Rate loop_rate(20);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
}

void ServoNode::ParseJson(std::string json, std::vector<short> &buffer)
{
	Document document;
	document.Parse(json.c_str());
	if (!document.HasParseError())
	{
		if (!ActionChecker::isServoAction(document))
			return;

		buffer.push_back((short)document[KEY_ID].GetFloat());
		buffer.push_back((short)document[KEY_ANGLE].GetFloat());
		buffer.push_back((short)document[KEY_SPEED].GetFloat());
	}
	else
	{
		LOG(WARNING) << "ServoNode Action数据格式错误";
	}
}

std::string ServoNode::BuildJson(unsigned char id, float angle, float utime, float speed)
{
	Document document;											  //生成DOM元素Document
	Document::AllocatorType &allocator = document.GetAllocator(); //获取分配器
	document.SetObject();

	document.AddMember(KEY_ID, id, allocator);
	document.AddMember(KEY_ANGLE, angle, allocator);
	document.AddMember(KEY_TIME, utime, allocator);
	document.AddMember(KEY_SPEED, speed, allocator);

	//生成字符串
	StringBuffer buff;
	Writer<StringBuffer> writer(buff);
	document.Accept(writer);
	// LOG(INFO) << "ServoNode: " << buff.GetString();
	return buff.GetString();
}

void ServoNode::AssembleCmd(std::vector<short> &cmd)
{
	short targetIncrementValue = 0;
	if (cmd.size() != 3)
		return;

	if (cmd[em_SPEED] <= 0 || cmd[em_SPEED] > 1000) //速度设置为零时，对应最大速度
	{
		LOG(WARNING) << "速度设置超出(0-1000)的范围";
		return;
	}

	if (servoDevices.find(cmd[em_ID]) == servoDevices.end())
	{
		LOG(WARNING) << "设备不存在";
		return;
	}

	auto &device = *servoDevices[cmd[em_ID]];

	if (!device.isOnline)
	{
		LOG(WARNING) << "设备不在线";
		return;
	}

	servoDriver->GetDeviceAzimuth(device);

	// LOG(INFO) << "当前Azimuth: " << device.currentAzimuth;

	unsigned short wantAzimuth = device.currentAzimuth + ANGLE_INCREMENT(cmd[em_ANGLE]) * device.multiplier;

	// LOG(INFO) << "想要Azimuth: " << wantAzimuth << ", cmd[em_ANGLE]: " << cmd[em_ANGLE];

	unsigned short azimuth = LIMIT_ANGLE(wantAzimuth, device.minAzimuth, device.maxAzimuth);

	servoDriver->SetDeviceAzimuth(device.id, azimuth, 0, cmd[em_SPEED]);
}

void ServoNode::ActionCallback(const std_msgs::String::ConstPtr &json)
{
	// LOG(INFO) << "操纵舵机指令: " << json->data;

	static std::vector<short> buffer;
	buffer.clear();

	ParseJson(json->data, buffer);

	AssembleCmd(buffer);
}

void ServoNode::ResetCallback(const std_msgs::String::ConstPtr &json)
{
}

// bool ServoNode::ServiceCallback(robot_common::currentAzimuth::Request &req, robot_common::currentAzimuth::Response &res)
// {
// 	res.AzimuthValue = servoDriver->SXS_CurrentAzimuth(req.id);
// 	printf("req.id:   %d\n", req.id);
// 	return true;
// }

std::map<std::string, double> ServoNode::ParseSonarJson(std::string json)
{
	std::map<std::string, double> val;
	Document document;
	document.Parse(json.c_str());
	if (!document.HasParseError())
	{
		if (!ActionChecker::isSonarInfo(document))
			return val;

		val[KEY_LEFT] = document[KEY_LEFT].GetFloat();
		val[KEY_RIGHT] = document[KEY_RIGHT].GetFloat();
		val[KEY_UP] = document[KEY_UP].GetFloat();
		val[STEERKEY_DOWN] = document[STEERKEY_DOWN].GetFloat();
	}
	return val;
}

void ServoNode::SonarTopicCallback(const std_msgs::String::ConstPtr &msg)
{
	sonardis = ParseSonarJson(msg->data);

	// m_sonarmsg.ldistance = msg->ldistance;
	// m_sonarmsg.mdistance = msg->mdistance;
	// m_sonarmsg.rdistance = msg->rdistance;
	// m_sonarmsg.bdistance = msg->bdistance;
}

/** 检测过压,过流等 */
void ServoNode::StatusScan()
{
	unsigned short currentAzimuth = 2048; //舵机当前值

	static uint8_t time = 0;

	while (isRunning && ros::ok())
	{
		if (++time > 5) // 每隔 5*1s, 执行一次状态监控
		{
			servoDriver->Pings(servoDevices);

			int size = 0;

			for (auto &it : servoDevices)
			{
				if (it.second->isOnline)
				{
					servoDriver->CheckStatus(*it.second);
					size++;
				}
			}

			LOG(INFO) << "在线舵机数量: " << size;

			time = 0;
		}

		this_thread::sleep_for(chrono::seconds(1));
	}
}

} // namespace robot

using namespace robot;

void signalHandler(int signum);

int main(int argc, char *argv[])
{
	LogHelper log(argv[0]);

	signal(SIGINT, signalHandler); //接收到ctrl+c的int信号

	ServoNode servoNode;

	servoNode.init(argc, argv);
	servoNode.run();

	LOG(INFO) << "ServoNode 退出";

	return 0;
}

void signalHandler(int signum)
{
	ServoNode::isRunning = false;
}

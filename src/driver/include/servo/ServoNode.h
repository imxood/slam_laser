#include <thread>
#include <map>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <csignal>

#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/String.h>

#include <robot_common/pkrobotdef.h>
#include <robot_common/JsonHelper.h>
#include <robot_common/ActionChecker.h>
#include <robot_common/param/ParamServer.h>
#include <robot_common/SoundwaveMsg.h>

#include <robot_driver/servo/ServoDriver.h>

using namespace NS_Servo;

namespace robot
{

#define SERVO_SPEED 200 //初始化时的舵机运行速度
#define SERVO_TIME 0	//舵机运转时间

#define SPEED_DU_EVERY_S(speed) (0.19 * speed)		 //每秒转多少度
#define DU_ACCURACY (4096 / 360.0)					 //角度精度值
#define ANGLE_INCREMENT(angle) (angle * DU_ACCURACY) //angle角度实际编码值增量

/** 限制角度 */
#define LIMIT_ANGLE(angle, min, max) ((angle > min) ? (angle > max ? max : angle) : min)

#define WAIT_TIME(INCREMENT, speed) ((ANGLE_INCREMENT(INCREMENT) / SPEED_DU_EVERY_S(speed)) * 1000000) //转增量角度需要耗时多少ms

//小舵机->手指舵机SCS
#define FINGER_ACCURACY (1024 / 280.0)
#define FINGER_ANGLE_INCREMENT(angle) (angle * FINGER_ACCURACY)

//port
#define PORT_SERVO "/dev/port_servo"
// #define PORT_SERVO "/dev/ttyUSB0"

//service
#define SERVICE_SERVO "/service/azimuth"

#define ISORNOTONLINE(flag) (flag ? "在线" : "掉线")
#define ANGLE_OVERSCOPE(maxangle, minangle) "角度超出范围[" #maxangle ", " #minangle "]"

//json字段
#define KEY_ID "id"
#define KEY_ANGLE "angle"
#define KEY_TIME "time"
#define KEY_SPEED "speed"

#define KEY_LEFT "left"
#define KEY_RIGHT "right"
#define KEY_UP "up"
#define STEERKEY_DOWN "down"

typedef unsigned char UCHAR;

enum
{
	em_ID,
	em_ANGLE,
	em_SPEED
};

class ServoNode
{
	ros::NodeHandle *nh;

	robot_common::SoundwaveMsg m_sonarmsg;

	map<DeviceIdType, ServoDevice *> servoDevices; // 所有的舵机设备

	int m_vermax, m_vermin;
	int m_hormax, m_hormin;

	bool m_isrecivecmd; //是否接收指令

	std::thread *statusThread; // 状态监控线程
	std::mutex m_mutex;
	std::condition_variable m_cv;

	std::map<int, std::string> mapServoDescption;	  //舵机警告标示
	std::map<unsigned char, unsigned char> m_onlinese; //在线的舵机

	ServoDriver *servoDriver; // 舵机控制

	ros::Subscriber subAction, subSonar, subReset; //动作数据, 声呐数据, 重置servo

	//超声波数据
	std::map<std::string, double> sonardis;

  public:
	ServoNode();
	~ServoNode();

	void init(int argc, char **argv);

	/** 初始化ros节点 */
	void init_ros(int argc, char **argv);

	/** 初始化设备列表 */
	void init_devices();

	void InitStatus();

	void run();

	static atomic<bool> isRunning; // 运行状态

  private:
	/** 检测过压,过流等 */
	void StatusScan();

	void ActionCallback(const std_msgs::String::ConstPtr &msg);
	void ResetCallback(const std_msgs::String::ConstPtr &json);
	// bool ServiceCallback(robot_common::currentAzimuth::Request &req, robot_common::currentAzimuth::Response &res);

	//传感器信息
	void SonarTopicCallback(const std_msgs::String::ConstPtr &msg);

	std::map<std::string, double> ParseSonarJson(std::string json);
	void ParseJson(std::string, std::vector<short> &buffer);
	std::string BuildJson(unsigned char id, float angle, float utime, float speed);

	/**
	 * 组装需要下发的指令
	 */
	void AssembleCmd(std::vector<short> &cmd);

	/**
	 * 判断给定目标值相对于当前的差值,以此决定是否下发给舵机指令
	 * increment: 根据角度值获得的编码增量值
	 * 返回值: 
	 * 		true:下发指令, false: 不下发
	*/
	bool IsOrNotPublishCmd(short &id, short &increment);

};

} // namespace robot

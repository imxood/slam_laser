#ifndef _CONTROL_SERVO_H
#define _CONTROL_SERVO_H

#include <robot_driver/chassis/ChassisDriver.h>
// #include <robot_driver/Odometry/odometry.h>
#include <robot_driver/Odometry/diff_control.h>

#include <std_msgs/String.h>
#include <csignal>
#include <sstream>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <ros/forwards.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <robot_common/pkrobotdef.h>

#include <robot_common/SoundwaveMsg.h>
#include <robot_common/JsonHelper.h>
#include <robot_common/ActionChecker.h>
//get param header file
#include <robot_common/param/ParamServer.h>

using namespace rapidjson;
using namespace Motorchassis;

namespace robot
{
#define MOTOR_NUM 2

//速度上限值设定不超过MAXSPEED, 单位cm/s
// #define MAXSPEED 100

//声呐最小距离m
#define MAXDIS 0.100000

//减速比reduction gear ratio
#define REDUCTION_GEAR_RATIO 0.98

//减速度, 确保3秒停止 1/2*a*t*t + v*t = distance
/**
 * 现在确定了t=3
 * 9/2*a + 3v = distance
 * 又知
 * v = a*t = 3a;
 * 综合上述:
 * a= 2*distance/27
*/
#define REDUCTION_SPEED(distance) (2 * distance / 27.0)

//减速开始的距离, 按照设定距离百分之30做减速处理, chavalue=距离差值
#define DECELDIS(chavalue) (double)(chavalue * 0.100000)

#define FORWARD 'f'
#define BACK 'b'

#define MAXSPEED 50.000001
#define MINSPEED -0.000001

#define ALLOW_MAXSPEED 1.000001
#define DOUBLE_ZERO 0.000001

#define KEY_TYPE "topic_id"
#define KEY_DISTANCE "distance"
#define KEY_ANGLE "angle"
#define KEY_L_SPEED "lspeed"
#define KEY_R_SPEED "rspeed"
#define KEY_PRIORITY "priority"

//远程遥控的线速度与角速度
#define KEY_LINE "line"
#define KEY_ANGULAR "angular"

#define KEY_LEFT "left"
#define KEY_RIGHT "right"
#define KEY_UP "up"
#define SERVOKEY_DOWN "down"

/***位置模式下:
 * 右侧轮子m_motor[0]:  
 * distance>0: 后退 
 * distance<0: 前进
 * 左侧轮子m_motor[1]: 
 * distance>0: 前进 
 * distance<0: 后退 
 * */

/***速度模式下
 * 右侧轮子m_motor[0]: 
 * speed>0: 后退
 * speed<0: 前进
 * 左侧轮子m_motor[1]:
 * speed>0: 前进
 * speed<0: 后退 
*/

typedef std::pair<unsigned short, unsigned int> PAIR;
typedef unsigned short USHORT;

//伺服电机的端口设置
#define PORT_LEFTWHEEL "/dev/port_leftwheel"
#define PORT_RIGHTWHEEL "/dev/port_rightwheel"

class ChassisNode
{
  public:
	ChassisNode();
	~ChassisNode();

	void init();
	void destroy();
	void InitParams(ros::NodeHandle *);

  private:
	//里程计信息的发布
	// Odometry odometry;
	// //差速的计算
	DiffControl m_diff_control;
	double linear_velocity_left, linear_velocity_right;
	bool m_webcontrol_flag;
	bool m_isornot_start_flag;

	enum
	{
		em_TYPE,
		em_DIS,
		em_ANGLE,
		em_LINE,
		em_REGULAR,
		em_PRI
	};
	std::mutex m_workmutex; //线程锁
	std::thread *m_scanthread;
	//超声波数据
	std::map<std::string, double> sonardis;

	std::mutex m_mutex;
	std::condition_variable m_cv;

	//ros参数
	ros::NodeHandle *m_nh;
	// ros::Publisher m_servo_publisher;
	robot_common::SoundwaveMsg m_sonarmsg;

	//储存此时电机的编码值
	std::vector<long> m_currentencodedvalue;
	//接收topic发来的数据
	std::vector<double> m_parvalue;
	std::map<std::string, double> m_remote_cmd;

	std::vector<float> m_motorspeed; //电机速度m/s
	std::vector<char> m_posneg;		 //判断电机转向的正负
	std::vector<std::map<unsigned char, std::vector<unsigned char>>> m_all_status;

	static std::vector<bool> m_isFault;   //储存motor是否发生故障
	std::vector<ChassisDriver *> m_motor; //储存motor    //需要添加motor时，也要增加port， MOTOR_NUM也需要修改， 三者要一一对应
	std::vector<const char *> m_port;	 //储存串口 port与motor个数要一致

	//是否右障碍物
	bool m_hasObstacle;

	//电机需要运行的距离,根据设定的距离与角度得到
	double m_distance;

	//设定速度值
	double m_line_velocity;
	double m_angular_velocity;

	//是否一直运行
	bool m_isRunning;

	//中间停止状态标志位
	bool m_middlestopflag;

  private:
	std::string m_servotopic, m_sonartopic, m_webservotopic;

  private:
	//callback 处理速度的回调
	void VelocityCallBack(const std_msgs::String::ConstPtr &msg);
	//callback 处理远程(web)控制的调用
	void ServoWebControlTopicCallback(const std_msgs::String::ConstPtr &json);
	//传感器信息
	void SonarTopicCallback(const std_msgs::String::ConstPtr &msg);

	void ScanThread();

	//超声波数据
	std::map<std::string, double> ParseSonarJson(std::string json);

	//web端遥控数据解析
	std::map<std::string, double> ParseRemoteControl(std::string json);

	//动作解析
	std::vector<double> ParseJson(std::string);

	/**相对位置模式下, 两个电机同速同距离情况下的设置
	 * distance: 运行距离, 单位为cm
	 * speed: 运行速度, 单位为cm/s
	 * dir: 两个轮子的转向, 同向: s,反向: r
	*/
	void RelativeSetDistanceSpeed(long distance, int speed, char dir);
	/**绝对位置模式下, 两个电机同速同距离情况下的设置, 
	 * 用此函数要注意: 电机启动的时刻为绝对位置的零点值
	*/
	void AbsoluteSetDistanceSpeed(long distance, int speed);

	void StopMotorRun();
	void StartMotorRun();

  private:
	//在需要传入正负值的速度指令时
	void SetVel(double lvel, double rvel);

  private:
	void StatusCallback();

  public:
	void run(int argc, char **argv);
};
} // namespace robot

#endif //_CONTROL_SERVO_H

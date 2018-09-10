#pragma once

#include <iostream>
#include <thread>
#include <mutex>

#include <signal.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

#include <robot_driver/chassis/WheelDriver.h>
#include <robot_driver/chassis/ChassisControl.h>
#include <robot_driver/odometry/DiffDrive.h>
#include <robot_driver/odometry/Odometry.h>

#include <robot_common/pkrobotdef.h>
#include <robot_common/JsonHelper.h>
#include <robot_common/LogHelper.h>
#include <robot_common/ActionChecker.h>

using namespace std;

namespace robot
{

class ChassisNode
{
  private:
	ros::NodeHandle *nh;

	thread *minitorThread;
	ChassisControl chassisControl;

  public:
	ChassisNode();
	~ChassisNode();

	/** 运行状态 */
	static atomic<bool> isRunning;

	/** 初始化: ros, 轮子驱动, 差速驱动, 里程计 */
	void init(int argc, char **argv);

	/** ros消息处理 */
	void run(); // 主循环

	/** 测试 */
	void test();

  private:
	/** 轮子状态监控线程函数 */
	void minitorLoop();

	/** 调度请求, 回调函数 */
	void scheduleCallback(const std_msgs::StringConstPtr &msg);

	/** 速度请求, 回调函数 */
	void velocityCallback(const geometry_msgs::TwistConstPtr &msg);

	/** 超声波回调 */
	void sonarCallback(const std_msgs::StringConstPtr &msg);
};

} // namespace robot

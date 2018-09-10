#pragma once

#include <mutex>
#include <cmath>

#include <sensor_msgs/JointState.h>

#include <robot_driver/chassis/WheelDriver.h>
#include <robot_driver/odometry/DiffDrive.h>
#include <robot_driver/odometry/Odometry.h>

#include <robot_common/CommonType.h>

namespace robot
{

enum ChassisOperate
{
	Forwarding,
	Velocitying,
	Rotating
};

/**
 * 底盘控制
 */
class ChassisControl
{
  private:
	double wheelRadius; // 轮子半径
	double bias;		// 轴距, 两轮间距
	double radPerTick;  // 每个编码值代表的弧度

	WheelDriver leftWheel;  // 左轮
	WheelDriver rightWheel; // 右轮

	DiffDrive diffDrive; // 差速驱动用于获取前后两个时刻的姿态信息
	Odometry *odometry;  // 使用差速的姿态信息更新并发布里程计信息

	ros::Publisher joint_state_publisher; // 发布机器人的关节状态信息
	ros::Publisher run_state_publisher;   // 发布机器人的运行状态

	RunState runState;			   // 运行状态, 空闲状态: IDLE, BUSY: 执行任务, CLOSED: 已关闭
	ChassisOperate chassisOperate; // 轮子转动模式: Position, Speed, Rotate

	sensor_msgs::JointState joint_states; // 关节状态
	std::recursive_mutex control_mutex;   // 用于保证: 速度设置,距离设置,差速控制,同一时刻只运行一种

  public:
	ChassisControl(double wheelRadius, double bias, double radPerTick);
	~ChassisControl();

	/** 初始化 */
	void Init(ros::NodeHandle &nh);

	/** 读取编码器信息, 并更新里程信息 */
	void Update(WheelState &leftState, WheelState &rightState);

	/** 关闭轮子 */
	void Close();

	/** 重置里程计超时时间(超时, 则机器人速度就设置为0) */
	void ResetTimeout();

	/** 速度, 单位: m/s, rad/s */
	void Velocity(double linearVelocity, double angularVectlor);

	/** 前进, 单位: 米 */
	void Forward(double distance);

	/** 旋转 */
	void Rotate(double angle);

	/** 停止 */
	void Stop();

	/** 获取运行状态 */
	RunState GetRunState();

  private:
	/** 更新运行状态 */
	void UpdateRunState(RunState runState);

	/** 
	 * 错误检查, 主要是为了解决端口重新插拔
	 * 左轮端口有错误, 会重启右轮端口, 因为左轮有错误发生时, 会有等待解决时间, 直到解决完成, 才会执行到右轮, 此时的右轮可能是原来打开的句柄fd, 需要重新打开
	 */
	void checkError();
};

} // namespace robot

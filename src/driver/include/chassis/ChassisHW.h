#pragma once

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <controller_manager/controller_manager.h>

#include <ecl/config.hpp>
#include <ecl/threads.hpp>
#include <ecl/devices.hpp>
#include <ecl/threads/mutex.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/sigslots.hpp>

#include <robot_driver/parameters.h>
#include <robot_driver/wheel_data.h>

#include <robot_common/LogHelper.h>

namespace robot
{

//底盘控制
//三个轮子: 两个驱动轮,一个万向轮
class ChassisHW : public hardware_interface::RobotHW
{
  public:
	ChassisHW() : tick_to_rad(0.001533980786f), wheel_radius(0.085) {}

	void init()
	{
		LOG(INFO) << "ChassisHW::init";

		std::fill_n(cmd_, 2, 0);
		std::fill_n(pos_, 2, 0);
		std::fill_n(vel_, 2, 0);
		std::fill_n(eff_, 2, 0);

		//注册相关HardWare接口

		const char *left_wheel_joint = "left_wheel";
		const char *right_wheel_joint = "right_wheel";
		const char *ommi_wheel_joint = "ommi_wheel";

		//注册关节状态接口
		hardware_interface::JointStateHandle state_handle_left(left_wheel_joint, &pos_[0], &vel_[0], &eff_[0]);
		jnt_state_interface_.registerHandle(state_handle_left);

		hardware_interface::JointStateHandle state_handle_right(right_wheel_joint, &pos_[1], &vel_[1], &eff_[1]);
		jnt_state_interface_.registerHandle(state_handle_right);

		hardware_interface::JointStateHandle state_handle_ommi(ommi_wheel_joint, &pos_[2], &vel_[2], &eff_[2]);
		jnt_state_interface_.registerHandle(state_handle_ommi);

		registerInterface(&jnt_state_interface_);

		//注册关节控制接口
		hardware_interface::JointHandle pos_handle_left(jnt_state_interface_.getHandle(left_wheel_joint), &cmd_[0]);
		jnt_vel_interface_.registerHandle(pos_handle_left);

		hardware_interface::JointHandle pos_handle_right(jnt_state_interface_.getHandle(right_wheel_joint), &cmd_[1]);
		jnt_vel_interface_.registerHandle(pos_handle_right);

		registerInterface(&jnt_vel_interface_);
	}

	ros::Time get_time() const { return ros::Time::now(); }

	ros::Duration get_period() const { return ros::Duration(0.01); }

	//读出差速控制指令
	void read(/*double& l_pos, */ float &l_speed /*, double& r_pos*/, float &r_speed)
	{
		//        l_pos = pos_[0];
		//        r_pos = pos_[1];
		l_speed = cmd_[0];
		r_speed = cmd_[1];
	}

	//写入关节状态
	void write(int &left_encoder, int &right_encoder)
	{
		pos_[0] = left_encoder * tick_to_rad;
		pos_[1] = right_encoder * tick_to_rad;
	}

  private:
	double cmd_[3]; //速度控制(m/s)
	double pos_[3]; //位置状态(rad)
	double vel_[3]; //速度状态(rad/s)
	double eff_[3]; //

	const double tick_to_rad;
	const double wheel_radius;

	hardware_interface::JointStateInterface jnt_state_interface_;
	hardware_interface::VelocityJointInterface jnt_vel_interface_;

	ecl::Signal<const ChassisJointState &> sig_chassis_joint_state;
};

} // namespace robot

#ifndef _CHASSISDRIVER_H
#define _CHASSISDRIVER_H

#include <serial/serial.h>
#include <map>
#include <vector>
#include <iostream>
#include <stdio.h>
#include <exception>
#include <unistd.h>


#include <robot_common/LogHelper.h>

namespace Motorchassis
{

typedef unsigned char BYTE;
typedef unsigned int UINT;

/**RS232通讯格式
 * 数据地址  数据高八位 数据低八位  数据校验和
 * A1		A2		 A3		    A1+A2+A3
*/
//电机启动
static const BYTE motor_run[8] = {0x00, 0x00, 0x01, 0x01};
//电机停止
static const BYTE motor_stop[8] = {0x00, 0x00, 0x00, 0x00};
//速度模式选择-PC数字输入
static const BYTE speed_mode[8] = {0x02, 0x00, 0xc4, 0xc6};

//位置模式选择-外部脉冲输入
static const BYTE position_mode_pulse[8] = {0x02, 0x00, 0xc0, 0xc2};
//位置模式选择-PC数字输入
static const BYTE position_mode_pc[8] = {0x02, 0x00, 0xd0, 0xd2};

//速度比例增益
// static const BYTE speed_proportion_gain[8] = {0x40, --, --, --};
//速度积分增益
// static const BYTE speed_integration_gain[8] = {0x41, --, --, --};
//速度微分增益
// static const BYTE speed_differential_gain[8] = {0x42, --, --, --};

//位置比例增益
// static const BYTE position_proportion_gain[8] = {0x1a, --, --, --};
//位置微分增益
// static const BYTE position_differential_gain[8] = {0x1b, --, --, --};
//位置前馈增益
// static const BYTE position_feedforward_gain[8] = {0x1c, --, --, --};

//速度模式(PC数字输入时有效)--加减速度设定
// static const BYTE speed_updown_set[8] = {0x0a, --, --, --};
//位置调试模式位置给定高16位--PC--位置
// static const BYTE position_hight16_set[8] = {0x50, --, --, --};
//位置调试模式位置给定低16位--PC--位置
// static const BYTE position_low16_set[8] = {0x05, --, --, --};
//速度调试模式--PC数字输入
// static const BYTE speed_debug_mode[8] = {0x06, --, --, --};
//寻找z信号机械原点
// static const BYTE find_zsignal_origin[8] = {0x53, 0x00, 0x00, 0x53};
//故障清除
static const BYTE clear_fault[8] = {0x4a, 0x00, 0x00, 0x4a};

//位置模式下绝对位置切换控制
//绝对位置
static const BYTE absolute_position[8] = {0x51, 0x00, 0x00, 0x51};
//相对位置
static const BYTE relative_position[8] = {0x51, 0x00, 0x01, 0x52};

//位置模式下的速度限幅值(位置命令下达到给定位置的实际转速)
//static const BYTE position_speed_amplitude_limiting[8] = {0x1d, --, --, --};

//读监控参数
static const BYTE monitor_cmd[3] = {0x80, 0x00, 0x80};

//状态反馈
#define STATUS_OV_I(status_word) 		(((status_word)^1) == 0x03) //过流
#define STATUS_OV_U(status_word) 		(((status_word)^2) == 0x06) //过压
#define STATUS_ERR_ENC(status_word)  	(((status_word)^3) == 0x0b) //编码器故障
#define STATUS_OV_Q(status_word) 		(((status_word)^5) == 0x30) //欠压
#define STATUS_OV_LOAD(status_word) 	(((status_word)^6) == 0x60) //过载


#define STATUS(flag)					(flag ? 0x80 : 0) //状态信息
#define BUSBAR_VOLTAGE(flag) 			(flag ? 0xE1 : 4) //母线电压
#define OUTPUT_CURRENT(flag)			(flag ? 0xE2 : 8)//输出电流
#define ROTATION_RATE(flag)      		(flag ? 0xE4 : 12)//转速
#define	POSITION_GIVEN_HIGH16(flag) 	(flag ? 0xE6 : 16) //位置给定高16bits
#define	POSITION_GIVEN_LOW16(flag)   	(flag ? 0xE7 : 20) //位置给定低16bits
#define	POSITION_FEEDBACK_HIGH16(flag) 	(flag ? 0xE8 : 24)//位置回馈高16bits
#define POSITION_FEEDBACK_LOW16(flag)  	(flag ? 0xE9 : 28)//位置回馈低16bits


//车轮周长m
#define WHEEL_CIRCUM 					0.5338
//车体两轮之间的距离cm
#define DISBETWEENBOTHMOTOR				51.0


//value是监控状态下,读出的设定值, 计算结果是实际转速, //实际转速最大为24000
#define REAL_ROTATION_RATE(value) 	  	(value*6000/16384.0)				
//value是监控状态下,读出的设定值, 计算结果是实际速度, 0.53m为车轮直径, // 转速转速度m/s 212
#define ROTATION_RATE2SPEED(value) 		(value*6000.0 / 16384 * WHEEL_CIRCUM / 60) 
//m/min米每秒 velocity单位是m/s
#define SPEED_PERMINITES(velocity) 		(((velocity * 60 / WHEEL_CIRCUM) / 6000.0 * 16384)) 	


//求转动相应的距离之后, 电机的位置给定编码值  distance单位为m
#define GETCODEDVALUEBY(distance)       ((distance/(WHEEL_CIRCUM))*4096)
//得到位置模式下根据限幅转速, 得到最大需要设定的值,
#define GETMAXSETVALUE(value)           (value/6000.0*16384.0) 

//线速度 = 周长*转速
//转速 = 线速度 / 周长
//speed为线速度cm/s, 结果转速是rpm
#define ROTATION_SPEED(speed) 			(speed/(WHEEL_CIRCUM*100))*60

//根据编码值求距离cm
#define GETDISBYCODEDVALUE(value)		(value/(4096/52.1))


//由给定角度值求距离m, 原地转的时候的圆的周长 (6.28*DISBETWEENBOTHMOTOR/2)
#define GETDISBY(angle)					((angle*((6.28*DISBETWEENBOTHMOTOR/2)/360.0))/100.0)
//两字节拼
#define H8_L8(MAP, WHO)					((MAP[WHO][0] << 8)|((MAP[WHO][1]) & 0xFF))
//四字节拼
#define H16_L16(H, L) 					((H << 16) | (L & 0xFFFF))

class ChassisDriver
{
  public:
	ChassisDriver(const char *Port, UINT BaudRate = 57600, UINT Timeout = 100);
	~ChassisDriver();

  private:
	serial::Serial *m_serial;
	std::string m_servoport;
	UINT m_bardrate;
	UINT m_timeout;
	//串口操作
	void Serial_Open(const char *Port, UINT BaudRate = 57600, UINT Timeout = 100);
	void Serial_Close();

	bool Serial_Read(BYTE *buf, long len);
	unsigned short Serial_Write(const BYTE *buf, long len);

	bool WriteCmd(const BYTE *cmd);
  public:

	//开启电机
	bool Start_Motors();
	//关闭电机
	bool ShutDown_Motors();
	//设置速度模式
	bool Set_Velocity_Mode();

	//设置位置模式
	bool Set_Position_Mode();
	//设置最高稳定限速值 cm/s
	bool Set_MaxH_Speed(uint16_t);
	//设置相对位置
	bool Set_Relative_Pos();
	//设置绝对位置
	bool Set_Absolute_Pos();
	//清除故障
	bool Clear_Fault();
	//监控状态
	void SendMonitorCommand();
	//读取状态
	void Read_Status(BYTE *data, UINT dataToRead, UINT &dataReaded);

	//解析状态数据
	/**
	 * 0x80 状态信息
	 * 0xE1 母线电压
	 * 0xE2 输出电流
	 * 0xE4 转速
	 * 0xE6 位置给定高16位
	 * 0xE7 位置给定低16位
	 * 0xE8 位置反馈高16位
	 * 0xE9 位置反馈低16位
	*/
	std::map<unsigned char, std::vector<unsigned char>> Parse_Status();
	//状态判断, true: 无故障， false: 有故障
	bool MotorStatus_Judge(BYTE status);

	//设置速度,speed: m/s
	void Set_Velocity(double velocity);
	//设置位置 cm
	void Set_Distance(int32_t);

	void Set_CodedValue(int32_t);
	
};
}

#endif

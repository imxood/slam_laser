#pragma once

#include <iostream>
#include <mutex>

#include <robot_driver/chassis/WheelProtocal.h>
#include <robot_common/LogHelper.h>
#include <robot_driver/serial/HardwareSerial.h>

struct WheelState
{
	unsigned long timestamp; // 时间戳, 单位: ms
	int voltage;			 // 电压, V
	float current;			 // 电流, A
	float velocity;			 // 速度, m/s
	int position;			 // 位置, 编码值
	int feedback;			 // 位置反馈

	bool isOverflow;	 // 是否过流
	bool isOverload;	 // 是否过载
	bool isOvervoltage;  // 是否过压
	bool isUndervoltage; // 是否欠压
	bool isEncoderError; // 是否编码器故障

	bool isParsedSuccess; // 是否解析成功

	WheelState()
		: timestamp(0), voltage(0), current(0.0), velocity(0.0), position(0), feedback(0), isOverflow(false), isOverload(false), isOvervoltage(false), isUndervoltage(false), isEncoderError(false), isParsedSuccess(false)
	{
	}
};

using namespace std;

// 底层功能实现
class WheelDriver : private WheelProtocal
{
  private:
	float wheelPerimeter;   // 轮子直径, 单位:m
	HardwareSerial *serial; // 串口操作

	WheelState wheelState;

	mutex mutexFun; // 互斥体, 保证不存在多线程同时调用功能函数

  public:
	volatile bool isRunning; // 正在运行

  public:
	~WheelDriver();
	WheelDriver(float wheelPerimeter = 0.5338, const char *port = "", unsigned int baudRate = 57600, unsigned int nReadTimeout = 0, unsigned int nWriteTimeout = 0);

	bool StartMotor();						   // 开启电机
	bool CloseMotor();						   // 关闭电机
	bool SetMode(Mode mode);				   // 设置速度模式
	bool SetLimitVelocity(float velocity);	 // 位置模式下的速度限幅值(位置命令下达时的实际速度) 单位: m/s
	bool ClearFault();						   // 清除故障
	void ReadStatus(u8 *data, u8 *dataReaded); // 读取状态
	bool SetVelocity(float velocity);		   // 设置速度 单位: m/s
	bool SetPosition(float distance);		   // 前进 单位: m
	void ReopenPort();						   // 重启端口

	const WheelState &ParseStatus(u8 *data, int len);	// 解析状态
	void PrintStatus();									 // 打印状态信息
	bool CheckCmdState(CmdState state, const char *tip); // 检测命令执行成功与否, 并打印错误日志
	bool HasErrorOccurred();							 // 有错误发生, 当执行该函数时, 表明错误已解决, 并且另一个轮子的端口需要重新打开

  protected:
	void flushInput();											   // 清除收到但是尚未读取的数据
	int write(const u8 *data, const int len, const int msTimeout); // 串口写
	int read(u8 *nDat, int nLen, const int msTimeout);			   // 串口读

  private:
	int velocity2Rpm(float velocity); // m/s的速度转换为rmp单位的速度
	bool checkSum(u8 *cmd, int len);  // 判断数字校验和
};

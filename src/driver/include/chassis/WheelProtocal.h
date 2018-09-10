#pragma once

#include <string.h>

#define ARRAY_LEN(array) (sizeof(array) / sizeof(array[0]))

typedef char s8;
typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned long u32;

enum Mode
{
	Position,	// 位置模式
	Speed,		 // 速度模式
	Torque,		 // 力矩模式
	RelPosition, // 相对位置模式
	AbsPosition  // 绝对位置模式
};

enum CmdState
{
	Normal,		   // 正常
	DataSizeError, // 数据长度错误
	AckAddrError,  // 上位机接收到的确认地址错误
	CmdNotExist	// 命令不存在
};

/**
 * RS232通讯格式
 * 		数据地址  数据高八位 数据低八位   数据校验和
 * 		A1			A2		 A3		  A1+A2+A3
 */

/**
 * 不可变命令
 */
static const u8 CMD_ENABLE[] = {0x00, 0x00, 0x01, 0x01};			// 电机使能
static const u8 CMD_DISABLE[] = {0x00, 0x00, 0x00, 0x00};			// 电机失能
static const u8 CMD_SPEED_MODE[] = {0x02, 0x00, 0xc4, 0xc6};		// 速度模式-PC数字输入
static const u8 CMD_POSITION_MODE[] = {0x02, 0x00, 0xd0, 0xd2};		// 位置模式-PC数字输入
static const u8 CMD_ABS_POSITION_MODE[] = {0x51, 0x00, 0x00, 0x51}; // 绝对位置模式
static const u8 CMD_REL_POSITION_MODE[] = {0x51, 0x00, 0x01, 0x52}; // 相对位置模式
static const u8 CMD_MONITOR[] = {0x80, 0x00, 0x80};					// 读监控参数
static const u8 CMD_CLEAN_FAILURE[] = {0x4a, 0x00, 0x00, 0x4a};		// 清除障碍

/**
 * 可变命令
 */
static u8 CMD_VELOCITY[] = {0x06, 0x00, 0x00, 0x00};				// 设置速度模式下的速度
static u8 CMD_ACC_VELOCITY[] = {0x0a, 0x00, 0x00, 0x00};			// 设置速度模式下的加减速度, A2:加速度, A3:减速度
static u8 CMD_POSITION_HIGH[] = {0x50, 0x00, 0x00, 0x00};			// 设置位置模式下的高16位
static u8 CMD_POSITION_LOW[] = {0x05, 0x00, 0x00, 0x00};			// 设置位置模式下的低16位
static u8 CMD_POSITION_LIMIT_VELOCITY[] = {0x1d, 0x00, 0x00, 0x00}; // 位置模式下的速度限幅值

//  底层协议, 提供底层的方法, 供上层调用
class WheelProtocal
{
  public:
	WheelProtocal();
	CmdState StartMotor();						   // 开启电机
	CmdState CloseMotor();						   // 关闭电机
	CmdState SetMode(Mode mode);				   // 设置速度模式
	CmdState SetLimitVelocity(int rpm);			   // 位置模式下的速度限幅值(位置命令下达时的实际速度) 单位:m/s
	CmdState ClearFault();						   // 清除故障
	CmdState ReadStatus(u8 *data, u8 *dataReaded); // 读取状态
	CmdState SetVelocity(int rpm);				   // 设置速度 单位:rpm
	CmdState SetPosition(int position);			   // 设置相对位置 单位:编码值

	void Data2u16(u16 *DataL, u16 *DataH, int Data); // 1个32位数拆分为2个16位数
	void Data2u8(u8 *DataL, u8 *DataH, u16 Data);  // 1个16位数拆分为2个8位数
	u16 U8toData(u8 DataL, u8 DataH);				 // 2个8位数组合为1个16位数

  protected:
	/**
	 * 这三个方法由Driver层负责实现
	 */
	virtual int write(const u8 *data, const int len, const int msTimeout = 0) = 0;
	virtual int read(u8 *nDat, int nLen, const int msTimeout = 0) = 0;
	/** 清除收到但是尚未读取的数据 */
	virtual void flushInput() = 0;

  private:
	CmdState writeCmd(const u8 *cmd, int len, const int msTimeout = 0); // 写入命令
	CmdState ack(u8 addr);												// 验证应答
	void checkSum(u8 *cmd, int len);									// 数据校验和
};

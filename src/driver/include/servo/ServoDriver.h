#pragma once

#include <map>
#include <chrono>

#include <robot_driver/servo/SXSProtocol.h>
#include <robot_driver/serial/HardwareSerial.h>

using namespace std;

typedef unsigned int DeviceIdType;

struct ServoDevice
{
	DeviceIdType id;			   // 设备id
	bool isOnline;				   // 是否在线
	unsigned short initAzimuth;	// 初始方位
	unsigned short currentAzimuth; // 当前方位
	unsigned short minAzimuth;	 // 最小方位
	unsigned short maxAzimuth;	 // 最大方位
	int state;					   // 设备状态
	char multiplier;	  // 增量因子, 表示增加的方向, 为1表明:增加为正, -1表明:增加为负
	string description;			   // 设备描述

	ServoDevice() {}


	ServoDevice(uint8_t id, uint16_t initAzimuth, uint16_t minAzimuth, uint16_t maxAzimuth, string description, char multiplier = 1)
		: id(id), initAzimuth(initAzimuth), minAzimuth(minAzimuth), maxAzimuth(maxAzimuth), description(description), multiplier(multiplier), currentAzimuth(0), state(0), isOnline(false) {}
};

class ServoDriver : private SXSProtocol
{
	HardwareSerial *pSerial; //串口指针
  public:
	ServoDriver(const char *szPort, unsigned int nBaudRate, unsigned int nReadTimeout = 10, unsigned int nWriteTimeout = 10);
	~ServoDriver();

	/** 查看设备是否在线, 并获取其状态信息: 可是否过流、过压、过热等 */
	void Ping(ServoDevice &device);
	void Pings(map<DeviceIdType, ServoDevice *> &servoDevices);

	/** 设置设备角度位置 */
	void SetDeviceAzimuth(unsigned char nID, unsigned short nAzimuth, unsigned short nRunTime, unsigned short nRunSpeed);
	void GetDeviceAzimuth(ServoDevice& device);

	/** 同步操控多个舵机 */
	void SetMultDeviceAzimuth(std::vector<unsigned char> id, std::vector<unsigned short> nAzimuth, std::vector<unsigned short> nRunTime, std::vector<unsigned short> nRunSpeed);

	/** 解析设备状态 */
	void CheckStatus(ServoDevice &servoDevice);

  private:
	virtual int writeSXS(unsigned char *nDat, int nLen); //输出nLen字节
	virtual int readSXS(unsigned char *nDat, int nLen);  //输入nLen字节
	virtual int writeSXS(unsigned char bDat);			 //输出1字节
	virtual void flushSXS();							 //刷新接口缓冲区
};

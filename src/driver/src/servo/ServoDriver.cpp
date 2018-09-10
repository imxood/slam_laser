#include <robot_driver/servo/ServoDriver.h>

ServoDriver::ServoDriver(const char *szPort, unsigned int nBaudRate, unsigned int nReadTimeout, unsigned int nWriteTimeout)
{
	pSerial = new HardwareSerial(szPort, nBaudRate, nReadTimeout, nWriteTimeout);
}

ServoDriver::~ServoDriver()
{
	if (!pSerial)
	{
		pSerial->close();
	}
}

void ServoDriver::Ping(ServoDevice &device)
{
	int state = SXSProtocol::Ping(device.id);

	if (state != -1)
	{
		device.isOnline = true;
	}
	device.state = state;
}

/** 查看设备是否在线, 并获取其状态信息: 可是否过流、过压、过热等 */
void ServoDriver::Pings(map<DeviceIdType, ServoDevice*> &servoDevices)
{
	for(auto &d : servoDevices){
		Ping(*d.second);
	}
}

// 设置设备角度位置
void ServoDriver::SetDeviceAzimuth(unsigned char nID, unsigned short nAzimuth, unsigned short nRunTime, unsigned short nRunSpeed)
{
	WritePos(nID, nAzimuth, nRunTime, nRunSpeed);
}

void ServoDriver::GetDeviceAzimuth(ServoDevice& device)
{
	device.currentAzimuth = ReadPos(device.id);
}

// 同步操控多个舵机
void ServoDriver::SetMultDeviceAzimuth(std::vector<unsigned char> id, std::vector<unsigned short> nAzimuth, std::vector<unsigned short> nRunTime, std::vector<unsigned short> nRunSpeed)
{
}

//  向设备写入数据后, 返回值的错误诊断
void ServoDriver::CheckStatus(ServoDevice &servoDevice)
{
	unsigned char status = servoDevice.state;
	const char* tips = servoDevice.description.c_str();

	if (End == 0) // SMS
	{
		if (SMS_ERROR_JUSTFINE(status)) //正常
		{
		}
		else if (SMS_ERROR_OVERLOAD(status)) //过载
		{
			LOG(WARNING) << tips << "\t SXS Status : Overload.";
		}
		else if (SMS_ERROR_OVERFLOW(status)) //过流
		{
			LOG(WARNING) << tips << "\t SXS Status : Overflow.";
		}
		else if (SMS_ERROR_OVERHEAT(status)) //过热
		{
			LOG(WARNING) << tips << "\t SXS Status : Overheat.";
		}
		else if (SMS_ERROR_ERRANGLE(status)) //角度错误
		{
			LOG(WARNING) << tips << "\t SXS Status : AngleError.";
		}
		else if (SMS_ERROR_OVERVOLT(status)) //过压
		{
			LOG(WARNING) << tips << "\t SXS Status : OverVoltage.";
		}
		else if (SMS_ERROR_UNKNOWNS(status)) //未知
		{
			LOG(WARNING) << tips << "\t SXS Status : Unknow.";
		}
	}
	else
	{
		if (SCS_ERROR_JUSTFINE(status)) //正常
		{
		}
		else if (SCS_ERROR_OVERLOAD(status)) //过载
		{
			LOG(WARNING) << tips << "\t SXS Status : Overload.";
		}
		else if (SCS_ERROR_OVERHEAT(status)) //过热
		{
			LOG(WARNING) << tips << "\t SXS Status : Overheat.";
		}
		else if (SCS_ERROR_OVERVOLT(status)) //过压
		{
			LOG(WARNING) << tips << "\t SXS Status : OverVoltage.";
		}
		else if (SCS_ERROR_UNKNOWNS(status)) //未知
		{
			LOG(WARNING) << tips << "\t SXS Status : Unknow.";
		}
	}
}

int ServoDriver::readSXS(unsigned char *nData, int nLen)
{
	if (nData == NULL || nLen == 0)
	{
		return 0;
	}

	return pSerial->read(nData, nLen);
}

int ServoDriver::writeSXS(unsigned char *nDat, int nLen)
{
	if (nDat == NULL || nLen == 0)
	{
		return 0;
	}
	return pSerial->write(nDat, nLen);
}

int ServoDriver::writeSXS(unsigned char bDat)
{
	return pSerial->write(&bDat, 1);
}

void ServoDriver::flushSXS()
{
	while (pSerial->read() != -1)
		;
}

// int main(int argc, char const *argv[])
// {
// 	LogHelper log(argv[0]);

// 	std::map<unsigned char, unsigned char> mapOnlines;

// 	ServoDriver servo("/dev/ttyUSB0", 115200, 5, 5); // 读写5ms超时
// 	servo.StatusScan(mapOnlines);

// 	LOG(WARNING) << "mapOnlines.size: " << mapOnlines.size();

// 	servo.SetDeviceAzimuth(1, 1400, 0, 100);

// 	return 0;
// }

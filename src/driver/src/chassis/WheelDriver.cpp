
#include <robot_driver/chassis/WheelDriver.h>

WheelDriver::WheelDriver(float wheelPerimeter, const char *port, unsigned int baudRate, unsigned int msReadTimeout, unsigned int msWriteTimeout)
{
	this->wheelPerimeter = wheelPerimeter;
	serial = new HardwareSerial(port, baudRate, msReadTimeout, msWriteTimeout);
	isRunning = true;
}

WheelDriver::~WheelDriver()
{
	if (serial)
	{
		serial->close();
		delete serial;
		serial = NULL;
	}
}

/**
 * 开启电机
 */
bool WheelDriver::StartMotor()
{
	unique_lock<mutex> lock(mutexFun); // 上锁

	auto time_now = std::chrono::system_clock::now().time_since_epoch();
	long timestamp = chrono::duration_cast<chrono::milliseconds>(time_now).count();

	CmdState state = WheelProtocal::StartMotor();

	time_now = std::chrono::system_clock::now().time_since_epoch();
	LOG(INFO) << "StartMotor() 执行时间是: " << chrono::duration_cast<chrono::milliseconds>(time_now).count() - timestamp << "ms";

	return CheckCmdState(state, "StartMotor");
}

/**
 * 关闭电机
 */
bool WheelDriver::CloseMotor()
{
	unique_lock<mutex> lock(mutexFun); // 上锁
	CmdState state = WheelProtocal::CloseMotor();
	return CheckCmdState(state, "CloseMotor");
}

/**
 * 设置速度模式
 */
bool WheelDriver::SetMode(Mode mode)
{
	unique_lock<mutex> lock(mutexFun); // 上锁

	auto time_now = std::chrono::system_clock::now().time_since_epoch();
	long timestamp = chrono::duration_cast<chrono::milliseconds>(time_now).count();

	CmdState state = WheelProtocal::SetMode(mode);

	time_now = std::chrono::system_clock::now().time_since_epoch();
	LOG(INFO) << "SetMode() 执行时间是: " << chrono::duration_cast<chrono::milliseconds>(time_now).count() - timestamp << "ms";

	return CheckCmdState(state, "SetMode");
}

/**
 * 位置模式下的速度限幅值(位置命令下达时的实际速度) 单位:m/s
 */
bool WheelDriver::SetLimitVelocity(float velocity)
{
	unique_lock<mutex> lock(mutexFun); // 上锁

	auto time_now = std::chrono::system_clock::now().time_since_epoch();
	long timestamp = chrono::duration_cast<chrono::milliseconds>(time_now).count();

	int rpm = velocity2Rpm(velocity);
	CmdState state = WheelProtocal::SetLimitVelocity(rpm);

	time_now = std::chrono::system_clock::now().time_since_epoch();
	LOG(INFO) << "SetLimitVelocity() 执行时间是: " << chrono::duration_cast<chrono::milliseconds>(time_now).count() - timestamp << "ms";

	return CheckCmdState(state, "SetLimitVelocity");
}

/**
 * 清除故障
 */
bool WheelDriver::ClearFault()
{
	unique_lock<mutex> lock(mutexFun); // 上锁
	CmdState state = WheelProtocal::ClearFault();
	return CheckCmdState(state, "ClearFault");
}

/**
 * 读取状态
 */
void WheelDriver::ReadStatus(u8 *data, u8 *dataReaded)
{
	unique_lock<mutex> lock(mutexFun); // 上锁
	CmdState state = WheelProtocal::ReadStatus(data, dataReaded);
	if (CheckCmdState(state, "ReadStatus"))
		ParseStatus(data, *dataReaded);
}

/**
 * 设置速度 单位: m/s
 */
bool WheelDriver::SetVelocity(float velocity)
{
	unique_lock<mutex> lock(mutexFun); // 上锁

	int rpm = velocity2Rpm(velocity);
	CmdState state = WheelProtocal::SetVelocity(rpm);
	return CheckCmdState(state, "SetVelocity");
}

/**
 * 设置相对位置 单位: m
 */
bool WheelDriver::SetPosition(float distance)
{
	int encode = distance / wheelPerimeter * 4096 + 0.5; // 编码值
	unique_lock<mutex> lock(mutexFun);					 // 上锁
	CmdState state = WheelProtocal::SetPosition(encode);
	return CheckCmdState(state, "SetPosition");
}

/**
 * 重启端口
 */
void WheelDriver::ReopenPort()
{
	serial->reopen();
}

/**
 * 解析状态
 */
const WheelState &WheelDriver::ParseStatus(u8 *data, int len)
{
	auto time_now = std::chrono::system_clock::now().time_since_epoch();
	long timestamp = chrono::duration_cast<chrono::milliseconds>(time_now).count();

	wheelState.timestamp = timestamp;
	wheelState.isParsedSuccess = false; // 初始值: 未解析成功

	if (len != 32)
	{
		LOG(ERROR) << "解析状态: 数据长度错误, 期望值是32, 实际是" << len;
		return wheelState;
	}

	u8 *p = data;

	// 故障状态
	if (*p == 0x80 && *(p + 1) == 0x00 && checkSum(p, 4))
	{
		wheelState.isOverflow = *(p + 2) ^ 1;
		wheelState.isOvervoltage = *(p + 2) ^ 2;
		wheelState.isEncoderError = *(p + 2) ^ 3;
		wheelState.isUndervoltage = *(p + 2) ^ 5;
		wheelState.isOverload = *(p + 2) ^ 6;

		p += 4;
		// 母线电压
		if (*p != 0xe1 || !checkSum(p, 4))
		{
			LOG(ERROR) << "母线电压, 数据错误";
			return wheelState;
		}
		wheelState.voltage = U8toData(*(p + 2), *(p + 1));

		p += 4;
		// 输出电流
		if (*p != 0xe2 || !checkSum(p, 4))
		{
			LOG(ERROR) << "输出电流, 数据错误";
			return wheelState;
		}
		wheelState.current = U8toData(*(p + 2), *(p + 1)) / 100.0;

		p += 4;
		// 输出转速
		if (*p != 0xe4 || !checkSum(p, 4))
		{
			LOG(ERROR) << "输出转速, 数据错误";
			return wheelState;
		}
		wheelState.velocity = U8toData(*(p + 2), *(p + 1)) / 16384.0 * 6000 * wheelPerimeter / 60;

		p += 4;
		// 位置给定
		if (*p != 0xe6 || !checkSum(p, 4))
		{
			LOG(ERROR) << "位置给定高16位, 数据错误";
			return wheelState;
		}
		int positionHigh = U8toData(*(p + 2), *(p + 1));
		p += 4;
		if (*p != 0xe7 || !checkSum(p, 4))
		{
			LOG(ERROR) << "位置给定低16位, 数据错误";
			return wheelState;
		}
		int positionLow = U8toData(*(p + 2), *(p + 1));
		wheelState.position = positionHigh << 16 | positionLow;

		p += 4;
		// 位置反馈
		if (*p != 0xe8 || !checkSum(p, 4))
		{
			LOG(ERROR) << "位置反馈高16位, 数据错误";
			return wheelState;
		}
		u16 feedbackHigh = U8toData(*(p + 2), *(p + 1));
		p += 4;
		if (*p != 0xe9 || !checkSum(p, 4))
		{
			LOG(ERROR) << "位置反馈低16位, 数据错误";
			return wheelState;
		}
		u16 feedbackLow = U8toData(*(p + 2), *(p + 1));
		wheelState.feedback = feedbackHigh << 16 | feedbackLow;

		wheelState.isParsedSuccess = true; // 解析成功

		return wheelState;
	}
	LOG(ERROR) << "故障状态, 数据错误";
	return wheelState;
}

/**
 * 打印状态信息
 */
void WheelDriver::PrintStatus()
{
	LOG(INFO) << "速度: " << wheelState.velocity << "m/s, "
			  //   << "current: " << wheelState.current << ", "
			  //   << "voltage: " << wheelState.voltage << ", "
			  << "position: " << wheelState.position << ", "
			  << "feedback: " << wheelState.feedback << ", "
			  //   << "isOverflow: " << wheelState.isOverflow << ", "
			  //   << "isOverload: " << wheelState.isOverload << ", "
			  //   << "isOvervoltage: " << wheelState.isOvervoltage << ", "
			  //   << "isUndervoltage: " << wheelState.isUndervoltage << ", "
			  << "isEncoderError: " << (wheelState.isEncoderError ? "是" : "否");
}

/**
 * 检测命令执行成功与否, 并打印错误日志
 */
bool WheelDriver::CheckCmdState(CmdState state, const char *tip)
{
	if (state == Normal)
		return true;

	switch (state)
	{
	case Normal:
		return true;

	case DataSizeError:
		LOG(ERROR) << tip << ": 数据长度错误";
		return false;

	case AckAddrError:
		LOG(ERROR) << tip << ": 校验地址错误";
		return false;

	case CmdNotExist:
		LOG(ERROR) << tip << ": 执行命令未发现";
		return false;
	}

	LOG(ERROR) << tip << ": 操作异常, 不合理的执行";

	return false;
}

bool WheelDriver::HasErrorOccurred()
{
	return serial->HasErrorOccurred();
}

/**
 * 清除收到但是尚未读取的数据
 */
void WheelDriver::flushInput()
{
	serial->flushInput();
	// while (serial->read() != -1)
	// 	;
}

/**
 * 串口写
 */
int WheelDriver::write(const u8 *data, const int len, const int msTimeout)
{
	if (data == NULL || len == 0)
		return 0;
	return serial->write(data, len, msTimeout);
}

/**
 * 串口读
 */
int WheelDriver::read(u8 *data, int len, const int msTimeout)
{
	if (data == NULL || len == 0)
		return 0;
	return serial->read(data, len, msTimeout);
}

/**
 * m/s的速度转换为rmp单位的速度
 */
int WheelDriver::velocity2Rpm(float velocity)
{
	if (velocity > 0)
		return velocity * 60 / wheelPerimeter + 0.5;

	return velocity * 60 / wheelPerimeter - 0.5;
}

/**
 * 数据校验和
 */
bool WheelDriver::checkSum(u8 *cmd, int len)
{
	u8 check_sum = 0;

	for (int i = 0; i < len - 1; i++)
		check_sum += cmd[i];

	if (cmd[len - 1] == check_sum)
		return true;

	return false;
}

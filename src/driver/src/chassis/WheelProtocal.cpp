#include <robot_common/LogHelper.h>
#include <robot_driver/chassis/WheelProtocal.h>

// 底层协议, 提供底层的方法, 供上层调用
WheelProtocal::WheelProtocal() {}

/**
 * 开启电机
 */
CmdState WheelProtocal::StartMotor()
{
	flushInput();
	return writeCmd(CMD_ENABLE, ARRAY_LEN(CMD_ENABLE));
}

/**
 * 关闭电机
 */
CmdState WheelProtocal::CloseMotor()
{
	flushInput();
	return writeCmd(CMD_DISABLE, ARRAY_LEN(CMD_DISABLE));
}

/**
 * 设置速度模式
 */
CmdState WheelProtocal::SetMode(Mode mode)
{
	flushInput();
	switch (mode)
	{
	case Position:
		return writeCmd(CMD_POSITION_MODE, ARRAY_LEN(CMD_POSITION_MODE)); // 设置位置模式

	case Speed:
		return writeCmd(CMD_SPEED_MODE, ARRAY_LEN(CMD_SPEED_MODE)); // 设置速度模式

	case RelPosition:
		return writeCmd(CMD_REL_POSITION_MODE, ARRAY_LEN(CMD_REL_POSITION_MODE)); // 设置相对位置模式

	case AbsPosition:
		return writeCmd(CMD_ABS_POSITION_MODE, ARRAY_LEN(CMD_ABS_POSITION_MODE)); // 设置绝对位置模式
	}
	return CmdNotExist;
}

/**
 * 位置模式下的速度限幅值(位置命令下达时的实际速度) 单位:m/s
 */
CmdState WheelProtocal::SetLimitVelocity(int rpm)
{
	flushInput();

	int len = ARRAY_LEN(CMD_POSITION_LIMIT_VELOCITY);

	int velocity = rpm / 6000.0 * 16384 + 0.5; // 四舍五入
	Data2u8(CMD_POSITION_LIMIT_VELOCITY + 2, CMD_POSITION_LIMIT_VELOCITY + 1, velocity);
	checkSum(CMD_POSITION_LIMIT_VELOCITY, len);

	return writeCmd(CMD_POSITION_LIMIT_VELOCITY, len);
}

/**
 * 清除故障
 */
CmdState WheelProtocal::ClearFault()
{
	flushInput();
	return writeCmd(CMD_CLEAN_FAILURE, ARRAY_LEN(CMD_CLEAN_FAILURE));
}

/**
 * 监控状态
 */
CmdState WheelProtocal::ReadStatus(u8 *data, u8 *dataReaded)
{
	flushInput();

	CmdState state;
	state = writeCmd(CMD_MONITOR, ARRAY_LEN(CMD_MONITOR));

	if (state != Normal)
		return state;

	*dataReaded = read(data, 32, 55);

	if (*dataReaded != 32)
		return DataSizeError;

	return Normal;
}

/**
 * 设置速度, 单位:rpm
 */
CmdState WheelProtocal::SetVelocity(int rpm)
{
	flushInput();

	int len = ARRAY_LEN(CMD_VELOCITY);

	int velocity = rpm / 6000.0 * 16384 + 0.5;
	Data2u8(CMD_VELOCITY + 2, CMD_VELOCITY + 1, velocity);
	checkSum(CMD_VELOCITY, len);

	return writeCmd(CMD_VELOCITY, len);
}

/**
 * 设置位置 单位:编码值
 */
CmdState WheelProtocal::SetPosition(int position)
{
	flushInput();

	u16 positionH, positionL;
	Data2u16(&positionL, &positionH, position);

	Data2u8(CMD_POSITION_HIGH + 2, CMD_POSITION_HIGH + 1, positionH);
	checkSum(CMD_POSITION_HIGH, ARRAY_LEN(CMD_POSITION_HIGH));

	CmdState state = writeCmd(CMD_POSITION_HIGH, ARRAY_LEN(CMD_POSITION_HIGH)); // 写入高16位

	if (state != Normal)
		return state;

	Data2u8(CMD_POSITION_LOW + 2, CMD_POSITION_LOW + 1, positionL);
	checkSum(CMD_POSITION_LOW, ARRAY_LEN(CMD_POSITION_LOW));

	state = writeCmd(CMD_POSITION_LOW, ARRAY_LEN(CMD_POSITION_LOW)); // 写入低16位

	return state;
}

/**
 * 写入命令
 */
CmdState WheelProtocal::writeCmd(const u8 *cmd, int len, const int msTimeout)
{
	if (write(cmd, len, msTimeout) != len)
	{
		LOG(ERROR) << "写入失败!";
		return DataSizeError;
	}
	if (cmd[0] == 0x80)
	{
		return Normal;
	}
	return ack(cmd[0]);
}

/**
 * 1个32位数拆分为2个16位数
 */
void WheelProtocal::Data2u16(u16 *DataL, u16 *DataH, int Data)
{
	*DataH = Data >> 16;
	*DataL = Data;
}

/**
 * 1个16位数拆分为2个8位数
 */
void WheelProtocal::Data2u8(u8 *DataL, u8 *DataH, u16 Data)
{
	*DataH = Data >> 8;
	*DataL = Data;
}

/**
 * 2个8位数组合为1个16位数
 */
u16 WheelProtocal::U8toData(u8 DataL, u8 DataH)
{
	u16 Data;
	Data = DataH;
	Data <<= 8;
	Data |= DataL;
	return Data;
}

/**
 * 验证应答
 */
CmdState WheelProtocal::ack(u8 addr)
{
	u8 buf[2];
	u8 size = read(buf, 2);

	if (size != 2)
		return DataSizeError;

	if (buf[0] == addr && buf[1] == addr)
		return Normal;

	return AckAddrError;
}

/**
 * 数据校验和
 */
void WheelProtocal::checkSum(u8 *cmd, int len)
{
	u8 check_sum = 0;
	for (int i = 0; i < len - 1; i++)
	{
		check_sum += cmd[i];
	}
	cmd[len - 1] = check_sum;
}

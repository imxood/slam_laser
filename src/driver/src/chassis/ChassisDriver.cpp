#include <robot_driver/chassis/ChassisDriver.h>

namespace Motorchassis
{
ChassisDriver::ChassisDriver(const char *Port, UINT BaudRate, UINT Timeout)
{
	m_serial = NULL;
	m_servoport = Port;
	m_bardrate = BaudRate;
	m_timeout = Timeout;
	Serial_Open(Port, BaudRate, Timeout);
}
ChassisDriver::~ChassisDriver()
{
	Serial_Close();
}

void ChassisDriver::Serial_Open(const char *Port, unsigned int BaudRate, unsigned int Timeout)
{
	bool logflag = true;
	while (m_serial == NULL)
	{
		try
		{
			m_serial = new serial::Serial(Port, BaudRate, serial::Timeout::simpleTimeout(Timeout));
		}
		catch (const std::exception &e)
		{
			if (logflag)
			{
				logflag = false;
				LOG(ERROR) << "---------------------chassis serial not connected----------------------\n";
			}
			sleep(3);
		}
	}
	while (!m_serial->isOpen())
	{
		try
		{
			m_serial->open();
		}
		catch (const std::exception &e)
		{
			LOG(ERROR) << "---------------------chassis serial not open----------------------\n";
			sleep(3);
		}
	}
	if (m_serial->isOpen())
		LOG(INFO) << "motor serial is open...";
}

void ChassisDriver::Serial_Close()
{
	if (m_serial)
	{
		m_serial->close();
		delete m_serial;
		m_serial = NULL;
		LOG(INFO) << "motor serial is close...";
	}
}

bool ChassisDriver::Serial_Read(BYTE *buf, long len)
{
	try{
		return m_serial->read(buf, len);
	}catch(const std::exception &e){
		m_serial = NULL;		
		Serial_Open(m_servoport.c_str(), m_bardrate, m_timeout);
	}
}

unsigned short ChassisDriver::Serial_Write(const BYTE *buf, long len)
{
	try{
		return m_serial->write(buf, len);
	}catch(const std::exception &e){
		m_serial = NULL;
		Serial_Open(m_servoport.c_str(), m_bardrate, m_timeout);
	}
}

bool ChassisDriver::WriteCmd(const BYTE *cmd)
{
	BYTE buf[2] = {0x00};
	if (Serial_Write(cmd, 4) != 4)
	{
		LOG(ERROR) << "motor serial write error!";
		return false;
	}
	Serial_Read(buf, 2);
	return true;
}

bool ChassisDriver::Start_Motors()
{
	return WriteCmd(motor_run);
}

bool ChassisDriver::ShutDown_Motors()
{
	return WriteCmd(motor_stop);
}

bool ChassisDriver::Set_Velocity_Mode()
{
	return WriteCmd(speed_mode);
}

bool ChassisDriver::Set_Position_Mode()
{
	return WriteCmd(position_mode_pc);
}

bool ChassisDriver::Set_MaxH_Speed(uint16_t value)
{
	if (value < 0) //速度不能小于0
		return false;
	LOG(INFO) << "value: ===" << value;
	uint8_t buff[4];
	int16_t speedSetValue = GETMAXSETVALUE(ROTATION_SPEED(value));
	LOG(INFO) << "speedSetValue: ===" << speedSetValue;
	buff[0] = 0x1D;
	buff[1] = (speedSetValue >> 8 & 0xFF);
	buff[2] = (speedSetValue & 0xFF);
	buff[3] = buff[0] + buff[1] + buff[2];
	return WriteCmd(buff);
}

bool ChassisDriver::Set_Relative_Pos()
{
	return WriteCmd(relative_position);
}

bool ChassisDriver::Set_Absolute_Pos()
{
	return WriteCmd(absolute_position);
}

bool ChassisDriver::Clear_Fault()
{
	return WriteCmd(clear_fault);
}

void ChassisDriver::SendMonitorCommand()
{
	BYTE buf[2] = {0x00};
	Serial_Write(monitor_cmd, 3);
	//第一次写入时有额外的信息获取
}

void ChassisDriver::Read_Status(BYTE *data, UINT dataToRead, UINT &dataReaded)
{
	if (m_serial->waitReadable())
		dataReaded = Serial_Read(data, dataToRead);
}

std::map<unsigned char, std::vector<unsigned char>> ChassisDriver::Parse_Status()
{
	BYTE src[32] = {0x80, 0x00, 0x01, 0x81, 0xE1, 0x00, 0x1C, 0xFD, 0xE2, 0x00, 0x11, 0xF3, 0xE4, 0x01, 0x0E, 0xF3,
					0xE6, 0x00, 0x00, 0xE6, 0xE7, 0x27, 0x10, 0x1E, 0xE8, 0x00, 0x00, 0xE8, 0xE9, 0x27, 0x10, 0x20};
	std::map<unsigned char, std::vector<unsigned char>> all_status;
	std::vector<unsigned char> status;
	std::vector<unsigned char> Busbar_voltage;
	std::vector<unsigned char> Output_current;
	std::vector<unsigned char> Rotation_rate;
	std::vector<unsigned char> Position_given_High16;
	std::vector<unsigned char> Position_given_Low16;
	std::vector<unsigned char> Position_feedback_High16;
	std::vector<unsigned char> Position_feedback_Low16;

	std::vector<unsigned char> error(1, 0x00);

	BYTE *data = (BYTE *)malloc(sizeof(BYTE *) * 40);
	memset(data, 0x00, 40);

	UINT dataToRead = 40;
	UINT &dataReaded = dataToRead;
	UINT len = 0;
	//发送监控指令
	SendMonitorCommand();
	// usleep(35000);
	//获取状态信息,
	Read_Status(data, dataToRead, len);

	// memcpy(data, src, 32);	//仅供测试

	if (data[0] != 0x80)
	{
		return all_status;
	}

	for (int i = 1; i < 4; i++)
	{
		status.push_back(data[i]);
		Busbar_voltage.push_back(data[i + BUSBAR_VOLTAGE(0)]);
		Output_current.push_back(data[i + OUTPUT_CURRENT(0)]);
		Rotation_rate.push_back(data[i + ROTATION_RATE(0)]);
		Position_given_High16.push_back(data[i + POSITION_GIVEN_HIGH16(0)]);
		Position_given_Low16.push_back(data[i + POSITION_GIVEN_LOW16(0)]);
		Position_feedback_High16.push_back(data[i + POSITION_FEEDBACK_HIGH16(0)]);
		Position_feedback_Low16.push_back(data[i + POSITION_FEEDBACK_LOW16(0)]);
	}

	all_status[data[STATUS(0)]] = status;
	all_status[data[BUSBAR_VOLTAGE(0)]] = Busbar_voltage;
	all_status[data[OUTPUT_CURRENT(0)]] = Output_current;
	all_status[data[ROTATION_RATE(0)]] = Rotation_rate;
	all_status[data[POSITION_GIVEN_HIGH16(0)]] = Position_given_High16;
	all_status[data[POSITION_GIVEN_LOW16(0)]] = Position_given_Low16;
	all_status[data[POSITION_FEEDBACK_HIGH16(0)]] = Position_feedback_High16;
	all_status[data[POSITION_FEEDBACK_LOW16(0)]] = Position_feedback_Low16;

	free(data);
	return all_status;
}

bool ChassisDriver::MotorStatus_Judge(BYTE status)
{
	if (STATUS_OV_I(status))
	{
		// std::cout << "过流" << std::endl;
		LOG(ERROR) << "motor 过流";
	}
	else if (STATUS_OV_U(status))
	{
		// std::cout << "过压" << std::endl;
		LOG(ERROR) << "motor 过压";
	}
	else if (STATUS_ERR_ENC(status))
	{
		// std::cout << "编码器故障" << std::endl;
		LOG(ERROR) << "motor 编码器故障";
	}
	else if (STATUS_OV_Q(status))
	{
		// std::cout << "欠压" << std::endl;
		LOG(ERROR) << "motor 欠压";
	}
	else if (STATUS_OV_LOAD(status))
	{
		// std::cout << "过载" << std::endl;
		LOG(ERROR) << "motor 过载";
	}
	else
		return true;
	return false;
}

void ChassisDriver::Set_Velocity(double velocity)
{
	BYTE buf[8] = {0x06};
	if (velocity > 1.000001) //确保速度不超过1m/s
		return;
	//把速度m/s转换成每分钟多少转，并转换成要写入的数据
	int value = SPEED_PERMINITES(velocity);

	//设置速度指令
	buf[1] = (BYTE)(value >> 8 & 0xff);
	buf[2] = (BYTE)(value & 0xff);
	buf[3] = buf[0] + buf[1] + buf[2];

	if (Serial_Write(buf, 4) != 4)
	{
		// std::cout << "serial write error!" << __FILE__ << __LINE__ << std::endl;
		LOG(ERROR) << "motor serial write failed!";
		return;
	}
}
void ChassisDriver::Set_Distance(int32_t distance)
{
	BYTE position[4] = {0x00};
	//得到需要设定的值
	long setValue = GETCODEDVALUEBY(distance);
	// printf("setValue: %d\n", setValue);
	position[0] = 0x50;
	//高16位
	position[1] = setValue >> 24 & 0xFF;
	position[2] = setValue >> 16 & 0xFF;
	position[3] = position[0] + position[1] + position[2];
	for (auto item : position)
		printf("高十六位: 0x%X\n", item);
	WriteCmd(position);

	position[0] = 0x05;
	//低16位
	position[1] = setValue >> 8 & 0xFF;
	position[2] = setValue & 0xFF;
	position[3] = position[0] + position[1] + position[2];
	for (auto item : position)
		printf("低十六位: 0x%X\n", item);

	WriteCmd(position);
}

void ChassisDriver::Set_CodedValue(int32_t codedValue)
{
	BYTE position[4] = {0x00};

	position[0] = 0x50;
	//高16位
	position[1] = codedValue >> 24 & 0xFF;
	position[2] = codedValue >> 16 & 0xFF;
	position[3] = position[0] + position[1] + position[2];
	for (auto item : position)
		printf("高十六位: 0x%X\n", item);
	WriteCmd(position);

	position[0] = 0x05;
	//低16位
	position[1] = codedValue >> 8 & 0xFF;
	position[2] = codedValue & 0xFF;
	position[3] = position[0] + position[1] + position[2];
	for (auto item : position)
		printf("低十六位: 0x%X\n", item);

	WriteCmd(position);
}

} // namespace Motorchassis

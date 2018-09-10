#include <robot_driver/serial/HardwareSerial.h>

atomic<bool> HardwareSerial::isRunning(false);

// 初始化串口
HardwareSerial::HardwareSerial(const char *port, unsigned int baudRate, unsigned int msReadTimeout, unsigned int msWriteTimeout)
	: pSerial(NULL), msReadTimeout(msReadTimeout), msWriteTimeout(msWriteTimeout), errorOccurred(false)
{
	isRunning = true;

	timeout = new serial::Timeout(0, 0, 0, 0, 0); // 读写超时, 运行期可修改

	pSerial = new serial::Serial();

	pSerial->setPort(port);
	pSerial->setBaudrate(baudRate);
}

// 关闭串口
HardwareSerial::~HardwareSerial()
{
	close();
}

// 关闭串口
void HardwareSerial::close()
{
	try
	{
		pSerial->close();
	}
	catch (const std::exception &e)
	{
		LOG(WARNING) << e.what();
		errorOccurred = true;
	}
}

// 重新打开
void HardwareSerial::reopen()
{
	close();
	isOpen();
}

// 端盘串口是否被打开, 若未打开则等待3s,重新打开
bool HardwareSerial::isOpen()
{
	if (pSerial->isOpen())
		return true;

	while (isRunning)
	{
		try
		{
			pSerial->open();
		}
		catch (const serial::IOException &e)
		{
			errorOccurred = true;
			close();
		}

		if (pSerial->isOpen())
			break;
		LOG(WARNING) << "串口打开失败, 3s后重试.";
		this_thread::sleep_for(chrono::seconds(3));
	}
	return true;
}

// 向串口写数据
int HardwareSerial::write(const unsigned char *nData, int nLen, int msWriteTimeout)
{
	try
	{
		if (isOpen())
		{
			setWriteTimeout(msWriteTimeout);
			return pSerial->write(nData, nLen);
		}
	}
	catch (const std::exception &e)
	{
		LOG(WARNING) << e.what();
		errorOccurred = true;
		close();
	}

	return 0;
}

/**
 * 从串口读一个字节的数据
 * 返回值为读到的数据, 若为-1,则读取失败, 否则成功.
 */
int HardwareSerial::read(int msReadTimeout)
{
	try
	{
		if (isOpen())
		{
			unsigned char ch;
			setReadTimeout(msReadTimeout);
			if (pSerial->read(&ch, 1) == 0)
				return -1;
			return ch;
		}
	}
	catch (const std::exception &e)
	{
		LOG(WARNING) << e.what();
		errorOccurred = true;
		close();
	}

	return -1;
}

/** 从串口读多个字节的数据 */
int HardwareSerial::read(unsigned char *nBuffer, int nLen, int msReadTimeout)
{
	try
	{
		if (isOpen())
		{
			setReadTimeout(msReadTimeout);
			return pSerial->read(nBuffer, nLen);
		}
	}
	catch (const std::exception &e)
	{
		LOG(WARNING) << e.what();
		errorOccurred = true;
		close();
	}

	return -1;
}

/** 丢弃收到但是尚未读取的数据 */
void HardwareSerial::flushInput()
{
	try
	{
		if (isOpen())
		{
			return pSerial->flushInput();
		}
	}
	catch (const std::exception &e)
	{
		LOG(WARNING) << e.what();
		errorOccurred = true;
		close();
	}
}

/** 丢弃要写入fd但尚未传输的数据 */
void HardwareSerial::flushOutput()
{
	try
	{
		if (isOpen())
		{
			return pSerial->flushOutput();
		}
	}
	catch (const std::exception &e)
	{
		LOG(WARNING) << e.what();
		errorOccurred = true;
		close();
	}
}

/** 有错误发生 */
bool HardwareSerial::HasErrorOccurred()
{
	if(errorOccurred){
		errorOccurred = false;
		return true;
	}
	return false;
}

/**
 * 运行期改变读超时参数
 */
void HardwareSerial::setReadTimeout(int msReadTimeout)
{
	if (msReadTimeout)
		timeout->read_timeout_constant = msReadTimeout;
	else
		timeout->read_timeout_constant = this->msReadTimeout;
	pSerial->setTimeout(*timeout);
}

/**
 * 运行期改变写超时参数
 */
void HardwareSerial::setWriteTimeout(int msWriteTimeout)
{
	if (msWriteTimeout)
		timeout->write_timeout_constant = msWriteTimeout;
	else
		timeout->write_timeout_constant = this->msWriteTimeout;
	pSerial->setTimeout(*timeout);
}

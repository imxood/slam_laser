#include <thread>
#include <atomic>
#include <exception>

#include <serial/serial.h>
#include <robot_common/LogHelper.h>

using namespace std;

class HardwareSerial
{
  private:
	int msReadTimeout;  // 若未指定超时, 则为默认的读超时
	int msWriteTimeout; // 若未指定超时, 则为默认的写超时

	serial::Timeout *timeout;   // 运行中的读写超时
	atomic<bool> errorOccurred; // 有错误发生, 异常捕获中改变该值

	serial::Serial *pSerial;

  public:
	static atomic<bool> isRunning; // 程序是否在运行

	HardwareSerial(const char *port, unsigned int baudRate, unsigned int msReadTimeout = 0, unsigned int msWriteTimeout = 0);
	~HardwareSerial();

	// 端盘串口是否被打开, 若未打开则等待3s,重新打开
	bool isOpen();

	// 关闭串口
	void close();

	// 重新打开
	void reopen();

	// 向串口写数据, 返回写入到的长度
	int write(const unsigned char *nData, int nLen, int msWriteTimeout = 0);

	/**
	 * 从串口读一个字节的数据
	 * 返回值为读到的数据, 若为-1,则读取失败, 否则成功.
	 */
	int read(int msReadTimeout = 0);

	/** 从串口读多个字节的数据, 返回读取到的长度 */
	int read(unsigned char *nBuffer, int nLen, int msReadTimeout = 0);

	/** 丢弃收到但是尚未读取的数据 */
	void flushInput();

	/** 丢弃要写入fd但尚未传输的数据 */
	void flushOutput();

	/** 有错误发生 */
	bool HasErrorOccurred();

  private:
	void setReadTimeout(int msReadTimeout);   // 运行期改变读超时参数
	void setWriteTimeout(int msWriteTimeout); // 运行期改变写超时参数
};

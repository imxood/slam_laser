#include <iostream>
#include <thread>
#include <chrono>

#include <robot_common/CommonType.h>
#include <robot_driver/chassis/WheelDriver.h>

using namespace std;

int main(int argc, char const *argv[])
{
	LogHelper log(argv[0]);

	WheelDriver wheel(0.5338, "/dev/port_rightwheel", 57600, 50, 10);

	thread t([&wheel]() {
		while (wheel.isRunning)
		{
			u8 buf[32];
			u8 dataReaded = 0;
			wheel.ReadStatus(buf, &dataReaded);
			wheel.PrintStatus();
			this_thread::sleep_for(chrono::milliseconds(200));
		}
		LOG(INFO) << "线程退出";
	});

	// 清除故障
	if (!wheel.ClearFault())
	{
		cout << "清除故障失败" << endl;
		return 1;
	}

	cout << "清除故障成功" << endl;

	// 启动电机
	if (!wheel.StartMotor())
	{
		cout << "电机启动失败" << endl;
		return 1;
	}

	cout << "电机启动成功" << endl;

	this_thread::sleep_for(chrono::milliseconds(1000));

	// 设置速度模式
	if (!wheel.SetMode(Speed))
	{
		cout << "设置速度模式失败" << endl;
		return 1;
	}
	cout << "设置速度模式成功" << endl;
	this_thread::sleep_for(chrono::milliseconds(1000));

	// 设置速度
	if (!wheel.SetVelocity(1))
	{
		cout << "设置速度失败" << endl;
		return 1;
	}
	cout << "设置速度为1" << endl;
	this_thread::sleep_for(chrono::seconds(5));

	if (!wheel.SetVelocity(-1))
	{
		cout << "设置速度失败" << endl;
		return 1;
	}
	cout << "设置速度为-1m/s" << endl;
	this_thread::sleep_for(chrono::seconds(5));

	// 设置位置模式
	if (!wheel.SetMode(Position))
	{
		cout << "设置位置模式失败" << endl;
		return 1;
	}
	cout << "设置位置模式成功" << endl;
	this_thread::sleep_for(chrono::milliseconds(500));

	// 设置相对位置模式
	if (!wheel.SetMode(RelPosition))
	{
		cout << "设置相对位置模式失败" << endl;
		return 1;
	}
	cout << "设置相对位置模式成功" << endl;
	this_thread::sleep_for(chrono::milliseconds(500));

	// 设置限速
	if (!wheel.SetLimitVelocity(LimitedVelocity))
	{
		cout << "设置限速失败" << endl;
		return 1;
	}
	cout << "设置限速 2m/s" << endl;
	this_thread::sleep_for(chrono::milliseconds(500));

	// 设置位置
	if (!wheel.SetPosition(-1)) // 前进1m
	{
		cout << "设置位置失败" << endl;
		return 1;
	}
	cout << "前进-1m" << endl;
	this_thread::sleep_for(chrono::seconds(10));

	// 停止电机
	if (!wheel.CloseMotor())
	{
		cout << "停止电机失败" << endl;
		return 1;
	}
	wheel.isRunning = false;

	cout << "停止电机成功" << endl;

	t.join();

	return 0;
}

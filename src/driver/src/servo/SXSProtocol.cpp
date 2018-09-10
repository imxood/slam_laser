/*
 * SXSProtocol.cpp
 * SXS串行舵机协议程序
 * 日期: 2016.8.25
 * 作者: 谭雄乐
 */

#include <stddef.h>
#include <robot_driver/servo/SXSProtocol.h>

SXSProtocol::SXSProtocol()
{
	Level = 1; //除广播指令所有指令返回应答
	End = 0;   //舵机处理器与控制板处理器端结构不一致
}

//1个16位数拆分为2个8位数
//DataL为低位，DataH为高位
void SXSProtocol::Host2SXS(u8 *DataL, u8 *DataH, int Data)
{
	if (End)
	{
		*DataL = (Data >> 8);
		*DataH = (Data & 0xff);
	}
	else
	{
		*DataH = (Data >> 8);
		*DataL = (Data & 0xff);
	}
}

//2个8位数组合为1个16位数
//DataL为低位，DataH为高位
int SXSProtocol::SXS2Host(u8 DataL, u8 DataH)
{
	int Data;
	if (End)
	{
		Data = DataL;
		Data <<= 8;
		Data |= DataH;
	}
	else
	{
		Data = DataH;
		Data <<= 8;
		Data |= DataL;
	}
	return Data;
}

void SXSProtocol::writeBuf(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen, u8 Fun)
{
	u8 msgLen = 2;
	u8 bBuf[6];
	u8 CheckSum = 0;
	bBuf[0] = 0xff;
	bBuf[1] = 0xff;
	bBuf[2] = ID;
	bBuf[4] = Fun;
	if (nDat)
	{
		msgLen += nLen + 1;
		bBuf[3] = msgLen;
		bBuf[5] = MemAddr;
		writeSXS(bBuf, 6);
	}
	else
	{
		bBuf[3] = msgLen;
		writeSXS(bBuf, 5);
	}
	CheckSum = ID + msgLen + Fun + MemAddr;
	u8 i = 0;
	if (nDat)
	{
		for (i = 0; i < nLen; i++)
		{
			CheckSum += nDat[i];
		}
	}
	writeSXS(nDat, nLen);
	writeSXS(~CheckSum);
}

//普通写指令
//舵机ID，MemAddr内存表地址，写入数据，写入长度
int SXSProtocol::genWrite(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen)
{
	flushSXS();
	writeBuf(ID, MemAddr, nDat, nLen, INST_WRITE);
	return Ack(ID);
}

//异步写指令
//舵机ID，MemAddr内存表地址，写入数据，写入长度
int SXSProtocol::regWrite(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen)
{
	flushSXS();
	writeBuf(ID, MemAddr, nDat, nLen, INST_REG_WRITE);
	return Ack(ID);
}

//同步写指令
//舵机ID[]数组，IDN数组长度，MemAddr内存表地址，写入数据，写入长度
void SXSProtocol::snycWrite(u8 ID[], u8 IDN, u8 MemAddr, u8 *nDat, u8 nLen)
{
	u8 mesLen = ((nLen + 1) * IDN + 4);
	u8 Sum = 0;
	u8 bBuf[7];
	bBuf[0] = 0xff;
	bBuf[1] = 0xff;
	bBuf[2] = 0xfe;
	bBuf[3] = mesLen;
	bBuf[4] = INST_SYNC_WRITE;
	bBuf[5] = MemAddr;
	bBuf[6] = nLen;
	writeSXS(bBuf, 7);

	Sum = 0xfe + mesLen + INST_SYNC_WRITE + MemAddr + nLen;
	u8 i, j;
	for (i = 0; i < IDN; i++)
	{
		writeSXS(ID[i]);
		writeSXS(nDat, nLen);
		Sum += ID[i];
		for (j = 0; j < nLen; j++)
		{
			Sum += nDat[j];
		}
	}
	writeSXS(~Sum);
}

int SXSProtocol::writeByte(u8 ID, u8 MemAddr, u8 bDat)
{
	flushSXS();
	writeBuf(ID, MemAddr, &bDat, 1, INST_WRITE);
	return Ack(ID);
}

int SXSProtocol::writeWord(u8 ID, u8 MemAddr, u16 wDat)
{
	flushSXS();
	u8 buf[2];
	Host2SXS(buf + 0, buf + 1, wDat);
	writeBuf(ID, MemAddr, buf, 2, INST_WRITE);
	return Ack(ID);
}

int SXSProtocol::EnableTorque(u8 ID, u8 Enable)
{
	return writeByte(ID, P_TORQUE_ENABLE, Enable);
}

int SXSProtocol::writePos(u8 ID, u16 Position, u16 Time, u16 Speed, u8 Fun)
{
	flushSXS();
	u8 buf[6];
	Host2SXS(buf + 0, buf + 1, Position);
	Host2SXS(buf + 2, buf + 3, Time);
	Host2SXS(buf + 4, buf + 5, Speed);
	writeBuf(ID, P_GOAL_POSITION_L, buf, 6, Fun);
	return Ack(ID);
}

//写位置指令
//舵机ID，Position位置，执行时间Time，执行速度Speed
int SXSProtocol::WritePos(u8 ID, u16 Position, u16 Time, u16 Speed)
{
	return writePos(ID, Position, Time, Speed, INST_WRITE);
}

//异步写位置指令
//舵机ID，Position位置，执行时间Time，执行速度Speed
int SXSProtocol::RegWritePos(u8 ID, u16 Position, u16 Time, u16 Speed)
{
	return writePos(ID, Position, Time, Speed, INST_REG_WRITE);
}

void SXSProtocol::RegWriteAction()
{
	writeBuf(0xfe, 0, NULL, 0, INST_ACTION);
}

//写位置指令
//舵机ID[]数组，IDN数组长度，Position位置，执行时间Time，执行速度Speed
void SXSProtocol::SyncWritePos(u8 ID[], u8 IDN, u16 Position, u16 Time, u16 Speed)
{
	u8 buf[6];
	Host2SXS(buf + 0, buf + 1, Position);
	Host2SXS(buf + 2, buf + 3, Time);
	Host2SXS(buf + 4, buf + 5, Speed);
	snycWrite(ID, IDN, P_GOAL_POSITION_L, buf, 6);
}

//读指令
//舵机ID，MemAddr内存表地址，返回数据nData，数据长度nLen
int SXSProtocol::Read(u8 ID, u8 MemAddr, u8 *nData, u8 nLen)
{
	flushSXS();
	writeBuf(ID, MemAddr, &nLen, 1, INST_READ);
	u8 bBuf[5];
	if (readSXS(bBuf, 5) != 5)
	{
		return 0;
	}
	int Size = readSXS(nData, nLen);
	if (readSXS(bBuf, 1))
	{
		return Size;
	}
	return 0;
}

//读1字节，超时返回-1
int SXSProtocol::readByte(u8 ID, u8 MemAddr)
{
	u8 bDat;
	int Size = Read(ID, MemAddr, &bDat, 1);
	if (Size != 1)
	{
		return -1;
	}
	else
	{
		return bDat;
	}
}

//读2字节，超时返回-1
int SXSProtocol::readWord(u8 ID, u8 MemAddr)
{
	u8 nDat[2];
	int Size;
	u16 wDat;
	Size = Read(ID, MemAddr, nDat, 2);
	if (Size != 2)
		return -1;
	wDat = SXS2Host(nDat[0], nDat[1]);
	return wDat;
}

//读位置，超时返回-1
int SXSProtocol::ReadPos(u8 ID)
{
	return readWord(ID, P_PRESENT_POSITION_L);
}

//多圈控制指令
int SXSProtocol::WriteSpe(u8 ID, s16 Speed)
{
	if (Speed < 0)
	{
		Speed = -Speed;
		Speed |= (1 << 10);
	}
	return writeWord(ID, P_GOAL_TIME_L, Speed);
}

//读电压，超时返回-1
int SXSProtocol::ReadVoltage(u8 ID)
{
	return readByte(ID, P_PRESENT_VOLTAGE);
}

//读温度，超时返回-1
int SXSProtocol::ReadTemper(u8 ID)
{
	return readByte(ID, P_PRESENT_TEMPERATURE);
}

//Ping指令，返回舵机ID，超时返回-1
int SXSProtocol::Ping(u8 ID)
{
	flushSXS();
	u8 bBuf[6];
	writeBuf(ID, 0, NULL, 0, INST_PING);
	int Size = readSXS(bBuf, 6);
	if (Size == 6)
	{
		return bBuf[4];
	}
	else
	{
		return -1;
	}
}

int SXSProtocol::wheelMode(u8 ID)
{
	u8 bBuf[4];
	bBuf[0] = 0;
	bBuf[1] = 0;
	bBuf[2] = 0;
	bBuf[3] = 0;
	return genWrite(ID, P_MIN_ANGLE_LIMIT_L, bBuf, 4);
}

int SXSProtocol::joinMode(u8 ID, u16 minAngle, u16 maxAngle)
{
	u8 bBuf[4];
	Host2SXS(bBuf, bBuf + 1, minAngle);
	Host2SXS(bBuf + 2, bBuf + 3, maxAngle);
	return genWrite(ID, P_MIN_ANGLE_LIMIT_L, bBuf, 4);
}

//复位舵机参数为默认值
int SXSProtocol::Reset(u8 ID)
{
	flushSXS();
	writeBuf(ID, 0, NULL, 0, INST_RESET);
	return Ack(ID);
}

int SXSProtocol::Ack(u8 ID)
{
	if (ID != 0xfe && Level)
	{
		u8 buf[6];
		u8 Size = readSXS(buf, 6);
		if (Size != 6)
		{
			return 0;
		}
	}
	return 1;
}

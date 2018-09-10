#ifndef __H_SXS_MEMORY_TABLE_PROTOCOL_H__
#define __H_SXS_MEMORY_TABLE_PROTOCOL_H__

#include <cstring>

namespace NS_Servo
{

#define SCS_ERROR_UNKNOWNS(UC_STATUS)    ((UC_STATUS | 0x00) != 0x00)  //未知
#define SCS_ERROR_JUSTFINE(UC_STATUS)    ((UC_STATUS | 0x00) == 0x00)  //正常
#define SCS_ERROR_OVERLOAD(UC_STATUS)    ((UC_STATUS & 0x20) == 0x20)  //过载
#define SCS_ERROR_OVERHEAT(UC_STATUS)    ((UC_STATUS & 0x04) == 0x04)  //过热
#define SCS_ERROR_OVERVOLT(UC_STATUS)    ((UC_STATUS & 0x01) == 0x01)  //过压

#define SMS_ERROR_UNKNOWNS(UC_STATUS)    ((UC_STATUS | 0x00) != 0x00)  //未知
#define SMS_ERROR_JUSTFINE(UC_STATUS)    ((UC_STATUS | 0x00) == 0x00)  //正常
#define SMS_ERROR_OVERLOAD(UC_STATUS)    ((UC_STATUS & 0x20) == 0x20)  //过载
#define SMS_ERROR_OVERFLOW(UC_STATUS)    ((UC_STATUS & 0x08) == 0x08)  //过流
#define SMS_ERROR_OVERHEAT(UC_STATUS)    ((UC_STATUS & 0x04) == 0x04)  //过热
#define SMS_ERROR_ERRANGLE(UC_STATUS)    ((UC_STATUS & 0x02) == 0x02)  //角度错误
#define SMS_ERROR_OVERVOLT(UC_STATUS)    ((UC_STATUS & 0x01) == 0x01)  //过压


#define SCS_ADDRESS_VersionH             0x03    //软件版本（H）      读            EEPROM
#define SCS_ADDRESS_VersionL             0x04    //软件版本（L）      读
#define SCS_ADDRESS_ID                   0x05    //ID                 读/写 （0x00）
#define SCS_ADDRESS_BaudRate             0x06    //波特率             读/写 （0x00）
#define SCS_ADDRESS_DelayTime            0x07    //返回延迟时间       读/写 （0x00）
#define SCS_ADDRESS_ReplyLevel           0x08    //应答状态级别       读/写 （0x01）
#define SCS_ADDRESS_MinAngleH            0x09    //最小角度限制（H）  读/写 （0x00）
#define SCS_ADDRESS_MinAngleL            0x0A    //最小角度限制（L）  读/写 （0x00）
#define SCS_ADDRESS_MaxAngleH            0x0B    //最大角度限制（H）  读/写 （0x03）
#define SCS_ADDRESS_MaxAngleL            0x0C    //最大角度限制（L）  读/写 （0xFF）
#define SCS_ADDRESS_MaxTemp              0x0D    //最高温度上限       读/写 （0x50）
#define SCS_ADDRESS_MaxVoltage           0x0E    //最高输入电压       读/写 （0xFA）
#define SCS_ADDRESS_MinVoltage           0x0F    //最低输入电压       读/写 （0x32）
#define SCS_ADDRESS_MaxTorqueH           0x10    //最大扭矩（H）      读/写 （0x03）
#define SCS_ADDRESS_MaxTorqueL           0x11    //最大扭矩（L）      读/写 （0xFF） 
#define SCS_ADDRESS_Unload               0x13    //卸载条件           读/写 （0x25）
#define SCS_ADDRESS_LedAlarm             0x14    //LED报警条件        读/写 （0x25）
#define SCS_ADDRESS_P                    0x15    //P                  读/写 （0x0F） 
#define SCS_ADDRESS_MinPwmH              0x18    //最小PWM（H）       读/写 （0x00）
#define SCS_ADDRESS_MinPwmL              0x19    //最小PWM（L）       读/写 （0x00）
#define SCS_ADDRESS_ClockwiseNSA         0x1A    //顺时针不灵敏区     读/写 （0x02）
#define SCS_ADDRESS_CClockwiseNSA        0x1B    //逆时针不灵敏区     读/写 （0x02） 

#define SCS_ADDRESS_TorqueSwitch         0x28    //扭矩开关           读/写 （0x00） RAM 
#define SCS_ADDRESS_TargetAzimuthH       0x2A    //目标位置（H）      读/写
#define SCS_ADDRESS_TargetAzimuthL       0x2B    //目标位置（L）      读/写
#define SCS_ADDRESS_RunTimeH             0x2C    //运行时间（H）      读/写 （0x00）
#define SCS_ADDRESS_RunTimeL             0x2D    //运行时间（L）      读/写 （0x00）
#define SCS_ADDRESS_RunSpeedH            0x2E    //运行速度（H）      读/写 （0x00）
#define SCS_ADDRESS_RunSpeedL            0x2F    //运行速度（L）      读/写 （0x00）
#define SCS_ADDRESS_Locked               0x30    //锁标志             读/写 （0x00） 
#define SCS_ADDRESS_CurrentAzimuthH      0x38    //当前位置（H）      读
#define SCS_ADDRESS_CurrentAzimuthL      0x39    //当前位置（L）      读
#define SCS_ADDRESS_CurrentSpeedH        0x3A    //当前速度（H）      读
#define SCS_ADDRESS_CurrentSpeedL        0x3B    //当前速度（L）      读
#define SCS_ADDRESS_CurrentOverloadH     0x3C    //当前负载（H）      读
#define SCS_ADDRESS_CurrentOverloadL     0x3D    //当前负载（L）      读
#define SCS_ADDRESS_CurrentVoltage       0x3E    //当前电压           读
#define SCS_ADDRESS_CurrentTemp          0x3F    //当前温度           读
#define SCS_ADDRESS_RegWrite             0x40    //REG WRITE标志      读（0x00）


#define SMS_ADDRESS_VersionL             0x03    //软件版本（L）      读            EEPROM
#define SMS_ADDRESS_VersionH             0x02    //软件版本（H）      读
#define SMS_ADDRESS_ID                   0x05    //ID                 读/写 （0x00）
#define SMS_ADDRESS_BaudRate             0x06    //波特率             读/写 （0x00）
#define SMS_ADDRESS_DelayTime            0x07    //返回延迟时间       读/写 （0x00）
#define SMS_ADDRESS_ReplyLevel           0x08    //应答状态级别       读/写 （0x01）
#define SMS_ADDRESS_MinAngleL            0x09    //最小角度限制（L）  读/写 （0x00）
#define SMS_ADDRESS_MinAngleH            0x0A    //最小角度限制（H）  读/写 （0x00）
#define SMS_ADDRESS_MaxAngleL            0x0B    //最大角度限制（L）  读/写 （0x0F）
#define SMS_ADDRESS_MaxAngleH            0x0C    //最大角度限制（H）  读/写 （0xFF）
#define SMS_ADDRESS_MaxTemp              0x0D    //最高温度上限       读/写 （0x50）
#define SMS_ADDRESS_MaxVoltage           0x0E    //最高输入电压       读/写 （0xFA）
#define SMS_ADDRESS_MinVoltage           0x0F    //最低输入电压       读/写 （0x32）
#define SMS_ADDRESS_MaxTorqueL           0x10    //最大扭矩（L）      读/写 （0x03）
#define SMS_ADDRESS_MaxTorqueH           0x11    //最大扭矩（H）      读/写 （0xE8）
#define SMS_ADDRESS_PwmModel             0x12    //PWM相位模式        读/写 （0x00）
#define SMS_ADDRESS_Unload               0x13    //卸载条件           读/写 （0x2F）
#define SMS_ADDRESS_LedAlarm             0x14    //LED报警条件        读/写 （0x2F）
#define SMS_ADDRESS_P                    0x15    //P                  读/写 （0x0F）
#define SMS_ADDRESS_D                    0x16    //D                  读/写 （0x00）
#define SMS_ADDRESS_I                    0x17    //I                  读/写 （0x00）
#define SMS_ADDRESS_MinPwmL              0x18    //最小PWM（L）       读/写 （0x64）
#define SMS_ADDRESS_MinPwmH              0x19    //最小PWM（H）       读/写 （0x64）
#define SMS_ADDRESS_ClockwiseNSA         0x1A    //顺时针不灵敏区     读/写 （0x01）
#define SMS_ADDRESS_CClockwiseNSA        0x1B    //逆时针不灵敏区     读/写 （0x01）
#define SMS_ADDRESS_IntegralLimitL       0x1C    //积分限制（L）      读/写 （0x00）
#define SMS_ADDRESS_IntegralLimitH       0x1D    //积分限制（H）      读/写 （0x00）
#define SMS_ADDRESS_Derivative           0x1E    //微分采样系数       读/写 （0x00）
#define SMS_ADDRESS_Torsion              0x1F    //扭力步进           读/写 （0x00）
#define SMS_ADDRESS_Azimuth              0x20    //位置步进           读/写 （0x00）
#define SMS_ADDRESS_AzimuthReviseL       0x21    //位置校正(L)        读/写 （0x00）
#define SMS_ADDRESS_AzimuthReviseH       0x22    //位置校正(H)        读/写 （0x00）
#define SMS_ADDRESS_RunModel             0x23    //运行模式           读/写 （0x00）
#define SMS_ADDRESS_Protective           0x24    //保护电流(L)        读/写 （0x07）

#define SMS_ADDRESS_TorqueSwitch         0x28    //扭矩开关           读/写 （0x00） RAM 
#define SMS_ADDRESS_TargetAzimuthL       0x2A    //目标位置（L）      读/写
#define SMS_ADDRESS_TargetAzimuthH       0x2B    //目标位置（H）      读/写
#define SMS_ADDRESS_RunTimeL             0x2C    //运行时间（L）      读/写 （0x00）
#define SMS_ADDRESS_RunTimeH             0x2D    //运行时间（H）      读/写 （0x00）
#define SMS_ADDRESS_RunSpeedL            0x2E    //运行速度（L）      读/写 （0x00）
#define SMS_ADDRESS_RunSpeedH            0x2F    //运行速度（H）      读/写 （0x00）
#define SMS_ADDRESS_Locked               0x30    //锁标志             读/写 （0x00） 
#define SMS_ADDRESS_CurrentAzimuthL      0x38    //当前位置（L）      读
#define SMS_ADDRESS_CurrentAzimuthH      0x39    //当前位置（H）      读
#define SMS_ADDRESS_CurrentSpeedL        0x3A    //当前速度（L）      读
#define SMS_ADDRESS_CurrentSpeedH        0x3B    //当前速度（H）      读
#define SMS_ADDRESS_CurrentOverloadL     0x3C    //当前负载（L）      读 
#define SMS_ADDRESS_CurrentOverloadH     0x3D    //当前负载（H）      读
#define SMS_ADDRESS_CurrentVoltage       0x3E    //当前电压           读
#define SMS_ADDRESS_CurrentTemp          0x3F    //当前温度           读
#define SMS_ADDRESS_RegWrite             0x40    //REG WRITE标志      读 （0x00）
#define SMS_ADDRESS_Error                0x41    //Error              读 
#define SMS_ADDRESS_MoveFlag             0x42    //移动标志           读 
#define SMS_ADDRESS_CTargetAzimuthL      0x43    //当前目标位置（L）  读
#define SMS_ADDRESS_CTargetAzimuthH      0x44    //当前目标位置（H）  读
#define SMS_ADDRESS_CurrentElectricityL  0x45    //当前电流（L）      读
#define SMS_ADDRESS_CurrentElectricityH  0x46    //当前电流（H）      读


#define SCS_PROTOCOL_CMD_PING            0x01    //查询工作状态
#define SCS_PROTOCOL_CMD_READ            0x02    //查询控制表里的字符
#define SCS_PROTOCOL_CMD_WRITE           0x03    //往控制表里写入字符
#define SCS_PROTOCOL_CMD_REGWRITE        0x04    //类似于WRITE但是控制字符写入后并不马上动作直到ACTION指令到达
#define SCS_PROTOCOL_CMD_ACTION          0x05    //触发REGWRITE写入的动作
#define SCS_PROTOCOL_CMD_SYCNWRITE       0x83    //用于同时控制多个舵机

#define SMS_PROTOCOL_CMD_PING            0x01    //查询工作状态
#define SMS_PROTOCOL_CMD_READ            0x02    //查询控制表里的字符
#define SMS_PROTOCOL_CMD_WRITE           0x03    //往控制表里写入字符
#define SMS_PROTOCOL_CMD_REGWRITE        0x04    //类似于WRITE但是控制字符写入后并不马上动作直到ACTION指令到达
#define SMS_PROTOCOL_CMD_ACTION          0x05    //触发REGWRITE写入的动作
#define SMS_PROTOCOL_CMD_SYCNWRITE       0x83    //用于同时控制多个舵机
#define SMS_PROTOCOL_CMD_RESET           0x06    //把控制表复位为出厂值
 

#define SXS_PROTOCOL_HEAD                0xFFFF  //协议字头
#define SXS_PROTOCOL_BroadcastID         0xFE    //广播ID



#pragma pack(push, 1)

  struct SXS_Head
  {
    unsigned short m_nHead;       //协议头字节
    unsigned char m_nID;          //ID
    unsigned char m_nLength;      //数据长度

    SXS_Head()
    {
      memset(this, 0, sizeof(SXS_Head));
      m_nHead = SXS_PROTOCOL_HEAD;
    }

    inline bool IsRight(){return m_nHead == SXS_PROTOCOL_HEAD; }
  };

#pragma pack(pop)


  //力矩输出开关
  enum SXS_TorqueSwitch : unsigned char
  {
    TorqueOff = 0x00,
    TorqueOn = 0x01
  };

  //锁开关
  enum SXS_LockSwitch : unsigned char
  {
    LockOff = 0x00,
    LockOn = 0x01
  };

  //运行模式
  enum SMS_RunModel : unsigned char
  {
    //伺服控制模式：设置目标位置,运行时间,运行速度
    Servo =  0x00,
    //电机调速模式：设置两字节BIT(15~0)的速度值 的 最高 BIT(15)=0正向, BIT(15)=1反向, BIT(14~0)速度值N, 总速度=(N * 0.087)度/秒
    Speed =  0x01,
    //扭力模式：设置两字节BIT(15~0)的时间值 的 BIT(10)=0正向, BIT(10)=1反向, BIT(9~0)时间值范围1~1000
    Torque = 0x02
  };

  //舵机类型
  enum ServoTypes : unsigned char
  {
    SCS = 0x01,
    SMS = 0x02
  };

#define SXS_PROTOCOL_LEN_DATA(vec)        (1 + (vec.size()) + 1)    //指令(1) + 参数.size() + 校验(1)
#define SXS_PROTOCOL_LEN_DATAGRAM(vec)    (sizeof(SXS_Head) + SXS_PROTOCOL_LEN_DATA(vec))     //sizeof(HEAD) + DATA

}

#endif // !__H_SXS_MEMORY_TABLE_PROTOCOL_H__

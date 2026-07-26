#ifndef ELE_MOTOR_DEF_H
#define ELE_MOTOR_DEF_H

#include <cstdint>

typedef union {
	uint8_t uValue[4];
	float fValue;
} unionFloat;

enum ControlMode {
	IMPEDANCE = 0,  // 阻抗控制模式
	SPEED     = 1,  // 速度控制模式
	POSITION  = 2   // 位置控制模式
};

#define MOTOR_Set_Test                0x00      //测试，无实际用途

#define MOTOR_OR_Error                0x01      //错误信息
/*电机可观测信号*/
#define MOTOR_OR_Ia                   0x02      //A相电流
#define MOTOR_OR_Ib                   0x03      //B相电流
#define MOTOR_OR_Ic                   0x04      //C相电流
#define MOTOR_OR_Id                   0x05      //D轴电流
#define MOTOR_OR_Iq                   0x06      //Q轴电流
#define MOTOR_OR_Vbus                 0x07      //总线电压
#define MOTOR_OR_Vd                   0x08      //D轴电压
#define MOTOR_OR_Vq                   0x09      //Q轴电压
#define MOTOR_OR_Te                   0x0A      //电磁转矩
#define MOTOR_OR_Angel                0x0B      //电角度
#define MOTOR_OR_We                   0x0C      //电角速度
#define MOTOR_OR_temperature          0x0D      //电机温度
#define DRIVE_OR_temperature          0x0E      //驱动温度
#define MOTOR_OR_angle                0x0F      //角度
#define MOTOR_OR_velocity             0x10      //角速度
#define MOTOR_OR_torque               0x11      //扭矩
#define MOTOR_OR_error_register       0x12      //电机错误信息寄存器1 + 2
#define MOTOR_OR_error_history_register  0x13      //电机错误历史信息寄存器1 + 2


#define MOTOR_OW_Save_Patemeter       0x2A        //将参数存入FLASH
#define MOTOR_OW_Clear_Fault          0x2B        //清除错误

/*电机电气参数*/
#define MOTOR_WR_LD                   0x40        //电机d轴电感
#define MOTOR_WR_LQ                   0x41        //电机q轴电感
#define MOTOR_WR_FLUX                 0x42        //电机转子磁链
#define MOTOR_WR_RESISTANCE           0x43        //电机相电阻
/*电机机械参数*/
#define MOTOR_WR_GR                   0x44        //电机减速比 1.0-32.0
#define MOTOR_WR_J                    0x45        //电机转动惯量
#define MOTOR_WR_B                    0x46        //粘滞系数
#define MOTOR_WR_P                    0x47        //电机极对数
#define MOTOR_WR_Tf                   0x48        //静摩擦力矩
#define MOTOR_WR_KT_OUT               0x49        //电机电磁转矩系数 0.1 - 10.0 A/NM
/*可读写其他参数*/
#define MOTOR_WR_Major                0x50      //电机型号
#define MOTOR_WR_CAN_ID               0x51      //设置CANID，其中CANID范围1-15
#define MOTOR_WR_Current_Risetime     0x52      //电机电流环响应时间 单位us 最小值为：0.345*MOTOR_LD/MOTOR_RESISTANCE * 1e6
#define MOTOR_WR_Max_Angle            0x53      //限位最大角度 单位弧度
#define MOTOR_WR_Min_Angle            0x54      //限位最小角度 单位弧度
#define MOTOR_WR_Angle_Limit_Switch   0x55      //角度限位开关 0或1
#define MOTOR_WR_Current_Limit        0x56      //最大电流限制 0-60A
#define MOTOR_WR_CAN_Timeout          0x57      //CAN通信中,Timeout个周期未收到CAN信号，电机进入失能模式
#define MOTOR_WR_ECAT_ID              0x58      //设置ECAT_ID，其中ECAT_ID范围0-255
#define MOTOR_WR_temp_Protection      0x59      //电机温度保护阈值,为0时表示关闭温度保护
#define DRIVER_WR_temp_Protection     0x5A      //驱动温度保护阈值,为0时表示关闭温度保护
#define MOTOR_WR_CONTROL_MODE         0x5B      //控制模式，0阻抗控制 1速度控制 2位置控制
#define MOTOR_WR_Velfilter_constant   0x5C      //角速度低通滤波常数
#define MOTOR_WR_CAN_REPLY_ID         0x5D      //CAN返回ID号
#define MOTOR_WR_CAN_REPLY_MAX_ANGLE  0x5E      //CAN返回最大角度
#define MOTOR_WR_CAN_REPLY_MAX_Velocity  0x5F      //CAN返回最大角速度
#define MOTOR_WR_CAN_REPLY_MAX_Torque    0x60      //CAN返回最大扭矩
#define MOTOR_WR_CAN_REPLY_MAX_KP     0x61      //CAN控制最大P值
#define MOTOR_WR_CAN_REPLY_MAX_KD     0x62      //CAN控制最大D值
#define MOTOR_WR_VOLTAGE_MAX        0x63       //电机最大电压值
/*控制参数范围限制（协议编码量程）
 *
 * 说明：这里的 min/max 是 float↔uint 的编码量程，必须与电机固件端一致。
 *       原本是一组全局宏，所有电机共用。现改为按电机类型（Hip/Thigh/Calf/Wheel）
 *       分行的结构体表 MOTOR_LIMITS，方便按类型单独调整；默认四行数值完全相同，
 *       等于原宏值，不改变现有行为。
 *       位宽 bits（15/12/31）是协议固定的，不放进结构体，仍在编码处以字面量给出。
 */

// 电机类型，对应同一 CAN 口内的 motor_id 1~4（映射见 motor_calibration.h）
enum MotorType {
	MOTOR_HIP = 0,   // motor_id=1 髋/侧摆
	MOTOR_THIGH,     // motor_id=2 大腿
	MOTOR_CALF,      // motor_id=3 小腿/膝
	MOTOR_WHEEL,     // motor_id=4 轮
	MOTOR_TYPE_NUM
};

// 单个电机的编码量程限幅
struct MotorLimits {
	float p_min,  p_max;    // 位置 (rad)
	float v_min,  v_max;    // 速度 (rad/s)
	float t_min,  t_max;    // 扭矩 (Nm)
	float kp_min, kp_max;   // 比例增益
	float kd_min, kd_max;   // 微分增益
	float ki_min, ki_max;   // 积分增益
};

// 各类型电机的限幅表（默认四行相同 = 原宏值；按类型调整时改对应行即可）
static const MotorLimits MOTOR_LIMITS[MOTOR_TYPE_NUM] = {
	/* Hip   */ {-12.5f, 12.5f, -65.0f, 65.0f, -150.0f, 150.0f, 0.0f, 500.0f, 0.0f, 500.0f, 0.0f, 500.0f},
	/* Thigh */ {-12.5f, 12.5f, -65.0f, 65.0f, -150.0f, 150.0f, 0.0f, 500.0f, 0.0f, 500.0f, 0.0f, 500.0f},
	/* Calf  */ {-12.5f, 12.5f, -65.0f, 65.0f, -150.0f, 150.0f, 0.0f, 500.0f, 0.0f, 500.0f, 0.0f, 500.0f},
	/* Wheel */ {-12.5f, 12.5f, -65.0f, 65.0f, -53.0f, 53.0f, 0.0f, 500.0f, 0.0f, 500.0f, 0.0f, 500.0f},
};

// 按 motor_id(1~4) 取限幅；越界回退到 Hip 行
static inline const MotorLimits& limits_of(uint8_t motor_id) {
	int idx = (motor_id >= 1 && motor_id <= MOTOR_TYPE_NUM) ? (motor_id - 1) : 0;
	return MOTOR_LIMITS[idx];
}


/*电机控制命令*/
#define MOTOR_STRAT  0xFC              //电机启动命令
#define MOTOR_STOP  0xFD               //电机停止命令
#define MOTOR_ANGLE_ZERO  0xFE         //电机角度归零命令
#define MOTOR_CHANGE_ID  0xF9          //电机改变ID命令
#define MOTOR_ANGLE_CORRECTION  0xF7   //电机角度校正命令
#define MOTOR_ABSOLUTE_POSITION_CORRECT  0xF6  //电机绝对位置校正命令
#define MOTOR_CLEAR_ERROR  0xF4        //电机清除错误命令
#define MOTOR_PARAMETER_DECERN  0xed   //电机参数判断命令

#endif // ELE_MOTOR_DEF_H

/**
 * @file    ele_motor_drive.h
 * @brief   电机驱动扩展模块 - 参数读写、验证、编解码等高级功能
 * @details 基于 motor_rw_api.h 中的宏定义，提供完整的参数管理功能
 *          包括参数验证、参数读写、数据编解码等
 */
#ifndef ELE_MOTOR_DRIVE_H_
#define ELE_MOTOR_DRIVE_H_

#include <cstdint>
#include <vector>
#include "data_types.h"

// =====================================================================
//                    参数范围定义（来自 motor_rw_api.h）
// =====================================================================

#define P_MIN   -12.5f      // 位置最小值 (rad)
#define P_MAX   12.5f       // 位置最大值 (rad)
#define V_MIN   -14.0f      // 速度最小值 (rad/s)
#define V_MAX   14.0f       // 速度最大值 (rad/s)
#define KP_MIN  0.0f        // Kp最小值
#define KP_MAX  500.0f      // Kp最大值
#define KD_MIN  0.0f        // Kd最小值
#define KD_MAX  100.0f      // Kd最大值
#define KI_MIN  0.0f        // Ki最小值
#define KI_MAX  10000.0f    // Ki最大值
#define T_MIN   -10.0f     // 扭矩最小值 (Nm)
#define T_MAX   10.0f      // 扭矩最大值 (Nm)

// =====================================================================
//                    参数类型定义（来自 motor_rw_api.h）
// =====================================================================

// 电机可观测信号
#define MOTOR_OR_Error                0x01
#define MOTOR_OR_Ia                   0x02
#define MOTOR_OR_Ib                   0x03
#define MOTOR_OR_Ic                   0x04
#define MOTOR_OR_Id                   0x05
#define MOTOR_OR_Iq                   0x06
#define MOTOR_OR_Vbus                 0x07
#define MOTOR_OR_Vd                   0x08
#define MOTOR_OR_Vq                   0x09
#define MOTOR_OR_Te                   0x0A
#define MOTOR_OR_Angel                0x0B
#define MOTOR_OR_We                   0x0C
#define MOTOR_OR_temperature          0x0D
#define DRIVE_OR_temperature          0x0E
#define MOTOR_OR_angle                0x0F
#define MOTOR_OR_velocity             0x10
#define MOTOR_OR_torque               0x11
#define MOTOR_OR_error_register       0x12
#define MOTOR_OR_error_history_register 0x13

// 电机可写参数
#define MOTOR_WR_LD                   0x40
#define MOTOR_WR_LQ                   0x41
#define MOTOR_WR_FLUX                 0x42
#define MOTOR_WR_RESISTANCE           0x43
#define MOTOR_WR_GR                   0x44
#define MOTOR_WR_J                    0x45
#define MOTOR_WR_B                    0x46
#define MOTOR_WR_P                    0x47
#define MOTOR_WR_Tf                   0x48
#define MOTOR_WR_KT_OUT               0x49
#define MOTOR_WR_Major                0x50
#define MOTOR_WR_CAN_ID               0x51
#define MOTOR_WR_Current_Risetime     0x52
#define MOTOR_WR_Max_Angle            0x53
#define MOTOR_WR_Min_Angle            0x54
#define MOTOR_WR_Angle_Limit_Switch   0x55
#define MOTOR_WR_Current_Limit        0x56
#define MOTOR_WR_CAN_Timeout          0x57
#define MOTOR_WR_ECAT_ID              0x58
#define MOTOR_WR_temp_Protection      0x59
#define DRIVER_WR_temp_Protection     0x5A
#define MOTOR_WR_CONTROL_MODE         0x5B
#define MOTOR_WR_Velfilter_constant   0x5C
#define MOTOR_WR_CAN_REPLY_ID         0x5D
#define MOTOR_WR_CAN_REPLY_MAX_ANGLE  0x5E
#define MOTOR_WR_CAN_REPLY_MAX_Velocity 0x5F
#define MOTOR_WR_CAN_REPLY_MAX_Torque   0x60
#define MOTOR_WR_CAN_REPLY_MAX_KP     0x61
#define MOTOR_WR_CAN_REPLY_MAX_KD     0x62
#define MOTOR_WR_VOLTAGE_MAX          0x63

// =====================================================================
//                    参数验证器类
// =====================================================================

/**
 * @class MotorParamValidator
 * @brief 电机参数验证器 - 提供参数范围检查和截断功能
 */
class MotorParamValidator {
public:
    /**
     * @brief 检查并截断浮点参数到指定范围
     * @param value 输入值
     * @param min 最小值
     * @param max 最大值
     * @return 截断后的值
     */
    static float Clamp(float value, float min, float max);

    // 位置、速度、扭矩验证
    static float ValidatePosition(float pos);
    static float ValidateVelocity(float vel);
    static float ValidateTorque(float torque);

    // 增益参数验证
    static float ValidateKp(float kp);
    static float ValidateKd(float kd);
    static float ValidateKi(float ki);

    // 范围检查（返回bool）
    static bool IsInRange(float value, float min, float max);
    static bool IsPositionValid(float pos);
    static bool IsVelocityValid(float vel);
    static bool IsTorqueValid(float torque);
    static bool IsKpValid(float kp);
    static bool IsKdValid(float kd);
    static bool IsKiValid(float ki);
};

// =====================================================================
//                    参数编解码器类
// =====================================================================

/**
 * @class MotorParamCodec
 * @brief 电机参数编解码器 - 提供参数打包、解包、转换等功能
 */
class MotorParamCodec {
public:
    /**
     * @brief 将float参数打包成CAN帧数据
     * @param parameter 参数值
     * @param RW 读写标志 (0=读, 1=写)
     * @param type 参数类型 (MOTOR_WR_* 宏)
     * @param id 电机ID
     * @param data 输出缓冲区（8字节）
     * @details 按照硬件协议格式打包参数指令：
     *          {0x80, val[0], val[1], val[2], val[3], RW, param_type, 0xEC}
     */
    static void Float2Bag(float parameter, uint8_t RW, uint8_t type, uint8_t id, uint8_t* data);

    /**
     * @brief 从字节数组解析float参数
     * @param data 字节数组指针
     * @param key 解析关键字（0=标准, 1=从data[1]开始）
     * @return 解析出的float值
     */
    static float U8Array2Float(const uint8_t* data, uint8_t key = 0);

    /**
     * @brief 将float转换为无符号整数（量化）
     * @param x 浮点数
     * @param x_min 最小值
     * @param x_max 最大值
     * @param bits 量化位数
     * @return 量化后的无符号整数
     */
    static unsigned int FloatToUint(float x, float x_min, float x_max, int bits);

    /**
     * @brief 将无符号整数转换为float（反量化）
     * @param x_int 整数值
     * @param x_min 最小值
     * @param x_max 最大值
     * @param bits 量化位数
     * @return 反量化后的浮点数
     */
    static float UintToFloat(int x_int, float x_min, float x_max, int bits);

    /**
     * @brief 解析接收到的命令数据
     * @param data 命令数据指针
     * @return 解析出的float值
     */
    static float UnpackCmd(const uint8_t* data);

    /**
     * @brief 设置电机参数（蓝牙/串口接口）
     * @param p1-p5 参数值
     * @param model 电机型号
     * @param id 电机ID
     */
    static void SetMotorParaBt(float p1, float p2, float p3, float p4, float p5, int model, int id);
};

// =====================================================================
//                    参数管理器类
// =====================================================================

/**
 * @struct MotorParameter
 * @brief 电机参数结构体 - 存储电机的各种参数
 */
struct MotorParameter {
    // 电气参数
    float ld;                   // D轴电感
    float lq;                   // Q轴电感
    float flux;                 // 转子磁链
    float resistance;           // 相电阻

    // 机械参数
    float gr;                   // 减速比
    float j;                    // 转动惯量
    float b;                    // 粘滞系数
    uint8_t p;                  // 极对数
    float tf;                   // 静摩擦力矩
    float kt_out;               // 电磁转矩系数

    // 其他参数
    uint8_t can_id;             // CAN ID
    float current_limit;        // 最大电流限制
    float temp_protection;      // 温度保护阈值
    uint8_t control_mode;       // 控制模式

    MotorParameter() : ld(0), lq(0), flux(0), resistance(0),
                       gr(1.0f), j(0), b(0), p(0), tf(0), kt_out(1.0f),
                       can_id(0), current_limit(60.0f), temp_protection(0), control_mode(0) {}
};

/**
 * @class MotorParameterManager
 * @brief 电机参数管理器 - 管理电机参数的读写和存储
 */
class MotorParameterManager {
public:
    MotorParameterManager();
    ~MotorParameterManager();

    /**
     * @brief 读取电机参数
     * @param param_type 参数类型
     * @return 参数值
     */
    float ReadParameter(uint8_t param_type);

    /**
     * @brief 写入电机参数
     * @param param_type 参数类型
     * @param value 参数值
     * @return 是否成功
     */
    bool WriteParameter(uint8_t param_type, float value);

    /**
     * @brief 获取电机参数结构体
     * @return 电机参数结构体引用
     */
    const MotorParameter& GetParameters() const { return m_params; }

    /**
     * @brief 设置电机参数结构体
     * @param params 电机参数结构体
     */
    void SetParameters(const MotorParameter& params) { m_params = params; }

private:
    MotorParameter m_params;    // 电机参数存储
};

#endif // ELE_MOTOR_DRIVE_H_

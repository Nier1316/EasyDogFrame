#include "motor/ele_motor.h"
#include "transport/canet_transport.h"
#include "transport/can_transport.h"
#include "motor/ele_motor_def.h"
#include "motor/motor_calibration.h"
#include "common/motor_logger.h"

void EleMotor::init() {
    current_speed = 0.0f;
    current_torque = 0.0f;
    current_position = 0.0f;
    current_temp = 0.0f;
    target_speed = 0.0f;
    target_torque = 0.0f;
    target_position = 0.0f;
    error_code = 0;
    enabled = false;
    kp = 0.0f;
    kd = 0.0f;
    ki = 0.0f;
    kvp = 0.0f;
    control_mode = IMPEDANCE;  // 原先未初始化，是未定义值
    hw_control_mode = -1;      // 未知 → 首次下发必定先同步固件模式
    mode_settle_ticks = 0;
    vel_lp = 0.0f;             // 轮速滤波状态复位
    vel_lp_init = false;
}

void EleMotor::enable() {
    enabled = true;
    uint8_t start_frame[8] = {0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, MOTOR_STRAT};
    // 使能帧原先不记日志，导致 sendcan 里完全看不到使能时刻——
    // 排查"使能瞬间轮电机就转"只能靠推断。mode 列用 -2 标记使能帧。
    MotorLogger::GetInstance().LogSendCan(device_idx, motor_id, -2, start_frame);
    CanFrame f;
    f.id = motor_id; f.dlc = 8; f.is_extended = 0;
    std::memcpy(f.data, start_frame, 8);
    if (transport) transport->send(device_idx, f);
}

void EleMotor::disable() {
    enabled = false;
    // 发送停止命令帧：80 FF FF FF FF FF FF FD
    uint8_t stop_frame[8] = {0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, MOTOR_STOP};
    // mode 列用 -3 标记失能帧
    MotorLogger::GetInstance().LogSendCan(device_idx, motor_id, -3, stop_frame);
    CanFrame f;
    f.id = motor_id; f.dlc = 8; f.is_extended = 0;
    std::memcpy(f.data, stop_frame, 8);
    if (transport) transport->send(device_idx, f);
}

/** 读写电机参数
 *  @param motor: 电机结构体引用，包含device_idx和motor_id
 *  @param parameter: 设置的参数值，读时设置为0
 *  @param RW: 读写：置0读，置1写
 *  @param type: 参数类型，头文件中定义如：MOTOR_OR_temperature 读取电机温度
 *  @note
 */
void float2bag(const EleMotor& motor, float parameter, uint8_t RW, uint8_t type){
    unsigned char *pdata = (unsigned char *)&parameter;
    uint8_t temp[8] = {0x80,*pdata++,*pdata++,*pdata++,*pdata++,RW,type,0xEC};//小端模式

    // 参数读写帧也记进 sendcan 日志：写控制模式(0x5B)、清错、归零都走这里，
    // 原先不记录，排查模式切换瞬态时只能从 send 日志的 mode 列反推时刻。
    // mode 列填 -1 以区别于 set_motor_para_bt 的 0/1/2。
    MotorLogger::GetInstance().LogSendCan(motor.device_idx, motor.motor_id, -1, temp);

    CanFrame f;
    f.id = motor.motor_id; f.dlc = 8; f.is_extended = 0;
    std::memcpy(f.data, temp, 8);
    if (motor.transport) motor.transport->send(motor.device_idx, f);
}

//控制指令：
//设置PD参数,参数对应关系看motorConsole上位机,例如速度控制p1:期望速度 p2:速度环kp p3:无设置 p4:无设置 p5:速度环ki,model为电机模式
/** 控制指令：
 *  @param motor: 电机结构体引用，包含device_idx和motor_id
 *  @param p1~p5: 对应相应的发送参数
 *  @param model: 控制模式（IMPEDANCE/SPEED/POSITION）
 *  @note 必须确保电机处于正确的控制模式，如：float2bag(motor, 0, 1, MOTOR_WR_CONTROL_MODE);切换到阻抗控制
 *        函数中的temp为CAN的8个数据帧
 */
void set_motor_para_bt(const EleMotor& motor, float p1, float p2, float p3, float p4, float p5, int model){
    uint8_t temp[8];
    const MotorLimits& lim = limits_of(motor.motor_id);                //按电机类型取限幅
    if(model == IMPEDANCE){                                            //阻抗控制模式
        uint16_t p_int = float_to_uint(p1, lim.p_min, lim.p_max, 15);   //期望角度   单位：弧度
        uint16_t v_int = float_to_uint(p2, lim.v_min, lim.v_max, 12);   //期望角速度 单位：弧度每秒
        uint16_t kp_int = float_to_uint(p3, lim.kp_min, lim.kp_max, 12); //刚度系数
        uint16_t kd_int = float_to_uint(p4, lim.kd_min, lim.kd_max, 12); //阻尼系数
        uint16_t t_int = float_to_uint(p5, lim.t_min, lim.t_max, 12);    //扭矩前馈   单位：牛米
        temp[0] = (uint8_t)(p_int >> 8 & 0x7f);
        temp[1] = (uint8_t)(p_int & 0xFF);
        temp[2] = (uint8_t)(v_int >> 4);
        temp[3] = (uint8_t)(((v_int & 0xF) << 4) | (kp_int >> 8));
        temp[4] = (uint8_t)(kp_int & 0xFF);
        temp[5] = (uint8_t)(kd_int >> 4);
        temp[6] = (uint8_t)(((kd_int & 0xF) << 4) | (t_int >> 8));
        temp[7] = (uint8_t)(t_int & 0xff);
    }
    else if(model == SPEED){                                           //速度控制模式
        uint32_t v_int = float_to_uint(p1, lim.v_min, lim.v_max, 31);   //期望角速度 单位：弧度每秒
        uint16_t kvp_int = float_to_uint(p2, lim.kp_min, lim.kp_max, 16); //速度环Kp
        uint16_t kvi_int = float_to_uint(p5, lim.ki_min, lim.ki_max, 16); //速度环Ki
        temp[0] = (uint8_t)(v_int >> 24 & 0x7f);
        temp[1] = (uint8_t)(v_int >> 16 & 0xFF);
        temp[2] = (uint8_t)(v_int >> 8 & 0xFF);
        temp[3] = (uint8_t)(v_int & 0xFF);
        temp[4] = (uint8_t)(kvp_int >> 8 & 0xFF);
        temp[5] = (uint8_t)(kvp_int & 0xff);
        temp[6] = (uint8_t)(kvi_int >> 8 & 0xFF);
        temp[7] = (uint8_t)(kvi_int & 0xff);
    }
    else if(model == POSITION){                                         //位置控制模式
        uint16_t p_int = float_to_uint(p1, lim.p_min, lim.p_max, 15);
        uint16_t kvp_int = float_to_uint(p2, lim.kp_min, lim.kp_max, 12); //位置环Kp
        uint16_t kp_int = float_to_uint(p3, lim.kp_min, lim.kp_max, 12);  //速度环Kp
        uint16_t kd_int = float_to_uint(p4, lim.kd_min, lim.kd_max, 12);  //位置环Kd
        uint16_t kvi_int = float_to_uint(p5, lim.ki_min, lim.ki_max, 12); //速度环Ki
        temp[0] = (uint8_t)(p_int >> 8 & 0x7f);
        temp[1] = (uint8_t)(p_int & 0xFF);
        temp[2] = (uint8_t)(kvp_int >> 4);
        temp[3] = (uint8_t)(((kvp_int & 0xF) << 4) | (kp_int >> 8));
        temp[4] = (uint8_t)(kp_int & 0xFF);
        temp[5] = (uint8_t)(kd_int >> 4);
        temp[6] = (uint8_t)(((kd_int & 0xF) << 4) | (kvi_int >> 8));
        temp[7] = (uint8_t)(kvi_int & 0xff);
    }

    // 记录编码后、真正上线的原始 CAN 帧（含轮电机 motor_id=4）
    MotorLogger::GetInstance().LogSendCan(motor.device_idx, motor.motor_id, model, temp);

    CanFrame f;
    f.id = motor.motor_id; f.dlc = 8; f.is_extended = 0;
    std::memcpy(f.data, temp, 8);
    if (motor.transport) motor.transport->send(motor.device_idx, f);
}


// 参数回帧详细打印开关，见 unpack_frame 内的说明。默认关闭。
bool g_param_verbose = false;

// ---- 轮速低通滤波（2026-08-28，抑制 USB2CAN 丢帧/解码跳变）----
// 现象：轮速反馈偶发跳变（FR 轮瞬时 ±4~8 rad/s，pos 在 ±0.06 rad 往复，物理不可能），
// 跳变会诱导策略对假轮速过激响应 + SendOnce 轮控误算 → 正反馈疯转。
// 处理：unpack 更新 current_speed 时对轮子做一阶低通（alpha=0.2 @ 500Hz，时间常数 ~10ms），
// 单帧尖峰被削平，真实速度趋势保留。策略观测（vel_policy）与 SendOnce 轮控共用滤波值。
// 只对轮子（motor_id==4）滤波，腿关节速度反馈保持原样（不影响已验证的腿控）。
static float FilterWheelVel(EleMotor& motor, float raw_vel) {
    if (motor.motor_id != MOTORS_PER_CAN) return raw_vel;   // 非轮子不过滤
    if (!motor.vel_lp_init) {
        motor.vel_lp = raw_vel;                              // 首帧对齐
        motor.vel_lp_init = true;
    } else {
        motor.vel_lp += 0.2f * (raw_vel - motor.vel_lp);     // 一阶低通
    }
    return motor.vel_lp;
}

// 直接解包 CAN 帧数据（不再接收，只解析）
void unpack_frame(EleMotor& motor, const uint8_t* data, uint8_t dlc) {
	if (!data || dlc < 6) {
		return;
	}

	if ((data[0] & 0x80) && (data[5] & 0x7f)) {
		// float2bag 读写参数后，电机返回数据包解包
		uint8_t type = data[5];
		unionFloat canRecev;
		canRecev.uValue[0] = data[1];
		canRecev.uValue[1] = data[2];
		canRecev.uValue[2] = data[3];
		canRecev.uValue[3] = data[4];

		// 诊断开关：置 1 时把所有参数回帧都打印出来。
		// 默认 0——角度/速度/扭矩走正常解包路径不打印，否则 1kHz 会淹没终端。
		// 需要看使能前的原始读数（如判断速度反馈是否上电即为假值）时开启。
		if (g_param_verbose) {
			printf("[PARAM] CAN%d motor%d type=0x%02X value=%.6f\n",
				   motor.device_idx, motor.motor_id, type, canRecev.fValue);
		}

		// 根据参数类型更新电机结构体
		switch (type) {
			case MOTOR_OR_temperature:
				motor.current_temp = canRecev.fValue;
				break;
			case MOTOR_OR_angle: {
				float raw_pos = canRecev.fValue;
				motor.current_position = raw_pos;
				float before[3] = {motor.current_position, motor.current_speed, motor.current_torque};
				ApplyMotorCalibration(motor.device_idx, motor.motor_id,
									  motor.current_position, motor.current_speed, motor.current_torque);
				MotorLogger::GetInstance().LogRecv(motor.device_idx, motor.motor_id,
					before[0], before[1], before[2],
					motor.current_position, motor.current_speed, motor.current_torque);
				break;
			}
			case MOTOR_OR_velocity: {
				float raw_vel = canRecev.fValue;
				motor.current_speed = FilterWheelVel(motor, raw_vel);
				float before[3] = {motor.current_position, motor.current_speed, motor.current_torque};
				ApplyMotorCalibration(motor.device_idx, motor.motor_id,
									  motor.current_position, motor.current_speed, motor.current_torque);
				MotorLogger::GetInstance().LogRecv(motor.device_idx, motor.motor_id,
					before[0], before[1], before[2],
					motor.current_position, motor.current_speed, motor.current_torque);
				break;
			}
			case MOTOR_OR_torque:
				motor.current_torque = canRecev.fValue;
				break;
			case MOTOR_OR_error_register:
				motor.error_code = (uint16_t)canRecev.fValue;
				break;
			default:
				// 其余寄存器（量程 0x60~0x62、控制模式 0x5B、电流限制 0x56、
				// 转矩系数 0x49 等）原先在此静默丢弃，读回来什么都看不到。
				// 这些不是周期性回帧，无论 verbose 与否都打印。
				if (!g_param_verbose) {
					printf("[PARAM] CAN%d motor%d type=0x%02X value=%.6f\n",
						   motor.device_idx, motor.motor_id, type, canRecev.fValue);
				}
				break;
		}
	} else {
		// set_motor_para_bt 设置参数后，电机返回数据包解包
		uint16_t p_int = (data[1] << 8) | data[2];
		uint16_t v_int = (data[3] << 4) | (data[4] >> 4);
		uint16_t i_int = ((data[4] & 0xF) << 8) | data[5];

		const MotorLimits& lim = limits_of(motor.motor_id);
		float p = uint_to_float(p_int, lim.p_min, lim.p_max, 16);
		float v = uint_to_float(v_int, lim.v_min, lim.v_max, 12);
		float t = uint_to_float(i_int, -lim.t_max, lim.t_max, 12);

		// 更新电机结构体（原始值）；轮速过一阶低通抑制解码跳变
		motor.current_position = p;
		motor.current_speed = FilterWheelVel(motor, v);
		motor.current_torque = t;

		// 记录原始值
		float raw_pos = p, raw_vel = v, raw_torque = t;

		// 应用标定参数
		ApplyMotorCalibration(motor.device_idx, motor.motor_id,
							  motor.current_position, motor.current_speed, motor.current_torque);

		// 日志: 原始反馈 → 标定后状态
		MotorLogger::GetInstance().LogRecv(motor.device_idx, motor.motor_id,
			raw_pos, raw_vel, raw_torque,
			motor.current_position, motor.current_speed, motor.current_torque);
	}
}

////////////////////////////用户使用函数//////////////////////////////////////

/////////////////////////////底层函数////////////////////////////////
float uint_to_float(int x_int, float x_min, float x_max, int bits) {
     /// converts unsigned int to float, given range and number of bits ///
     // 用 1u<<bits 做无符号移位，避免 bits=31 时 1<<31 的有符号整数溢出(UB)
     float span = x_max - x_min; float offset = x_min;
     return ((float)x_int)*span/((float)((1u<<bits)-1u)) + offset;
}

//float转uint
unsigned int float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    // 用 1u<<bits 做无符号移位，避免 bits=31 时 1<<31 的有符号整数溢出(UB)
    float span = x_max - x_min;
    float offset = x_min;
    return ((x-offset)*(float)((1u<<bits)-1u)/span);
}

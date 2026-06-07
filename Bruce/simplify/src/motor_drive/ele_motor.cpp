#include "ele_motor.h"
#include "bsp_can.h"
#include "ele_motor_def.h"
#include "motor_calibration.h"
#include <vector>

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
}

void EleMotor::enable() {
    enabled = true;
    uint8_t start_frame[8] = {0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, MOTOR_STRAT};
    BspCan::GetInstance().Can_Tx(device_idx, motor_id, start_frame, 8);
}

void EleMotor::disable() {
    enabled = false;
    // 发送停止命令帧：80 FF FF FF FF FF FF FD
    uint8_t stop_frame[8] = {0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, MOTOR_STOP};
    BspCan::GetInstance().Can_Tx(device_idx, motor_id, stop_frame, 8);
}

bool EleMotor::has_error() const {
    return error_code != 0;
}

void EleMotor::clear_error() {
    error_code = 0;
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

    BspCan::GetInstance().Can_Tx(motor.device_idx, motor.motor_id, temp, 8);
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
    if(model == IMPEDANCE){                                            //阻抗控制模式
        uint16_t p_int = float_to_uint(p1, P_MIN, P_MAX, 15);          //期望角度   单位：弧度
        uint16_t v_int = float_to_uint(p2, V_MIN, V_MAX, 12);          //期望角速度 单位：弧度每秒
        uint16_t kp_int = float_to_uint(p3, KP_MIN, KP_MAX, 12);       //刚度系数
        uint16_t kd_int = float_to_uint(p4, KD_MIN, KD_MAX, 12);       //阻尼系数
        uint16_t t_int = float_to_uint(p5, T_MIN, T_MAX, 12);          //扭矩前馈   单位：牛米
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
        uint32_t v_int = float_to_uint(p1, V_MIN, V_MAX, 31);          //期望角速度 单位：弧度每秒
        uint16_t kvp_int = float_to_uint(p2, KP_MIN, KP_MAX, 16);      //速度环Kp
        uint16_t kvi_int = float_to_uint(p5, KI_MIN, KI_MAX, 16);      //速度环Ki
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
        uint16_t p_int = float_to_uint(p1, P_MIN, P_MAX, 15);
        uint16_t kvp_int = float_to_uint(p2, KP_MIN, KP_MAX, 12);       //位置环Kp
        uint16_t kp_int = float_to_uint(p3, KP_MIN, KP_MAX, 12);        //速度环Kp
        uint16_t kd_int = float_to_uint(p4, KD_MIN, KD_MAX, 12);        //位置环Kd
        uint16_t kvi_int = float_to_uint(p5, KI_MIN, KI_MAX, 12);       //速度环Ki
        temp[0] = (uint8_t)(p_int >> 8 & 0x7f);
        temp[1] = (uint8_t)(p_int & 0xFF);
        temp[2] = (uint8_t)(kvp_int >> 4);
        temp[3] = (uint8_t)(((kvp_int & 0xF) << 4) | (kp_int >> 8));
        temp[4] = (uint8_t)(kp_int & 0xFF);
        temp[5] = (uint8_t)(kd_int >> 4);
        temp[6] = (uint8_t)(((kd_int & 0xF) << 4) | (kvi_int >> 8));
        temp[7] = (uint8_t)(kvi_int & 0xff);
    }

    BspCan::GetInstance().Can_Tx(motor.device_idx, motor.motor_id, temp, 8);
}


/** CAN数据解包
 *  @param motor: 电机结构体引用，包含device_idx和接收的数据
 *  @param timeout_ms: 接收超时时间（毫秒）
 *  @return: 成功返回true，失败返回false
 */
bool unpack_cmd(EleMotor& motor, int timeout_ms)
{
	std::vector<BspCanFrame> frames;

	if (!BspCan::GetInstance().ReceiveFrames(motor.device_idx, frames, timeout_ms)) {
		return false;
	}

	for (const auto& frame : frames) {
		uint8_t *data = (uint8_t *)frame.data;

		if ((data[0] & 0x80) && (data[5] & 0x7f)) {
			// float2bag读写参数后，电机返回数据包解包
			uint8_t type = data[5];
			unionFloat canRecev;
			canRecev.uValue[0] = data[1];
			canRecev.uValue[1] = data[2];
			canRecev.uValue[2] = data[3];
			canRecev.uValue[3] = data[4];

			// 根据参数类型更新电机结构体
			switch (type) {
				case MOTOR_OR_temperature:
					motor.current_temp = canRecev.fValue;
					break;
				case MOTOR_OR_angle:
					motor.current_position = canRecev.fValue;
					break;
				case MOTOR_OR_velocity:
					motor.current_speed = canRecev.fValue;
					break;
				case MOTOR_OR_torque:
					motor.current_torque = canRecev.fValue;
					break;
				case MOTOR_OR_error_register:
					motor.error_code = (uint16_t)canRecev.fValue;
					break;
				default:
					break;
			}
		}
		else {
			// set_motor_para_bt设置参数后，电机返回数据包解包
			uint16_t id = data[0] & 0x0f;
			uint16_t p_int = (data[1] << 8) | data[2];
			uint16_t v_int = (data[3] << 4) | (data[4] >> 4);
			uint16_t i_int = ((data[4] & 0xF) << 8) | data[5];

			float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
			float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
			float t = uint_to_float(i_int, -T_MAX, T_MAX, 12);

			// 更新电机结构体
			motor.current_position = p;
			motor.current_speed = v;
			motor.current_torque = t;
		}
	}

	return true;
}

// 直接解包 CAN 帧数据（不再接收，只解析）
void unpack_frame(EleMotor& motor, const uint8_t* data, uint8_t dlc) {
	if (!data || dlc < 6) {
		return;
	}

	if ((data[0] & 0x80) && (data[5] & 0x7f)) {
		// float2bag 读写参数后，电机返回数据包解包
		uint8_t type = data[5];
		union {
			uint8_t uValue[4];
			float fValue;
		} canRecev;
		canRecev.uValue[0] = data[1];
		canRecev.uValue[1] = data[2];
		canRecev.uValue[2] = data[3];
		canRecev.uValue[3] = data[4];

		// 根据参数类型更新电机结构体
		switch (type) {
			case MOTOR_OR_temperature:
				motor.current_temp = canRecev.fValue;
				break;
			case MOTOR_OR_angle:
				motor.current_position = canRecev.fValue;
				break;
			case MOTOR_OR_velocity:
				motor.current_speed = canRecev.fValue;
				break;
			case MOTOR_OR_torque:
				motor.current_torque = canRecev.fValue;
				break;
			case MOTOR_OR_error_register:
				motor.error_code = (uint16_t)canRecev.fValue;
				break;
			default:
				break;
		}
	} else {
		// set_motor_para_bt 设置参数后，电机返回数据包解包
		uint16_t p_int = (data[1] << 8) | data[2];
		uint16_t v_int = (data[3] << 4) | (data[4] >> 4);
		uint16_t i_int = ((data[4] & 0xF) << 8) | data[5];

		float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
		float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
		float t = uint_to_float(i_int, -T_MAX, T_MAX, 12);

		// 更新电机结构体
		motor.current_position = p;
		motor.current_speed = v;
		motor.current_torque = t;

		// 应用标定参数
		ApplyMotorCalibration(motor.device_idx, motor.motor_id,
							  motor.current_position, motor.current_speed, motor.current_torque);
	}
}

////////////////////////////用户使用函数//////////////////////////////////////

/////////////////////////////底层函数////////////////////////////////
float uint_to_float(int x_int, float x_min, float x_max, int bits) {
     /// converts unsigned int to float, given range and number of bits ///
     float span = x_max - x_min; float offset = x_min;
     return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

//float转uint
unsigned int float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((x-offset)*(float)((unsigned int)(1<<bits)-1)/span);
}


#include "motor_rw_api.h"

////////////////////////////用户使用函数//////////////////////////////////////
/** 读写电机参数
 *  @param parameter   设置的参数值，读时设置为0
 *  @param RW          读写：置0读，置1写
 *  @param type        参数类型，头文件中定义如：MOTOR_OR_temperature 读取电机温度
 *  @param id          电机的ID号
 *  @note
 */
void float2bag(float parameter,uint8_t RW,uint8_t type,uint8_t id){
    unsigned char *pdata = (unsigned char *)&parameter;
    uint8_t temp[8] = {0x80,*pdata++,*pdata++,*pdata++,*pdata++,RW,type,0xEC};//小端模式
		TxMes.StdId = id;
		canTrans(temp); //使用CAN发送数据包，修改为自己程序中的CAN发送函数
}

//控制指令：
//设置PD参数,参数对应关系看motorConsole上位机,例如速度控制p1:期望速度 p2:速度环kp p3:无设置 p4:无设置 p5:速度环ki,model为电机模式,id为电机id
/** 控制指令：
 *  @param p1~p5       对应相应的发送参数
 *  @param id          电机的ID号
 *  @note 必须确保电机处于正确的控制模式，如：float2bag(IMPEDANCE,1,MOTOR_WR_CONTROL_MODE,id);切换到阻抗控制
 *        函数中的temp为CAN的8个数据帧
 */
void set_motor_para_bt(float p1,float p2,float p3,float p4,float p5,int model,int id){
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
		TxMes.StdId = id;//设置CAN的ID帧，修改为自己程序中的CAN数据结构体
		canTrans(temp); //使用CAN发送数据包，修改为自己程序中的CAN发送函数
}

/** //CAN数据解包
 */
float unpack_cmd(uint8_t *data)
{
	if((data[0]&0x80) && data[5]&0x7f){	//float2bag读写参数后，电机返回数据包解包
		uint8_t type = data[5]; //电机返回的是哪个参数
		unionFloat canRecev;
		canRecev.uValue[0] = data[1]; //canRecev.fValue中为返回的对应参数的值
		canRecev.uValue[1] = data[2];
		canRecev.uValue[2] = data[3];
		canRecev.uValue[3] = data[4];
		//motor error.uValue[0] = controller.motor error register2>>8;
		//motor error.uValue[1] = controller.motor error register2;
		//motor error.uValue[2] = controller.motor error register1>>8;
		//motor error.uValue[3] = controller.motor error register1;
		//return value = motor error.fValue,
		// uint8_t temp[13];
		// sprintf(temp,"d:%.2f,t:%d",canRecev.fValue,type); // float 到 char
		// HAL_UART_Transmit(&huart1,(uint8_t *)temp,12,1000);
	}
	else{	//set_motor_para_bt设置参数后，电机返回数据包解包
		uint16_t id = data[0] & 0x0f;                                    //接收到的id
		uint16_t p_int = (data[1]<<8)|data[2];
		uint16_t v_int = (data[3]<<4)|(data[4]>>4);
		uint16_t i_int = ((data[4]&0xF)<<8)|data[5];
		/// convert uints to floats ///
		float p = uint_to_float(p_int, P_MIN, P_MAX, 16);                //返回的电机角度
		float v = uint_to_float(v_int, V_MIN, V_MAX, 12);                //返回的电机角速度
		float t = uint_to_float(i_int, -T_MAX, T_MAX, 12);               //返回的电机扭矩
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

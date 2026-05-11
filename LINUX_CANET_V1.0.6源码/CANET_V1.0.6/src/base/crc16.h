#ifndef __CRC16_H_ 
#define __CRC16_H_ 

#include <stdint.h>

#ifdef __cplusplus 
extern "C" { 
#endif 

/** 
  * \brief CRC16 计算 
  * \param p_buf   : 计算数据 
  * \param len     : 计算数据长度 
  * \param crc_val : 用于多次CRC校验，第一次校验时，值为0xFFFF,其后校验沿用上次校验值 
  * 
* */ 
uint16_t crc16_calc (const uint8_t *p_buf, uint16_t len, uint16_t crc_val); 

/**
* \brief BCC 计算
* \param data   : 计算数据
* \param len     : 计算数据长度
* \param bcc_val : 用于多次CRC校验，第一次校验时，值为0x0000,其后校验沿用上次校验值
*
* */
uint8_t bcc_calc(const uint8_t *data, uint16_t len, uint8_t bcc_val);
#ifdef __cplusplus 
} 
#endif 

#endif /* __CRC16_H_ */ 


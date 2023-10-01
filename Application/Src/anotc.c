/*
 *  anotc.c
 *
 *  Created on: 2021年11月5日
 *      Author: LBQ
 */

#include "usart.h"
#include "stm32f4xx_hal_uart.h"
#include "anotc.h"
#include "PID.h"

#define UserDataLen		4	// 数据长度
#define ANOTC_USART		huart2
// 数据拆分宏定义, 在发送大于1字节的数据类型时, 需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)	( *( (char *)(&dwTemp)    ) )   /* !< uint32_t 数据拆分 byte0 低字节 */
#define BYTE1(dwTemp)	( *( (char *)(&dwTemp) + 1) )   /* !< uint32_t 数据拆分 byte1 */
#define BYTE2(dwTemp)	( *( (char *)(&dwTemp) + 2) )   /* !< uint32_t 数据拆分 byte2 */
#define BYTE3(dwTemp)	( *( (char *)(&dwTemp) + 3) )   /* !< uint32_t 数据拆分 byte3 */

extern UART_HandleTypeDef huart2;
extern PIDTypeDef_t PID_Platform[PlatformPID_NUM];

/*
 * @function:   Send to Anotc
 * @param:      none
 * @return:     none
 * @notion:     V7
 */
void Anotc_Send(void)
{
    uint8_t waveform[6 + UserDataLen] = {0};// 数据帧缓存
    uint8_t _cnt = 0;

    waveform[_cnt ++ ] = 0xAA;        		// 帧头
    waveform[_cnt ++ ] = 0xFF;        		// 目标地址
    waveform[_cnt ++ ] = 0xF1;				// 功能码ID
	waveform[_cnt ++ ] = UserDataLen;		// 有效数据长度

	// 要发送到上位机的数据
	uint16_t UserData_1 = (uint16_t)PID_Platform[PlatformPID_X].Now_Val;
	uint16_t UserData_2 = (uint16_t)PID_Platform[PlatformPID_X].EX_Val;

    // 数据区
	// 使用小端模式, 低字节在前
    waveform[_cnt ++ ] = BYTE0(UserData_1);   // 数据内容
    waveform[_cnt ++ ] = BYTE1(UserData_1);
    waveform[_cnt ++ ] = BYTE0(UserData_2);
    waveform[_cnt ++ ] = BYTE1(UserData_2);

    // 写入有效数据字节数
    waveform[3] = _cnt - 4;
    // 计算并写入校验
    uint8_t sumcheck = 0;   				// 和校验SC
    uint8_t addcheck = 0;   				// 附加校验AC
    for(uint8_t i = 0; i < waveform[3] + 4; i ++ ) {
      sumcheck += waveform[i];      		// 从帧头开始, 一直到 data 区结束, 对每一字节进行累加操作, 只取低8位
      addcheck += sumcheck;         		// 计算和校验时, 每进行一字节的加法运算, 同时进行一次 sumcheck 的累加操作, 只取低8位
    }
    waveform[_cnt ++ ] = sumcheck;
    waveform[_cnt ++ ] = addcheck;

    // 串口发送数据
    HAL_UART_Transmit_DMA(&ANOTC_USART, (uint8_t *)waveform, _cnt);
}

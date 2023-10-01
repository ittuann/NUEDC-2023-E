/*
 * communication.c
 *
 *  Created on: Aug 2, 2023
 *      Author: LBQ
 */

#include "main.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "communication.h"
#include "platformTask.h"

extern UART_HandleTypeDef huart2;

extern struct Square_t ElectricalTape;
extern struct Point_t RedPointNow;
extern volatile bool_t rxDataFlag;

static uint8_t communicationRxData[22];		// USART的DMA接收数据缓存

volatile bool_t ElectricalTapeInitFlag;		// 只读取一次胶带四个顶点

/**
* @breif   	初始化RS485收发器为接收模式
* @retval  	HAL_StatusTypeDef
*/
HAL_StatusTypeDef USARTCommunicationIT_Init(void)
{
	HAL_StatusTypeDef status;

	// 使能串口接收中断
	status = HAL_UART_Receive_DMA(&huart2, communicationRxData, sizeof(communicationRxData) / sizeof(uint8_t));
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);

	return status;
}

static uint8_t CRC8_Calculate(const uint8_t *data, uint8_t data_len)
{
    uint8_t crc = 0x00;			// CRC 初始值
    uint8_t crc_poly = 0x07;	// CRC Ploy值
    //依次输入数据
    while (data_len--) {
    	uint8_t data_in = *data ++ ;
        crc = crc ^ data_in;	// CRC的高八位异或输入数据
        // 判断8次CRC的做高位 并左移
        for (uint8_t i = 0; i < 8; i++) {
            if (crc & 0x80) {
            	// 最高位为1
                crc = (crc << 1) ^ crc_poly;
            } else {
            	// 最高位为0
                crc = crc << 1;
            }
        }
    }

    return crc ^ 0x00;
}

/**
* @breif   	通讯协议解析
*/
uint8_t CommunicationProtocolAnalysis(const uint8_t *data)
{
	if (data[0] == 0x1F) {
		if (CRC8_Calculate(data, 21) == data[21]) {
//			if (!ElectricalTapeInitFlag && (((uint16_t)data[1] << 8 | data[2]) != 0 && ((uint16_t)data[3] << 8 | data[4]) != 0)) {
				ElectricalTape.upperLeft.x = ((uint16_t)data[1] << 8 | data[2]);
				ElectricalTape.upperLeft.y = ((uint16_t)data[3] << 8 | data[4]);
				ElectricalTape.upperRight.x = ((uint16_t)data[5] << 8 | data[6]);
				ElectricalTape.upperRight.y = ((uint16_t)data[7] << 8 | data[8]);
				ElectricalTape.lowerRight.x = ((uint16_t)data[9] << 8 | data[10]);
				ElectricalTape.lowerRight.y = ((uint16_t)data[11] << 8 | data[12]);
				ElectricalTape.lowerLeft.x = ((uint16_t)data[13] << 8 | data[14]);
				ElectricalTape.lowerLeft.y = ((uint16_t)data[15] << 8 | data[16]);

				ElectricalTape.center.x = (ElectricalTape.upperLeft.x + ElectricalTape.lowerRight.x + ElectricalTape.upperRight.x + ElectricalTape.lowerLeft.x) / 4.0f;
				ElectricalTape.center.y = (ElectricalTape.upperLeft.y + ElectricalTape.lowerRight.y + ElectricalTape.upperRight.y + ElectricalTape.lowerLeft.y) / 4.0f;
				ElectricalTapeInitFlag = 1;
//			}

			HAL_GPIO_WritePin(LED1G_GPIO_Port, LED1G_Pin, GPIO_PIN_RESET);
			if (((uint16_t)data[1] << 8 | data[2]) == 0 && ((uint16_t)data[3] << 8 | data[4]) == 0) {
				HAL_GPIO_WritePin(LED1G_GPIO_Port, LED1G_Pin, GPIO_PIN_SET);
			}

			uint16_t nowX = (uint16_t)data[17] << 8 | data[18];
			uint16_t nowY = (uint16_t)data[19] << 8 | data[20];
			if (nowX == 0 && nowX != UINT16_MAX && nowY == 0 && nowY != UINT16_MAX) {
				rxDataTrueFlag = 0;
			} else {
				RedPointNow.x = nowX;
				RedPointNow.y = nowY;
				rxDataTrueFlag = 1;
			}

			return 1;
		} else {
			return 0;
		}
	} else {
		return 0;
	}
}

/**
 * @brief   串口接收中断回调函数
 * @note    reDefine
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//	if (huart->Instance == USART2) {
	if (huart == &huart2) {
		if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE) != RESET) {
			__HAL_UART_CLEAR_IDLEFLAG(&huart2);

			rxDataFlag = CommunicationProtocolAnalysis(communicationRxData);

			HAL_UART_Receive_DMA(&huart2, communicationRxData, sizeof(communicationRxData) / sizeof(uint8_t));
		}
    }
}

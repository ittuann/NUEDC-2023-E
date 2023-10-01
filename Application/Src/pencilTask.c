/*
 * pencilTask.c
 *
 *  Created on: Aug 2, 2023
 *      Author: LBQ
 */

#include <math.h>
#include "main.h"
#include "time.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "pencilTask.h"
#include "platformTask.h"
#include "w25qxx.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
extern uint16_t platformPWMInitVal;

extern volatile bool_t isKey0PressedStopFlag;
extern volatile bool_t isKey1PressedResetFlag;

static uint8_t singleKeyNum;			// 单次模式的按键编号
static uint8_t continuousKeyNum;		// 连续模式的按键编号
static uint8_t ConfirmKeyCnt;			// 确认按键按下的次数

struct Square_t PencilFrame;			// 铅笔正方形框
static struct Point_t PointCurrent;		// 当前激光位置 矩阵键盘调试时

static uint8_t pointTargetState;
static const uint16_t divisionsPencilNum = 100;
static uint16_t divisionsNumCnts;		// 线性插值 迭代增加数
static struct Point_t RedPointTarget;	// 线性插值 目标为方形的哪个角
static struct Point_t PointSet;			// 线性插值 单次输出值

static uint8_t waitStartInitTimeCnt;	// 在左上角短暂等待后再开始扫线
static const uint8_t waitStartInitTimeNum = 40;
bool_t isPencilScanDoneFlag = 0;		// 扫铅笔线线完成标志位

static uint32_t flashAddr = 0;			// flash开始地址
static bool_t isWriteAllFlashDoneFlag;	// flash一次写入完成标志位

/**
  * @brief  矩阵键盘扫描
  */
static uint8_t scanningMatrixSingleKeyboard(void)
{
	uint8_t num = 0;

	// 将所矩阵键盘有行ROW拉高
	HAL_GPIO_WritePin(MATRIX_OUT1_GPIO_Port, MATRIX_OUT1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MATRIX_OUT2_GPIO_Port, MATRIX_OUT2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MATRIX_OUT3_GPIO_Port, MATRIX_OUT3_Pin, GPIO_PIN_SET);

	// 拉低第一行 检测列COL
	HAL_GPIO_WritePin(MATRIX_OUT1_GPIO_Port, MATRIX_OUT1_Pin, GPIO_PIN_RESET);
	if (HAL_GPIO_ReadPin(MATRIX_IN2_GPIO_Port, MATRIX_IN2_Pin) == GPIO_PIN_RESET) num = 2;
	if (HAL_GPIO_ReadPin(MATRIX_IN4_GPIO_Port, MATRIX_IN4_Pin) == GPIO_PIN_RESET) num = 4;
	HAL_GPIO_WritePin(MATRIX_OUT1_GPIO_Port, MATRIX_OUT1_Pin, GPIO_PIN_SET); // 重新拉高第一行

	HAL_GPIO_WritePin(MATRIX_OUT2_GPIO_Port, MATRIX_OUT2_Pin, GPIO_PIN_RESET);
	if (HAL_GPIO_ReadPin(MATRIX_IN1_GPIO_Port, MATRIX_IN1_Pin) == GPIO_PIN_RESET) num = 5;
	if (HAL_GPIO_ReadPin(MATRIX_IN2_GPIO_Port, MATRIX_IN2_Pin) == GPIO_PIN_RESET) num = 6;
	if (HAL_GPIO_ReadPin(MATRIX_IN3_GPIO_Port, MATRIX_IN3_Pin) == GPIO_PIN_RESET) num = 7;
	if (HAL_GPIO_ReadPin(MATRIX_IN4_GPIO_Port, MATRIX_IN4_Pin) == GPIO_PIN_RESET) num = 8;
	HAL_GPIO_WritePin(MATRIX_OUT2_GPIO_Port, MATRIX_OUT2_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(MATRIX_OUT3_GPIO_Port, MATRIX_OUT3_Pin, GPIO_PIN_RESET);
	if (HAL_GPIO_ReadPin(MATRIX_IN2_GPIO_Port, MATRIX_IN2_Pin) == GPIO_PIN_RESET) num = 10;
	HAL_GPIO_WritePin(MATRIX_OUT3_GPIO_Port, MATRIX_OUT3_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(MATRIX_OUT4_GPIO_Port, MATRIX_OUT4_Pin, GPIO_PIN_RESET);
	if (HAL_GPIO_ReadPin(MATRIX_IN1_GPIO_Port, MATRIX_IN1_Pin) == GPIO_PIN_RESET) num = 13;
	if (HAL_GPIO_ReadPin(MATRIX_IN2_GPIO_Port, MATRIX_IN2_Pin) == GPIO_PIN_RESET) num = 14;
	if (HAL_GPIO_ReadPin(MATRIX_IN4_GPIO_Port, MATRIX_IN4_Pin) == GPIO_PIN_RESET) num = 16;
	HAL_GPIO_WritePin(MATRIX_OUT4_GPIO_Port, MATRIX_OUT4_Pin, GPIO_PIN_SET);

	return num;
}

/**
  * @brief  按键三步消抖法
  */
static uint8_t readTureKey(void)
{
	static uint8_t key_val, key_down, key_old;
	key_val = scanningMatrixSingleKeyboard();
	key_down = key_val & (key_old ^ key_val);
	key_old = key_val;
	return key_down;
}

static union Uint16Bytes_u {
    uint16_t uint16Value;
    unsigned char bytes[2];	// 按照0为高字节,1为低字节的顺序存储数据
} u16b;

void writePointFlash(uint16_t point)
{
	uint8_t tmp = 0;
	u16b.uint16Value = point;
	W25qxx_EraseSector(flashAddr);
	W25qxx_WriteByte(u16b.bytes[0], flashAddr);
	W25qxx_ReadByte(&tmp, flashAddr);
	flashAddr ++ ;
	W25qxx_EraseSector(flashAddr);
	W25qxx_WriteByte(u16b.bytes[1], flashAddr);
	W25qxx_ReadByte(&tmp, flashAddr);
	flashAddr ++ ;
}
uint16_t readPointFlash(void)
{
	W25qxx_ReadByte(&u16b.bytes[0], flashAddr);
	flashAddr ++ ;
	W25qxx_ReadByte(&u16b.bytes[1], flashAddr);
	flashAddr ++ ;
	return u16b.uint16Value;
}

void writeAllPointFlash(void)
{
	W25qxx_EraseChip();
	writePointFlash((uint16_t)PencilFrame.center.x);
	writePointFlash((uint16_t)PencilFrame.center.y);
	writePointFlash((uint16_t)PencilFrame.upperLeft.x);
	writePointFlash((uint16_t)PencilFrame.upperLeft.y);
	writePointFlash((uint16_t)PencilFrame.upperRight.x);
	writePointFlash((uint16_t)PencilFrame.upperRight.y);
	writePointFlash((uint16_t)PencilFrame.lowerRight.x);
	writePointFlash((uint16_t)PencilFrame.lowerRight.y);
	writePointFlash((uint16_t)PencilFrame.lowerLeft.x);
	writePointFlash((uint16_t)PencilFrame.lowerLeft.y);
}

void readAllPointFlash(void)
{
	PencilFrame.upperLeft.x = PencilFrame.upperRight.x = PencilFrame.lowerRight.x = PencilFrame.lowerLeft.x = platformPWMInitVal;
	PencilFrame.upperLeft.y = PencilFrame.upperRight.y = PencilFrame.lowerRight.y = PencilFrame.lowerLeft.y = platformPWMInitVal;
	PencilFrame.center.x = PencilFrame.center.y = platformPWMInitVal;
	PointCurrent.x = PointCurrent.y = platformPWMInitVal;

	PencilFrame.center.x = readPointFlash();
	PencilFrame.center.y = readPointFlash();
	PencilFrame.upperLeft.x = readPointFlash();
	PencilFrame.upperLeft.y = readPointFlash();
	PencilFrame.upperRight.x = readPointFlash();
	PencilFrame.upperRight.y = readPointFlash();
	PencilFrame.lowerRight.x = readPointFlash();
	PencilFrame.lowerRight.y = readPointFlash();
	PencilFrame.lowerLeft.x = readPointFlash();
	PencilFrame.lowerLeft.y = readPointFlash();
}

/**
  * @brief  Function implementing the basicTask thread.
  * @param  argument: Not used
  * @retval None
  */
void PencilTask(void const * argument)
{
	flashAddr = 0;

	  HAL_GPIO_WritePin(NORFLASH_SPICS_GPIO_Port, NORFLASH_SPICS_Pin, GPIO_PIN_SET);
	  W25qxx_Init();
	  osDelay(10);
	  readAllPointFlash();

	while(1)
	{
		uint8_t stepPWm = 1;

		continuousKeyNum = scanningMatrixSingleKeyboard();
		if (continuousKeyNum == 2 || continuousKeyNum == 5 || continuousKeyNum == 6 || continuousKeyNum == 7 || continuousKeyNum == 10 || continuousKeyNum == 16) {
			continuousKeyNum = 0;
		}
		singleKeyNum = readTureKey();

		// 连续按键
		if (continuousKeyNum != 0) {
			switch (continuousKeyNum) {
			  case 4:
				  PointCurrent.y -= stepPWm;
				  __HAL_TIM_SetCompare(&PLATFORM_TIM, PLATFORM_Y_TIM_CHANNEL, (uint32_t)(PointCurrent.y));
				  break;
			  case 8:
				  PointCurrent.y += stepPWm;
				  __HAL_TIM_SetCompare(&PLATFORM_TIM, PLATFORM_Y_TIM_CHANNEL, (uint32_t)(PointCurrent.y));
				  break;
			  case 13:
				  PointCurrent.x += stepPWm;
				  __HAL_TIM_SetCompare(&PLATFORM_TIM, PLATFORM_X_TIM_CHANNEL, (uint32_t)(PointCurrent.x));
				  break;
			  case 14:
				  PointCurrent.x -= stepPWm;
				  __HAL_TIM_SetCompare(&PLATFORM_TIM, PLATFORM_X_TIM_CHANNEL, (uint32_t)(PointCurrent.x));
				  break;
			  default:
				  break;
			}
			continuousKeyNum = 0;
		}

		// 单次按键
		if (singleKeyNum != 0) {
			switch (singleKeyNum) {
			  case 2:
				  PointCurrent.y -= stepPWm;
				  __HAL_TIM_SetCompare(&PLATFORM_TIM, PLATFORM_Y_TIM_CHANNEL, (uint32_t)(PointCurrent.y));
				  break;
			  case 5:
				  PointCurrent.x += stepPWm;
				  __HAL_TIM_SetCompare(&PLATFORM_TIM, PLATFORM_X_TIM_CHANNEL, (uint32_t)(PointCurrent.x));
				  break;
			  case 7:
				  PointCurrent.x -= stepPWm;
				  __HAL_TIM_SetCompare(&PLATFORM_TIM, PLATFORM_X_TIM_CHANNEL, (uint32_t)(PointCurrent.x));
				  break;
			  case 10:
				  PointCurrent.y += stepPWm;
				  __HAL_TIM_SetCompare(&PLATFORM_TIM, PLATFORM_Y_TIM_CHANNEL, (uint32_t)(PointCurrent.y));
				  break;
			  case 16:
				  // 恢复位置到原点
//				  PointCurrent.x = PencilFrame.center.x;
//				  PointCurrent.y = PencilFrame.center.y;
//				  __HAL_TIM_SetCompare(&PLATFORM_TIM, PLATFORM_X_TIM_CHANNEL, (uint32_t)(PointCurrent.x));
//				  __HAL_TIM_SetCompare(&PLATFORM_TIM, PLATFORM_Y_TIM_CHANNEL, (uint32_t)(PointCurrent.y));

				  isWriteAllFlashDoneFlag = 1;
				  ConfirmKeyCnt = 5;
				  break;
			  case 6:
				  switch (ConfirmKeyCnt) {
				  	  case 0:
				  		  PencilFrame.center.x = PointCurrent.x; PencilFrame.center.y = PointCurrent.y;
				  	  	  break;
					  case 1:
						  PencilFrame.upperLeft.x = PointCurrent.x; PencilFrame.upperLeft.y = PointCurrent.y;

						  // 移动到下一位置 upperRight
						  PointCurrent.x = PencilFrame.center.x + (PencilFrame.center.x - PencilFrame.upperLeft.x);
						  __HAL_TIM_SetCompare(&PLATFORM_TIM, PLATFORM_X_TIM_CHANNEL, (uint32_t)(PointCurrent.x));
						  __HAL_TIM_SetCompare(&PLATFORM_TIM, PLATFORM_Y_TIM_CHANNEL, (uint32_t)(PointCurrent.y));
						  break;
					  case 2:
						  PencilFrame.upperRight.x = PointCurrent.x; PencilFrame.upperRight.y = PointCurrent.y;

						  PointCurrent.y = PencilFrame.center.y + (PencilFrame.center.y - PencilFrame.upperRight.y);
						  __HAL_TIM_SetCompare(&PLATFORM_TIM, PLATFORM_X_TIM_CHANNEL, (uint32_t)(PointCurrent.x));
						  __HAL_TIM_SetCompare(&PLATFORM_TIM, PLATFORM_Y_TIM_CHANNEL, (uint32_t)(PointCurrent.y));
						  break;
					  case 3:
						  PencilFrame.lowerRight.x = PointCurrent.x; PencilFrame.lowerRight.y = PointCurrent.y;

						  PointCurrent.x = PencilFrame.upperLeft.x;
						  __HAL_TIM_SetCompare(&PLATFORM_TIM, PLATFORM_X_TIM_CHANNEL, (uint32_t)(PointCurrent.x));
						  __HAL_TIM_SetCompare(&PLATFORM_TIM, PLATFORM_Y_TIM_CHANNEL, (uint32_t)(PointCurrent.y));
						  break;
					  case 4:
						  PencilFrame.lowerLeft.x = PointCurrent.x; PencilFrame.lowerLeft.y = PointCurrent.y;
						  break;
					  default:
						  break;
				  }
				  ConfirmKeyCnt ++ ;
				  HAL_GPIO_TogglePin(LED1G_GPIO_Port, LED1G_Pin);
				  break;
			  default:
				  break;
			}
			singleKeyNum = 0;
		}

		// 执行扫描铅笔线段
		if (ConfirmKeyCnt == 5) {
			switch (pointTargetState) {
				case 0:
					RedPointTarget.x = PencilFrame.upperRight.x;
					RedPointTarget.y = PencilFrame.upperRight.y;
					break;
				case 1:
					RedPointTarget.x = PencilFrame.lowerRight.x;
					RedPointTarget.y = PencilFrame.lowerRight.y;
					break;
				case 2:
					RedPointTarget.x = PencilFrame.lowerLeft.x;
					RedPointTarget.y = PencilFrame.lowerLeft.y;
					break;
				case 3:
					RedPointTarget.x = PencilFrame.upperLeft.x;
					RedPointTarget.y = PencilFrame.upperLeft.y;
					break;
				case 4:
					// 回到中点
					RedPointTarget.x = PencilFrame.center.x;
					RedPointTarget.y = PencilFrame.center.y;
					PointSet.x = PencilFrame.center.x;
					PointSet.y = PencilFrame.center.y;
					break;
				default :
					break;
			}

			if ((uint16_t)(fabsf(PointSet.x - RedPointTarget.x)) <= 1 && (uint16_t)(fabsf(PointSet.y - RedPointTarget.y)) < 1e-5) {
				PointCurrent.x = RedPointTarget.x;
				PointCurrent.y = RedPointTarget.y;
				divisionsNumCnts = 0;
				pointTargetState ++ ;
			}

			// 在左上角短暂等待后再开始扫线
			if (waitStartInitTimeCnt <= waitStartInitTimeNum && pointTargetState == 0) {
				PointCurrent.x = PencilFrame.upperLeft.x;
				PointCurrent.y = PencilFrame.upperLeft.y;
				__HAL_TIM_SetCompare(&PLATFORM_TIM, PLATFORM_X_TIM_CHANNEL, (uint32_t)(PointCurrent.x));
				__HAL_TIM_SetCompare(&PLATFORM_TIM, PLATFORM_Y_TIM_CHANNEL, (uint32_t)(PointCurrent.y));
				waitStartInitTimeCnt ++ ;
			}

			if (!isWriteAllFlashDoneFlag) {
				isWriteAllFlashDoneFlag = 1;

				flashAddr = 0;
				writeAllPointFlash();
			}

			// 扫线执行
			if (!isKey0PressedStopFlag && !isKey1PressedResetFlag && ConfirmKeyCnt == 5 && waitStartInitTimeCnt > waitStartInitTimeNum) {
				divisionsNumCnts ++ ;
				float equidistant = divisionsNumCnts * 1.0f / divisionsPencilNum;
				PointSet.x = PointCurrent.x + (RedPointTarget.x - PointCurrent.x) * equidistant;
				PointSet.y = PointCurrent.y + (RedPointTarget.y - PointCurrent.y) * equidistant;

				__HAL_TIM_SetCompare(&PLATFORM_TIM, PLATFORM_X_TIM_CHANNEL, (uint32_t)(PointSet.x));
				__HAL_TIM_SetCompare(&PLATFORM_TIM, PLATFORM_Y_TIM_CHANNEL, (uint32_t)(PointSet.y));

				if (pointTargetState == 5) {
					pointTargetState ++ ;
					ConfirmKeyCnt ++ ;
					isPencilScanDoneFlag = 1;
				}
			}

			if (isKey1PressedResetFlag) {
				__HAL_TIM_SetCompare(&PLATFORM_TIM, PLATFORM_X_TIM_CHANNEL, (uint32_t)(PencilFrame.center.x));
				__HAL_TIM_SetCompare(&PLATFORM_TIM, PLATFORM_Y_TIM_CHANNEL, (uint32_t)(PencilFrame.center.y));
			}
		}

		vTaskDelay(pdMS_TO_TICKS(55));
	}
}

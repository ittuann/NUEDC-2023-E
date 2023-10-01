/*
 * platformTask.c
 *
 *  Created on: Aug 2, 2023
 *      Author: LBQ
 */

#include <stdio.h>
#include <math.h>
#include "main.h"
#include "time.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "platformTask.h"
#include "PID.h"

extern TIM_HandleTypeDef htim1;
extern PIDTypeDef_t PID_Platform[PlatformPID_NUM];
extern bool_t isPencilScanDoneFlag;
extern struct Square_t PencilFrame;

uint16_t platformPWMInitVal = 375;				// PWM初始值
uint16_t platformPWMInitLimitVal = 60;			// PWM边界值
uint8_t redPointThreshold = 3;					// 红点和长方形角点的阈值
uint16_t divisionsLargestNum = 8;				// 当前红点距离期望细分点过远的判断极大值

// 按键中断相关
static volatile bool_t isKey0PressedExitFlag;
static volatile uint32_t key0TimeStamp;
static volatile bool_t isKey1PressedExitFlag;
static volatile uint32_t Key1TimeStamp;
volatile bool_t isKey0PressedStopFlag = 0;		// 按键暂停标志位
volatile bool_t isKey1PressedResetFlag;
static bool_t isStopFlag;

struct Square_t ElectricalTape;					// A4纸电工胶带方形框
struct Point_t RedPointNow;						// 当前位置
static struct Point_t RedPointEx;				// PID期望位置
static struct Point_t RedPointAim;				// 目标为方形的哪个角
static struct Point_t RedPointAimLast;
struct Point_t PointLimitMax;					// 限幅
struct Point_t PointLimitMin;

static uint8_t pointAimState;
static bool_t isPointAimState0InitFlag;
static const uint16_t platformWaitCameraInit = 1000;	// 等待摄像头初始化时间
static uint16_t platformWaitCameraCnt;

volatile bool_t rxDataFlag;						// 收到通讯数值标志位
volatile bool_t rxDataTrueFlag;					// 收到正确的通讯数值标志位
static const uint32_t divisionsShortNum = 3500;	// 线性插值 细分数 80
const uint32_t divisionsLongNum = divisionsShortNum * 30 / 21;
static uint32_t divisionsNum;
static uint32_t divisionsNumCnt;

/**
  * @brief		reDirect Printf to SWV Console
  */
int _write(int file, char *ptr, int len)
{
    for (int dataIdx = 0; dataIdx < len; dataIdx ++ ) {
        ITM_SendChar(*ptr++);
    }

    return len;
}

/**
  * @brief		云台PWM初始化
  */
void Platform_PWM_Init(uint32_t initVal)
{
	HAL_TIM_Base_Start(&PLATFORM_TIM);										// 开启定时器

	HAL_TIM_PWM_Start(&PLATFORM_TIM, PLATFORM_X_TIM_CHANNEL);				// 使对应定时器的对应通道开始PWM输出
	HAL_TIM_PWM_Start(&PLATFORM_TIM, PLATFORM_Y_TIM_CHANNEL);

	__HAL_TIM_SetCompare(&PLATFORM_TIM, PLATFORM_X_TIM_CHANNEL, initVal);	// PE9
	__HAL_TIM_SetCompare(&PLATFORM_TIM, PLATFORM_Y_TIM_CHANNEL, initVal);	// PE11
}

/**
  * @brief  LED Toggle
  * @notice	用于判断任务是否正常运行
  */
static void LEDToggle(void)
{
	static uint16_t LEDCount = 0;

	if (LEDCount >= 100) {
		HAL_GPIO_TogglePin(LED0R_GPIO_Port, LED0R_Pin);
		LEDCount = 0;
	} else {
		LEDCount ++ ;
	}
}

/**
  * @brief  线性插值函数
  */
struct Point_t linearInterpolation(struct Point_t startPoint, struct Point_t endPoint, float t) {
	struct Point_t interpolatedPoint;
    interpolatedPoint.x = startPoint.x + (endPoint.x - startPoint.x) * t;
    interpolatedPoint.y = startPoint.y + (endPoint.y - startPoint.y) * t;
    return interpolatedPoint;
}


/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == KEY0_Pin) {
    	isKey0PressedExitFlag = 1;
    	key0TimeStamp = HAL_GetTick();
    }
    if (GPIO_Pin == KEY1_Pin) {
    	isKey1PressedExitFlag = 1;
    	Key1TimeStamp = HAL_GetTick();
    }
}


/**
  * @brief  Function implementing the basicTask thread.
  * @param  argument: Not used
  * @retval None
  */
void PlatformTask(void const * argument)
{
	static portTickType xLastWakeTime;
	const portTickType xTimeIncrement = pdMS_TO_TICKS(2UL);	// 绝对延时

	// 用当前tick时间初始化 pxPreviousWakeTime
	xLastWakeTime = xTaskGetTickCount();

	PID_Init();

	HAL_GPIO_WritePin(LED0R_GPIO_Port, LED0R_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED1G_GPIO_Port, LED1G_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SWITCH_GPIO_Port, SWITCH_Pin, GPIO_PIN_SET);

	PointLimitMax.x = PointLimitMax.y = platformPWMInitVal + platformPWMInitLimitVal;
	PointLimitMin.x = PointLimitMin.y = platformPWMInitVal - platformPWMInitLimitVal;
	divisionsNum = divisionsShortNum;

	PID_Platform[PlatformPID_X].Output = __HAL_TIM_GetCompare(&PLATFORM_TIM, PLATFORM_X_TIM_CHANNEL);
	PID_Platform[PlatformPID_Y].Output = __HAL_TIM_GetCompare(&PLATFORM_TIM, PLATFORM_Y_TIM_CHANNEL);

	while(1)
	{
		// 任务绝对延时
		vTaskDelayUntil(&xLastWakeTime, xTimeIncrement);

		// 按键消抖检测
		if (isKey0PressedExitFlag) {
	        if ((HAL_GetTick() - key0TimeStamp) > 50) {
	        	isKey0PressedExitFlag = 0;
	            if (HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) == GPIO_PIN_RESET) {
	                // 按键已稳定
	            	if (isKey0PressedStopFlag) {
	            		isKey0PressedStopFlag = 0;
	            	} else {
	            		isKey0PressedStopFlag = 1;
	            	}
	            }
	        }
		}
		if (isKey1PressedExitFlag) {
	        if ((HAL_GetTick() - Key1TimeStamp) > 50) {
	        	isKey1PressedExitFlag = 0;
	            if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET) {
	            	if (isKey1PressedResetFlag) {
	            		isKey1PressedResetFlag = 0;
	            	} else {
	            		isKey1PressedResetFlag = 1;
	            	}
	            }
	        }
		}

		/****************************** 逻辑 ******************************/
		// 如果当前位置和目标角位很近, 则更换目标点为下一个
		if ((uint16_t)(fabsf(RedPointAim.x - RedPointNow.x)) <= redPointThreshold && (uint16_t)(fabsf(RedPointAim.y - RedPointNow.y)) <= redPointThreshold
				&& RedPointAim.x != 0.0f && !isStopFlag) {
			divisionsNumCnt = 0;
			pointAimState ++ ;
		}

		// 判断和更新当前目标点是四个角的哪一个点
		switch (pointAimState) {
			case 0:
				if (!isPointAimState0InitFlag && rxDataTrueFlag) {
					RedPointAimLast.x = RedPointNow.x;
					RedPointAimLast.y = RedPointNow.y;
					isPointAimState0InitFlag = 1;
				}
				RedPointAim.x = ElectricalTape.upperLeft.x;
				RedPointAim.y = ElectricalTape.upperLeft.y;
				break;
			case 1:
				RedPointAimLast.x = ElectricalTape.upperLeft.x;
				RedPointAimLast.y = ElectricalTape.upperLeft.y;
				RedPointAim.x = ElectricalTape.upperRight.x;
				RedPointAim.y = ElectricalTape.upperRight.y;
				break;
			case 2:
				RedPointAimLast.x = ElectricalTape.upperRight.x;
				RedPointAimLast.y = ElectricalTape.upperRight.y;
				RedPointAim.x = ElectricalTape.lowerRight.x;
				RedPointAim.y = ElectricalTape.lowerRight.y;
				break;
			case 3:
				RedPointAimLast.x = ElectricalTape.lowerRight.x;
				RedPointAimLast.y = ElectricalTape.lowerRight.y;
				RedPointAim.x = ElectricalTape.lowerLeft.x;
				RedPointAim.y = ElectricalTape.lowerLeft.y;
				break;
			case 4:
				RedPointAimLast.x = ElectricalTape.lowerLeft.x;
				RedPointAimLast.y = ElectricalTape.lowerLeft.y;
				RedPointAim.x = ElectricalTape.upperLeft.x;
				RedPointAim.y = ElectricalTape.upperLeft.y;
				break;
			default :
//				stopFlag = 1;
				pointAimState = 0;
				HAL_GPIO_WritePin(SWITCH_GPIO_Port, SWITCH_Pin, GPIO_PIN_RESET);
				break;
		}

		// 等待摄像头初始化和识别
		if (platformWaitCameraCnt <= platformWaitCameraInit && isPencilScanDoneFlag) {
			platformWaitCameraCnt ++ ;
		}
		if (isKey0PressedStopFlag) {
			platformWaitCameraCnt = platformWaitCameraInit;
		}

		// PID计算
//		if (rxDataFlag && rxDataTrueFlag && platformWaitCameraCnt > platformWaitCameraInit && !isStopFlag && !isKey0PressedStopFlag && !isKey1PressedResetFlag && isPencilScanDoneFlag) {
//			rxDataTrueFlag = 0;
		if (rxDataFlag && platformWaitCameraCnt > platformWaitCameraInit && !isStopFlag && !isKey0PressedStopFlag && !isKey1PressedResetFlag && isPencilScanDoneFlag) {

			// 线性插值. 计算期望坐标,即下一个要移动的位置
			if (divisionsNumCnt < divisionsNum) {
//			if (divisionsNumCnt < divisionsNum && (uint16_t)(fabsf(RedPointEx.x - RedPointNow.x) <= redPointThreshold && (uint16_t)(fabsf(RedPointEx.y - RedPointNow.y)) <= redPointThreshold)) {
				divisionsNumCnt ++ ;
			}
			float equidistant = divisionsNumCnt * 1.0f / divisionsNum;
			RedPointEx.x = RedPointAimLast.x + (RedPointAim.x - RedPointAimLast.x) * equidistant;
			RedPointEx.y = RedPointAimLast.y + (RedPointAim.y - RedPointAimLast.y) * equidistant;

			// 特判 如果当前红点距离期望细分点过远 则更新期望细分点为当前点位置的下一个细分点
			if (fabsf(RedPointEx.x - RedPointNow.x) >= divisionsLargestNum) {
				while (fabsf(RedPointEx.x - RedPointNow.x) >= 1 && divisionsNumCnt < divisionsNum) {
					if (divisionsNumCnt < divisionsNum) {
						divisionsNumCnt ++ ;
					}
					equidistant = divisionsNumCnt * 1.0f / divisionsNum;
					RedPointEx.x = RedPointAimLast.x + (RedPointAim.x - RedPointAimLast.x) * equidistant;
					RedPointEx.y = RedPointAimLast.y + (RedPointAim.y - RedPointAimLast.y) * equidistant;
				}
			}

			// 运算
			PID_Calc(&PID_Platform[PlatformPID_X], RedPointNow.x, RedPointEx.x);
			PID_Calc(&PID_Platform[PlatformPID_Y], RedPointNow.y, RedPointEx.y);

			// PID输出
			// x轴左+右- y轴上-下+
			__HAL_TIM_SetCompare(&PLATFORM_TIM, PLATFORM_X_TIM_CHANNEL, (uint32_t)(PID_Platform[PlatformPID_X].Output));
			__HAL_TIM_SetCompare(&PLATFORM_TIM, PLATFORM_Y_TIM_CHANNEL, (uint32_t)(PID_Platform[PlatformPID_Y].Output));

		}

		// 按下KEY1回到中值
		if (isKey1PressedResetFlag && isPencilScanDoneFlag) {
			__HAL_TIM_SetCompare(&PLATFORM_TIM, PLATFORM_X_TIM_CHANNEL, (uint32_t)(PencilFrame.center.x));
			__HAL_TIM_SetCompare(&PLATFORM_TIM, PLATFORM_Y_TIM_CHANNEL, (uint32_t)(PencilFrame.center.y));
		}

		// LED翻转电平闪烁红灯 提示线程正在运行
		LEDToggle();
	}

}


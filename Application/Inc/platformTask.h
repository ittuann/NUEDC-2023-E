/*
 * platformTask.h
 *
 *  Created on: Aug 2, 2023
 *      Author: LBQ
 */

#pragma once

#define PLATFORM_TIM				htim1
#define PLATFORM_X_TIM_CHANNEL		TIM_CHANNEL_1
#define PLATFORM_Y_TIM_CHANNEL		TIM_CHANNEL_2

struct Point_t {
    float x;
    float y;
};

struct Square_t {
    struct Point_t upperLeft;
    struct Point_t upperRight;
    struct Point_t lowerRight;
    struct Point_t lowerLeft;
    struct Point_t center;
};

extern uint16_t platformPWMInitVal;
extern uint16_t platformPWMInitLimitVal;
extern struct Point_t PointLimitMax;
extern struct Point_t PointLimitMin;

extern volatile bool_t key0_pressedStopFlag;
extern volatile bool_t key1_pressedResetFlag;

extern struct Point_t RedPointNow;
extern volatile bool_t rxDataFlag;
extern volatile bool_t rxDataTrueFlag;

extern void Platform_PWM_Init(uint32_t initVal);


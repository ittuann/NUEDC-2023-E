/*
 * PID.h
 *
 *  Created on: 2021年10月24日
 *      Author: LBQ
 */
#pragma once

typedef struct
{
    float   Now_Val;            // 当前值
	float   Last_Val;			// 上次值
	float   EX_Last;			// 上次期望值
    float   EX_Val;             // 期望值

    float   Err_Now;            // 当前误差
    float   Err_ABS;            // 当前误差绝对值
    float   Err_Last;           // 上次误差
    float   Err_LastLast;       // 上上次误差
    float   Err_Sum;            // 误差累积

    float   Kp;                 // 比例系数
    float   Ki;                 // 积分系数
    float   Kd;                 // 微分系数

	float   p_Max;              // 比例项输出限幅
    float   i_Max;              // 积分项限幅
	float   d_Max;              // 微分项输出限幅
	float   errSum_Max;			// 累积误差限幅
	float   out_Max;            // 输出限幅
	
    float   Output_p;           // 比例项输出
    float   Output_i;           // 积分项输出
    float   Output_d;           // 微分项输出
    float   Output_dd;          // 上次微分项输出
    float   Output_Last;        // 上次输出值
	float   Output;             // 输出
} PIDTypeDef_t;

enum PlatformPID_ID_e {
	PlatformPID_X = 0,
	PlatformPID_Y,
	PlatformPID_NUM
};

typedef struct {
	PIDTypeDef_t *BaseStructInstance;	// 定义PID基类指针
	float ILErr;						// 积分项误差死区范围上限
	float ISErr;						// 积分项误差死区范围下限
	float outPutLimit;					// 输出项死区
	float onlineK;						// 强作用比率
} PIDProfessTypeDef_t;					// 专家PID规则设置

extern PIDTypeDef_t PID_Platform[PlatformPID_NUM];

extern void PID_Init(void);
extern void PID_Calc_Original(PIDTypeDef_t *pid, float ref, float set);
extern void PID_Calc(PIDTypeDef_t *pid, float ref, float set);
extern void PID_Profess(PIDTypeDef_t *pid, PIDProfessTypeDef_t *pp, float ref, float set);
extern void PID_Clear(PIDTypeDef_t *pid);


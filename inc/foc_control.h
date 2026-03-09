#ifndef __FOC_CONTROL_H
#define __FOC_CONTROL_H

#include "stm32f10x.h"
#include <stdint.h>

/* ============================= 用户可调参数 ============================= */
#define POLE_PAIRS                 (7.0f)      /* 电机极对数 */
#define SHUNT_RESISTANCE           (0.005f)    /* 采样电阻，单位：欧姆 */
#define CURRENT_AMP_GAIN           (20.0f)     /* 电流采样运放倍数 */
#define ADC_REF_VOLTAGE            (3.3f)      /* ADC参考电压 */
#define ADC_FULL_SCALE             (4095.0f)   /* 12位ADC满量程 */

#define PWM_FREQUENCY_HZ           (20000U)    /* PWM频率：20kHz */
#define CONTROL_FREQUENCY_HZ       (10000.0f)  /* 控制环频率：10kHz */

#define VBUS_DEFAULT               (24.0f)     /* 默认母线电压，可由ADC实时更新 */
#define MAX_MODULATION             (0.577f)    /* SVPWM线性调制极限 */

/* 速度/位置单位约定：
 * - 位置环输入与输出：机械角度（rad）
 * - 速度环输入与输出：机械角速度（rad/s）
 */

/* ============================= 数据结构定义 ============================= */
typedef struct
{
    float kp;
    float ki;
    float integral;
    float output_max;
    float output_min;
} PIController_t;

typedef struct
{
    /* 采样量（A） */
    float ia;
    float ib;
    float ic;

    /* dq轴电流（A） */
    float id;
    float iq;

    /* 指令值 */
    float id_ref;
    float iq_ref;
    float speed_ref;      /* rad/s */
    float position_ref;   /* rad */

    /* 状态量 */
    float mech_angle;     /* 机械角度(rad) */
    float elec_angle;     /* 电角度(rad) */
    float mech_speed;     /* 机械角速度(rad/s) */

    /* dq轴电压输出（V） */
    float vd;
    float vq;

    float vbus;           /* 母线电压（V） */
} FOCState_t;

extern FOCState_t g_foc_state;

/* ============================= 接口函数 ============================= */
void FOC_Init(void);
void FOC_ControlLoop_ISR(void);

/* 传感器接口：
 * MA732：SPI读取绝对角度
 * 磁栅位置传感器：TIM编码器接口读取位置
 */
float MA732_ReadMechanicalAngleRad(void);
float MagScale_ReadMechanicalAngleRad(void);
float MagScale_ReadMechanicalSpeedRadPerSec(void);

#endif /* __FOC_CONTROL_H */

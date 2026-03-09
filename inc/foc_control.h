#ifndef __FOC_CONTROL_H
#define __FOC_CONTROL_H

#include "stm32f10x.h"
#include <stdint.h>

/* ============================= 用户可调参数 ============================= */
#define POLE_PAIRS                    (7.0f)      /* 电机极对数 */
#define SHUNT_RESISTANCE              (0.005f)    /* 相电流采样电阻(ohm) */
#define CURRENT_AMP_GAIN              (20.0f)     /* 电流采样运放增益 */
#define ADC_REF_VOLTAGE               (3.3f)
#define ADC_FULL_SCALE                (4095.0f)

#define PWM_FREQUENCY_HZ              (20000U)
#define CONTROL_FREQUENCY_HZ          (20000.0f)  /* 与PWM同步，20kHz */

#define VBUS_DEFAULT                  (24.0f)
#define MAX_MODULATION                (0.577f)    /* SVPWM线性区 */

/* 三相电流ADC通道（示例：PA0/PA1/PA2） */
#define ADC_CHANNEL_IA                ADC_Channel_0
#define ADC_CHANNEL_IB                ADC_Channel_1
#define ADC_CHANNEL_IC                ADC_Channel_2

/* 磁栅SPI参数（仅位置） */
#define MAGSCALE_SPI_RESOLUTION       (16384.0f)  /* 假设14bit */

/* MA732参数（角度+速度） */
#define MA732_RESOLUTION              (16384.0f)  /* 14bit */

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
    /* 三相电流(A) */
    float ia;
    float ib;
    float ic;

    /* dq电流(A) */
    float id;
    float iq;

    /* 参考量 */
    float id_ref;
    float iq_ref;
    float speed_ref;      /* rad/s */
    float position_ref;   /* rad */

    /* 状态量 */
    float mech_angle;     /* 机械角度(rad)：由磁栅SPI读取 */
    float mech_speed;     /* 机械角速度(rad/s)：由MA732计算 */
    float elec_angle;     /* 电角度(rad)：由MA732+零电角偏移计算 */

    /* 控制输出 */
    float vd;
    float vq;

    float vbus;
    float elec_zero_offset; /* 找电角度得到的零偏 */
} FOCState_t;

extern FOCState_t g_foc_state;

/* ============================= 对外接口 ============================= */
void FOC_Init(void);
void FOC_RunElecAngleCalibration(void);
void FOC_ControlLoop_ISR(void);

/* 传感器接口 */
float MA732_ReadMechanicalAngleRad(void);
float MA732_ReadMechanicalSpeedRadPerSec(void);
float MagScaleSPI_ReadMechanicalPositionRad(void);

#endif

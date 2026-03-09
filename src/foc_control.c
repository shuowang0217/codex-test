#include "foc_control.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define TWO_PI (2.0f * M_PI)

/* 磁栅传感器参数：
 * 假设使用AB相增量编码器，TIM编码器模式下4倍频计数。
 */
#define MAGSCALE_CPR             (4096.0f)    /* 每机械圈计数（4倍频后） */

/* MA732参数：14bit绝对角度 */
#define MA732_RESOLUTION         (16384.0f)

/* 采样周期 */
#define TS_CURRENT_LOOP          (1.0f / CONTROL_FREQUENCY_HZ)
#define TS_SPEED_LOOP            (0.001f)      /* 1kHz */
#define TS_POSITION_LOOP         (0.005f)      /* 200Hz */

/* PWM定时器周期（72MHz主频，中心对齐模式频率 = timer_clk/(2*ARR)） */
#define TIM1_ARR_VALUE           (1800U)

FOCState_t g_foc_state;

static PIController_t s_id_pi;
static PIController_t s_iq_pi;
static PIController_t s_speed_pi;
static PIController_t s_pos_pi;

/* 环路分频计数器 */
static uint16_t s_speed_div = 0;
static uint16_t s_pos_div = 0;

/* ============================= 基础数学工具 ============================= */
static float wrap_angle(float x)
{
    while (x >= TWO_PI)
    {
        x -= TWO_PI;
    }
    while (x < 0.0f)
    {
        x += TWO_PI;
    }
    return x;
}

static float clamp(float in, float minv, float maxv)
{
    if (in > maxv)
    {
        return maxv;
    }
    if (in < minv)
    {
        return minv;
    }
    return in;
}

static float pi_run(PIController_t *pi, float ref, float fb, float ts)
{
    float err = ref - fb;
    float p_out = pi->kp * err;

    pi->integral += pi->ki * err * ts;
    pi->integral = clamp(pi->integral, pi->output_min, pi->output_max);

    return clamp(p_out + pi->integral, pi->output_min, pi->output_max);
}

/* Clarke变换：将三相电流变到αβ坐标系
 * 假定 ia + ib + ic = 0
 */
static void clarke_transform(float ia, float ib, float *i_alpha, float *i_beta)
{
    *i_alpha = ia;
    *i_beta = (ia + 2.0f * ib) * 0.57735026919f; /* 1/sqrt(3) */
}

/* Park变换：αβ -> dq */
static void park_transform(float i_alpha, float i_beta, float theta, float *id, float *iq)
{
    float sin_t = sinf(theta);
    float cos_t = cosf(theta);

    *id = i_alpha * cos_t + i_beta * sin_t;
    *iq = -i_alpha * sin_t + i_beta * cos_t;
}

/* 逆Park变换：dq -> αβ */
static void inv_park_transform(float vd, float vq, float theta, float *v_alpha, float *v_beta)
{
    float sin_t = sinf(theta);
    float cos_t = cosf(theta);

    *v_alpha = vd * cos_t - vq * sin_t;
    *v_beta = vd * sin_t + vq * cos_t;
}

/* SVPWM：输入αβ电压，输出三相占空比（0~1）
 * 这里采用零序注入法（min-max法）简化实现，便于理解与移植。
 */
static void svm_generate(float v_alpha, float v_beta, float vbus, float *duty_a, float *duty_b, float *duty_c)
{
    float va, vb, vc;
    float v_max, v_min;
    float v_offset;

    va = v_alpha;
    vb = -0.5f * v_alpha + 0.8660254f * v_beta;
    vc = -0.5f * v_alpha - 0.8660254f * v_beta;

    /* 零序电压注入，最大化母线利用率 */
    v_max = fmaxf(va, fmaxf(vb, vc));
    v_min = fminf(va, fminf(vb, vc));
    v_offset = 0.5f * (v_max + v_min);

    va -= v_offset;
    vb -= v_offset;
    vc -= v_offset;

    /* 归一化成占空比，中心对齐 */
    *duty_a = 0.5f + va / vbus;
    *duty_b = 0.5f + vb / vbus;
    *duty_c = 0.5f + vc / vbus;

    *duty_a = clamp(*duty_a, 0.0f, 1.0f);
    *duty_b = clamp(*duty_b, 0.0f, 1.0f);
    *duty_c = clamp(*duty_c, 0.0f, 1.0f);
}

/* ============================= 硬件相关函数 ============================= */
static void hw_gpio_spi_tim_adc_pwm_init(void)
{
    /*
     * 这里仅给出标准外设库配置骨架。
     * 具体IO口可按硬件原理图修改。
     */
    GPIO_InitTypeDef gpio;
    SPI_InitTypeDef spi;
    TIM_TimeBaseInitTypeDef tim;
    TIM_OCInitTypeDef oc;
    TIM_BDTRInitTypeDef bdtr;
    ADC_InitTypeDef adc;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_AFIO | RCC_APB2Periph_SPI1 |
                           RCC_APB2Periph_TIM1 | RCC_APB2Periph_ADC1,
                           ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* SPI1: PA5 SCK, PA6 MISO, PA7 MOSI */
    gpio.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_6;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    /* MA732片选：PA4 */
    gpio.GPIO_Pin = GPIO_Pin_4;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &gpio);
    GPIO_SetBits(GPIOA, GPIO_Pin_4);

    SPI_StructInit(&spi);
    spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_Mode = SPI_Mode_Master;
    spi.SPI_DataSize = SPI_DataSize_16b;
    spi.SPI_CPOL = SPI_CPOL_Low;
    spi.SPI_CPHA = SPI_CPHA_2Edge;
    spi.SPI_NSS = SPI_NSS_Soft;
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
    spi.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_Init(SPI1, &spi);
    SPI_Cmd(SPI1, ENABLE);

    /* TIM3编码器接口：PB4/PB5 */
    gpio.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &gpio);

    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_SetAutoreload(TIM3, 0xFFFF);
    TIM_SetCounter(TIM3, 0);
    TIM_Cmd(TIM3, ENABLE);

    /* TIM1三相PWM输出（CH1/2/3）：PA8/PA9/PA10 */
    tim.TIM_Prescaler = 0;
    tim.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
    tim.TIM_Period = TIM1_ARR_VALUE - 1;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &tim);

    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputNState_Enable;
    oc.TIM_Pulse = TIM1_ARR_VALUE / 2;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    oc.TIM_OCNPolarity = TIM_OCNPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCIdleState_Reset;
    TIM_OC1Init(TIM1, &oc);
    TIM_OC2Init(TIM1, &oc);
    TIM_OC3Init(TIM1, &oc);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

    bdtr.TIM_OSSRState = TIM_OSSRState_Enable;
    bdtr.TIM_OSSIState = TIM_OSSIState_Enable;
    bdtr.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
    bdtr.TIM_DeadTime = 72; /* 约1us死区，需按驱动器调整 */
    bdtr.TIM_Break = TIM_Break_Disable;
    bdtr.TIM_BreakPolarity = TIM_BreakPolarity_High;
    bdtr.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
    TIM_BDTRConfig(TIM1, &bdtr);

    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);

    /* ADC1：示例配置，可用于采样Ia、Ib、母线电压 */
    ADC_StructInit(&adc);
    adc.ADC_Mode = ADC_Mode_Independent;
    adc.ADC_ScanConvMode = ENABLE;
    adc.ADC_ContinuousConvMode = DISABLE;
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel = 3;
    ADC_Init(ADC1, &adc);

    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1))
    {
    }
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1))
    {
    }
}

/* 根据实际硬件替换：读取ADC并换算为相电流 */
static void sample_phase_currents(float *ia, float *ib, float *ic)
{
    uint16_t adc_ia = ADC_GetConversionValue(ADC1);
    uint16_t adc_ib = ADC_GetConversionValue(ADC1);

    /* 假定中点偏置1.65V */
    float v_ia = (adc_ia / ADC_FULL_SCALE) * ADC_REF_VOLTAGE - 1.65f;
    float v_ib = (adc_ib / ADC_FULL_SCALE) * ADC_REF_VOLTAGE - 1.65f;

    *ia = v_ia / (SHUNT_RESISTANCE * CURRENT_AMP_GAIN);
    *ib = v_ib / (SHUNT_RESISTANCE * CURRENT_AMP_GAIN);
    *ic = -(*ia + *ib);
}

static void pwm_set_duty(float duty_a, float duty_b, float duty_c)
{
    uint16_t ccr_a = (uint16_t)(duty_a * (float)(TIM1_ARR_VALUE - 1));
    uint16_t ccr_b = (uint16_t)(duty_b * (float)(TIM1_ARR_VALUE - 1));
    uint16_t ccr_c = (uint16_t)(duty_c * (float)(TIM1_ARR_VALUE - 1));

    TIM_SetCompare1(TIM1, ccr_a);
    TIM_SetCompare2(TIM1, ccr_b);
    TIM_SetCompare3(TIM1, ccr_c);
}

/* ============================= 传感器读取 ============================= */
float MA732_ReadMechanicalAngleRad(void)
{
    uint16_t raw;

    GPIO_ResetBits(GPIOA, GPIO_Pin_4);
    SPI_I2S_SendData(SPI1, 0x0000);
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
    {
    }
    raw = SPI_I2S_ReceiveData(SPI1) & 0x3FFF;
    GPIO_SetBits(GPIOA, GPIO_Pin_4);

    return ((float)raw / MA732_RESOLUTION) * TWO_PI;
}

float MagScale_ReadMechanicalAngleRad(void)
{
    int16_t cnt = (int16_t)TIM_GetCounter(TIM3);
    float mech_turn = (float)cnt / MAGSCALE_CPR;
    float angle = mech_turn * TWO_PI;

    return wrap_angle(angle);
}

float MagScale_ReadMechanicalSpeedRadPerSec(void)
{
    static int16_t last_cnt = 0;
    int16_t now_cnt = (int16_t)TIM_GetCounter(TIM3);
    int16_t diff = now_cnt - last_cnt;
    last_cnt = now_cnt;

    return ((float)diff / MAGSCALE_CPR) * TWO_PI / TS_SPEED_LOOP;
}

/* ============================= FOC核心 ============================= */
void FOC_Init(void)
{
    hw_gpio_spi_tim_adc_pwm_init();

    g_foc_state.vbus = VBUS_DEFAULT;
    g_foc_state.id_ref = 0.0f;
    g_foc_state.position_ref = 0.0f;

    /* 电流环PI（内环） */
    s_id_pi.kp = 1.2f;
    s_id_pi.ki = 300.0f;
    s_id_pi.integral = 0.0f;
    s_id_pi.output_min = -12.0f;
    s_id_pi.output_max = 12.0f;

    s_iq_pi = s_id_pi;

    /* 速度环PI（中环）：输出iq_ref */
    s_speed_pi.kp = 0.05f;
    s_speed_pi.ki = 3.0f;
    s_speed_pi.integral = 0.0f;
    s_speed_pi.output_min = -10.0f;
    s_speed_pi.output_max = 10.0f;

    /* 位置环PI（外环）：输出speed_ref */
    s_pos_pi.kp = 6.0f;
    s_pos_pi.ki = 1.0f;
    s_pos_pi.integral = 0.0f;
    s_pos_pi.output_min = -80.0f;
    s_pos_pi.output_max = 80.0f;
}

/* 该函数应在PWM中断或固定周期定时器中断中调用（10kHz） */
void FOC_ControlLoop_ISR(void)
{
    float i_alpha, i_beta;
    float v_alpha, v_beta;
    float duty_a, duty_b, duty_c;
    float v_limit;

    /* 1) 位置环（低频）: 磁栅位置 -> speed_ref */
    s_pos_div++;
    if (s_pos_div >= (uint16_t)(CONTROL_FREQUENCY_HZ * TS_POSITION_LOOP))
    {
        s_pos_div = 0;
        g_foc_state.mech_angle = MagScale_ReadMechanicalAngleRad();
        g_foc_state.speed_ref = pi_run(&s_pos_pi,
                                       g_foc_state.position_ref,
                                       g_foc_state.mech_angle,
                                       TS_POSITION_LOOP);
    }

    /* 2) 速度环（中频）: 磁栅速度 -> iq_ref */
    s_speed_div++;
    if (s_speed_div >= (uint16_t)(CONTROL_FREQUENCY_HZ * TS_SPEED_LOOP))
    {
        s_speed_div = 0;
        g_foc_state.mech_speed = MagScale_ReadMechanicalSpeedRadPerSec();
        g_foc_state.iq_ref = pi_run(&s_speed_pi,
                                    g_foc_state.speed_ref,
                                    g_foc_state.mech_speed,
                                    TS_SPEED_LOOP);
    }

    /* 3) 角度读取：MA732用于电角度（高速、绝对值） */
    g_foc_state.elec_angle = wrap_angle(MA732_ReadMechanicalAngleRad() * POLE_PAIRS);

    /* 4) 采样电流并做Clarke/Park变换 */
    sample_phase_currents(&g_foc_state.ia, &g_foc_state.ib, &g_foc_state.ic);
    clarke_transform(g_foc_state.ia, g_foc_state.ib, &i_alpha, &i_beta);
    park_transform(i_alpha, i_beta, g_foc_state.elec_angle, &g_foc_state.id, &g_foc_state.iq);

    /* 5) 电流环PI输出vd/vq */
    g_foc_state.vd = pi_run(&s_id_pi, g_foc_state.id_ref, g_foc_state.id, TS_CURRENT_LOOP);
    g_foc_state.vq = pi_run(&s_iq_pi, g_foc_state.iq_ref, g_foc_state.iq, TS_CURRENT_LOOP);

    /* 电压矢量限幅，防止超调制 */
    v_limit = g_foc_state.vbus * MAX_MODULATION;
    {
        float v_mag = sqrtf(g_foc_state.vd * g_foc_state.vd + g_foc_state.vq * g_foc_state.vq);
        if (v_mag > v_limit)
        {
            float scale = v_limit / v_mag;
            g_foc_state.vd *= scale;
            g_foc_state.vq *= scale;
        }
    }

    /* 6) 逆Park + SVPWM */
    inv_park_transform(g_foc_state.vd, g_foc_state.vq, g_foc_state.elec_angle, &v_alpha, &v_beta);
    svm_generate(v_alpha, v_beta, g_foc_state.vbus, &duty_a, &duty_b, &duty_c);
    pwm_set_duty(duty_a, duty_b, duty_c);
}

#include "foc_control.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define TWO_PI                        (2.0f * M_PI)
#define SQRT3                         (1.73205080757f)
#define INV_SQRT3                     (0.57735026919f)

#define TS_CURRENT_LOOP               (1.0f / CONTROL_FREQUENCY_HZ)
#define TS_SPEED_LOOP                 (0.001f)   /* 1kHz */
#define TS_POSITION_LOOP              (0.002f)   /* 500Hz */

#define TIM1_ARR_VALUE                (1800U)    /* 72MHz中心对齐 -> 20kHz */

/* MA732 SPI1片选 */
#define MA732_CS_PORT                 GPIOA
#define MA732_CS_PIN                  GPIO_Pin_4
/* 磁栅SPI2片选（示例） */
#define MAGSCALE_CS_PORT              GPIOB
#define MAGSCALE_CS_PIN               GPIO_Pin_12

FOCState_t g_foc_state;

static PIController_t s_id_pi;
static PIController_t s_iq_pi;
static PIController_t s_speed_pi;
static PIController_t s_pos_pi;

static uint16_t s_speed_div;
static uint16_t s_pos_div;

static uint16_t s_adc_raw_ia;
static uint16_t s_adc_raw_ib;
static uint16_t s_adc_raw_ic;
static float s_ia_offset = 1.65f;
static float s_ib_offset = 1.65f;
static float s_ic_offset = 1.65f;

/* ========================= 数学工具与坐标变换 ========================= */
static float clampf(float v, float min_v, float max_v)
{
    if (v > max_v)
    {
        return max_v;
    }
    if (v < min_v)
    {
        return min_v;
    }
    return v;
}

static float wrap_angle(float a)
{
    while (a >= TWO_PI)
    {
        a -= TWO_PI;
    }
    while (a < 0.0f)
    {
        a += TWO_PI;
    }
    return a;
}

static float angle_diff(float ref, float fb)
{
    float d = ref - fb;
    while (d > M_PI)
    {
        d -= TWO_PI;
    }
    while (d < -M_PI)
    {
        d += TWO_PI;
    }
    return d;
}

static float pi_run(PIController_t *pi, float err, float ts)
{
    float p = pi->kp * err;
    pi->integral += pi->ki * err * ts;
    pi->integral = clampf(pi->integral, pi->output_min, pi->output_max);
    return clampf(p + pi->integral, pi->output_min, pi->output_max);
}

/* Clarke: abc -> αβ
 * 三相都参与采样，先保证 ia+ib+ic≈0，再转换
 */
static void clarke_transform(float ia, float ib, float ic, float *i_alpha, float *i_beta)
{
    float i_sum = (ia + ib + ic) / 3.0f;
    float ia_n = ia - i_sum;
    float ib_n = ib - i_sum;

    *i_alpha = ia_n;
    *i_beta = (ia_n + 2.0f * ib_n) * INV_SQRT3;
}

static void park_transform(float i_alpha, float i_beta, float theta, float *id, float *iq)
{
    float sin_t = sinf(theta);
    float cos_t = cosf(theta);

    *id = i_alpha * cos_t + i_beta * sin_t;
    *iq = -i_alpha * sin_t + i_beta * cos_t;
}

static void inv_park_transform(float vd, float vq, float theta, float *v_alpha, float *v_beta)
{
    float sin_t = sinf(theta);
    float cos_t = cosf(theta);

    *v_alpha = vd * cos_t - vq * sin_t;
    *v_beta = vd * sin_t + vq * cos_t;
}

/* SVPWM(min-max零序注入)
 * 输入：αβ电压
 * 输出：3相占空比
 */
static void svpwm_generate(float v_alpha, float v_beta, float vbus,
                           float *duty_a, float *duty_b, float *duty_c)
{
    float va = v_alpha;
    float vb = -0.5f * v_alpha + 0.5f * SQRT3 * v_beta;
    float vc = -0.5f * v_alpha - 0.5f * SQRT3 * v_beta;

    float v_max = fmaxf(va, fmaxf(vb, vc));
    float v_min = fminf(va, fminf(vb, vc));
    float v_offset = 0.5f * (v_max + v_min);

    va -= v_offset;
    vb -= v_offset;
    vc -= v_offset;

    *duty_a = 0.5f + va / vbus;
    *duty_b = 0.5f + vb / vbus;
    *duty_c = 0.5f + vc / vbus;

    *duty_a = clampf(*duty_a, 0.0f, 1.0f);
    *duty_b = clampf(*duty_b, 0.0f, 1.0f);
    *duty_c = clampf(*duty_c, 0.0f, 1.0f);
}

/* =============================== 硬件层 =============================== */
static uint16_t spi1_read16(uint16_t tx)
{
    SPI_I2S_SendData(SPI1, tx);
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) {}
    return SPI_I2S_ReceiveData(SPI1);
}

static uint16_t spi2_read16(uint16_t tx)
{
    SPI_I2S_SendData(SPI2, tx);
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET) {}
    return SPI_I2S_ReceiveData(SPI2);
}

static void pwm_set_duty(float da, float db, float dc)
{
    TIM_SetCompare1(TIM1, (uint16_t)(da * (TIM1_ARR_VALUE - 1)));
    TIM_SetCompare2(TIM1, (uint16_t)(db * (TIM1_ARR_VALUE - 1)));
    TIM_SetCompare3(TIM1, (uint16_t)(dc * (TIM1_ARR_VALUE - 1)));
}

/* 使用ADC规则组顺序扫描3相电流：Ia/Ib/Ic */
static void adc_start_and_read_3phase_raw(void)
{
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) {}
    s_adc_raw_ia = ADC_GetConversionValue(ADC1);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) {}
    s_adc_raw_ib = ADC_GetConversionValue(ADC1);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) {}
    s_adc_raw_ic = ADC_GetConversionValue(ADC1);
}

static void sample_phase_currents(float *ia, float *ib, float *ic)
{
    float v_ia;
    float v_ib;
    float v_ic;

    adc_start_and_read_3phase_raw();

    v_ia = ((float)s_adc_raw_ia / ADC_FULL_SCALE) * ADC_REF_VOLTAGE - s_ia_offset;
    v_ib = ((float)s_adc_raw_ib / ADC_FULL_SCALE) * ADC_REF_VOLTAGE - s_ib_offset;
    v_ic = ((float)s_adc_raw_ic / ADC_FULL_SCALE) * ADC_REF_VOLTAGE - s_ic_offset;

    *ia = v_ia / (SHUNT_RESISTANCE * CURRENT_AMP_GAIN);
    *ib = v_ib / (SHUNT_RESISTANCE * CURRENT_AMP_GAIN);
    *ic = v_ic / (SHUNT_RESISTANCE * CURRENT_AMP_GAIN);
}

/* 电流零偏校准：上电PWM关闭时执行 */
static void current_offset_calibration(void)
{
    uint32_t sum_ia = 0;
    uint32_t sum_ib = 0;
    uint32_t sum_ic = 0;
    uint16_t i;

    for (i = 0; i < 512; i++)
    {
        adc_start_and_read_3phase_raw();
        sum_ia += s_adc_raw_ia;
        sum_ib += s_adc_raw_ib;
        sum_ic += s_adc_raw_ic;
    }

    s_ia_offset = ((float)(sum_ia / 512U) / ADC_FULL_SCALE) * ADC_REF_VOLTAGE;
    s_ib_offset = ((float)(sum_ib / 512U) / ADC_FULL_SCALE) * ADC_REF_VOLTAGE;
    s_ic_offset = ((float)(sum_ic / 512U) / ADC_FULL_SCALE) * ADC_REF_VOLTAGE;
}

static void hw_init(void)
{
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
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

    /* SPI1(MA732): PA4 CS, PA5 SCK, PA6 MISO, PA7 MOSI */
    gpio.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_6;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);
    gpio.GPIO_Pin = MA732_CS_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(MA732_CS_PORT, &gpio);
    GPIO_SetBits(MA732_CS_PORT, MA732_CS_PIN);

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

    /* SPI2(磁栅): PB12 CS, PB13 SCK, PB14 MISO, PB15 MOSI */
    gpio.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_14;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &gpio);
    gpio.GPIO_Pin = MAGSCALE_CS_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(MAGSCALE_CS_PORT, &gpio);
    GPIO_SetBits(MAGSCALE_CS_PORT, MAGSCALE_CS_PIN);

    SPI_Init(SPI2, &spi);
    SPI_Cmd(SPI2, ENABLE);

    /* TIM1: 三相PWM */
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
    oc.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OC1Init(TIM1, &oc);
    TIM_OC2Init(TIM1, &oc);
    TIM_OC3Init(TIM1, &oc);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

    bdtr.TIM_OSSRState = TIM_OSSRState_Enable;
    bdtr.TIM_OSSIState = TIM_OSSIState_Enable;
    bdtr.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
    bdtr.TIM_DeadTime = 72;
    bdtr.TIM_Break = TIM_Break_Disable;
    bdtr.TIM_BreakPolarity = TIM_BreakPolarity_High;
    bdtr.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
    TIM_BDTRConfig(TIM1, &bdtr);

    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);

    /* ADC1：扫描3通道采3相电流 */
    ADC_StructInit(&adc);
    adc.ADC_Mode = ADC_Mode_Independent;
    adc.ADC_ScanConvMode = ENABLE;
    adc.ADC_ContinuousConvMode = DISABLE;
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel = 3;
    ADC_Init(ADC1, &adc);

    ADC_RegularChannelConfig(ADC1, ADC_CHANNEL_IA, 1, ADC_SampleTime_13Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_CHANNEL_IB, 2, ADC_SampleTime_13Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_CHANNEL_IC, 3, ADC_SampleTime_13Cycles5);

    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1)) {}
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1)) {}
}

/* ============================= 传感器读取 ============================= */
float MA732_ReadMechanicalAngleRad(void)
{
    uint16_t raw;
    GPIO_ResetBits(MA732_CS_PORT, MA732_CS_PIN);
    raw = spi1_read16(0x0000U) & 0x3FFFU;
    GPIO_SetBits(MA732_CS_PORT, MA732_CS_PIN);

    return ((float)raw / MA732_RESOLUTION) * TWO_PI;
}

float MA732_ReadMechanicalSpeedRadPerSec(void)
{
    static float last_angle = 0.0f;
    float now = MA732_ReadMechanicalAngleRad();
    float d = angle_diff(now, last_angle);
    last_angle = now;
    return d / TS_SPEED_LOOP;
}

/* 磁栅SPI仅获取机械位置 */
float MagScaleSPI_ReadMechanicalPositionRad(void)
{
    uint16_t raw;
    GPIO_ResetBits(MAGSCALE_CS_PORT, MAGSCALE_CS_PIN);
    raw = spi2_read16(0x0000U) & 0x3FFFU;
    GPIO_SetBits(MAGSCALE_CS_PORT, MAGSCALE_CS_PIN);

    return ((float)raw / MAGSCALE_SPI_RESOLUTION) * TWO_PI;
}

/* ============================= 控制器主流程 ============================= */
void FOC_Init(void)
{
    hw_init();

    g_foc_state.vbus = VBUS_DEFAULT;
    g_foc_state.id_ref = 0.0f;
    g_foc_state.iq_ref = 0.0f;
    g_foc_state.speed_ref = 0.0f;
    g_foc_state.position_ref = 0.0f;
    g_foc_state.elec_zero_offset = 0.0f;

    s_id_pi.kp = 1.2f;
    s_id_pi.ki = 300.0f;
    s_id_pi.integral = 0.0f;
    s_id_pi.output_min = -12.0f;
    s_id_pi.output_max = 12.0f;

    s_iq_pi = s_id_pi;

    s_speed_pi.kp = 0.08f;
    s_speed_pi.ki = 5.0f;
    s_speed_pi.integral = 0.0f;
    s_speed_pi.output_min = -10.0f;
    s_speed_pi.output_max = 10.0f;

    s_pos_pi.kp = 8.0f;
    s_pos_pi.ki = 2.0f;
    s_pos_pi.integral = 0.0f;
    s_pos_pi.output_min = -100.0f;
    s_pos_pi.output_max = 100.0f;

    current_offset_calibration();
}

/* 找电角度（零电角标定）
 * 方法：给定小的d轴励磁(或q轴锁定)，让转子吸附到已知磁链方向，读取MA732机械角并映射成电角零偏。
 */
void FOC_RunElecAngleCalibration(void)
{
    float locked_mech_angle;

    /* 简化示例：维持一个小的定向电压矢量（电角=0）一段时间 */
    uint16_t i;
    for (i = 0; i < 3000; i++)
    {
        float duty_a;
        float duty_b;
        float duty_c;
        svpwm_generate(2.0f, 0.0f, g_foc_state.vbus, &duty_a, &duty_b, &duty_c);
        pwm_set_duty(duty_a, duty_b, duty_c);
    }

    locked_mech_angle = MA732_ReadMechanicalAngleRad();
    g_foc_state.elec_zero_offset = wrap_angle(locked_mech_angle * POLE_PAIRS);

    /* 标定结束后输出置中 */
    pwm_set_duty(0.5f, 0.5f, 0.5f);
}

void FOC_ControlLoop_ISR(void)
{
    float i_alpha;
    float i_beta;
    float v_alpha;
    float v_beta;
    float duty_a;
    float duty_b;
    float duty_c;
    float v_mag;
    float v_limit;
    float pos_err;

    /* 1) 位置环：磁栅SPI只用于位置 */
    s_pos_div++;
    if (s_pos_div >= (uint16_t)(CONTROL_FREQUENCY_HZ * TS_POSITION_LOOP))
    {
        s_pos_div = 0;
        g_foc_state.mech_angle = MagScaleSPI_ReadMechanicalPositionRad();
        pos_err = angle_diff(g_foc_state.position_ref, g_foc_state.mech_angle);
        g_foc_state.speed_ref = pi_run(&s_pos_pi, pos_err, TS_POSITION_LOOP);
    }

    /* 2) 速度环：MA732计算机械速度 */
    s_speed_div++;
    if (s_speed_div >= (uint16_t)(CONTROL_FREQUENCY_HZ * TS_SPEED_LOOP))
    {
        s_speed_div = 0;
        g_foc_state.mech_speed = MA732_ReadMechanicalSpeedRadPerSec();
        g_foc_state.iq_ref = pi_run(&s_speed_pi,
                                    g_foc_state.speed_ref - g_foc_state.mech_speed,
                                    TS_SPEED_LOOP);
    }

    /* 3) MA732角度 -> 电角度（含零偏） */
    g_foc_state.elec_angle = wrap_angle(MA732_ReadMechanicalAngleRad() * POLE_PAIRS -
                                        g_foc_state.elec_zero_offset);

    /* 4) 三相电流采样 + Clarke/Park */
    sample_phase_currents(&g_foc_state.ia, &g_foc_state.ib, &g_foc_state.ic);
    clarke_transform(g_foc_state.ia, g_foc_state.ib, g_foc_state.ic, &i_alpha, &i_beta);
    park_transform(i_alpha, i_beta, g_foc_state.elec_angle, &g_foc_state.id, &g_foc_state.iq);

    /* 5) 电流环 */
    g_foc_state.vd = pi_run(&s_id_pi, g_foc_state.id_ref - g_foc_state.id, TS_CURRENT_LOOP);
    g_foc_state.vq = pi_run(&s_iq_pi, g_foc_state.iq_ref - g_foc_state.iq, TS_CURRENT_LOOP);

    /* 6) 电压限幅 */
    v_limit = g_foc_state.vbus * MAX_MODULATION;
    v_mag = sqrtf(g_foc_state.vd * g_foc_state.vd + g_foc_state.vq * g_foc_state.vq);
    if (v_mag > v_limit)
    {
        float scale = v_limit / v_mag;
        g_foc_state.vd *= scale;
        g_foc_state.vq *= scale;
    }

    /* 7) 逆Park + SVPWM */
    inv_park_transform(g_foc_state.vd, g_foc_state.vq, g_foc_state.elec_angle, &v_alpha, &v_beta);
    svpwm_generate(v_alpha, v_beta, g_foc_state.vbus, &duty_a, &duty_b, &duty_c);
    pwm_set_duty(duty_a, duty_b, duty_c);
}

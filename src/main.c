#include "stm32f10x.h"
#include "foc_control.h"

static void nvic_init(void)
{
    NVIC_InitTypeDef nvic;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    nvic.NVIC_IRQChannel = TIM1_UP_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

static void tim1_update_irq_init(void)
{
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
}

int main(void)
{
    SystemInit();
    nvic_init();

    FOC_Init();

    /* 上电后先做一次找电角度 */
    FOC_RunElecAngleCalibration();

    tim1_update_irq_init();

    /* 示例：给定机械位置90° */
    g_foc_state.position_ref = 1.5707963f;

    while (1)
    {
    }
}

void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
        FOC_ControlLoop_ISR();
    }
}

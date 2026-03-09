#include "stm32f10x.h"
#include "foc_control.h"

/*
 * 示例主程序：
 * - 初始化系统时钟（依赖 system_stm32f10x.c）
 * - 初始化FOC模块
 * - 在TIM1更新中断中执行FOC_ControlLoop_ISR()
 */

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
    /* 触发控制中断：可与PWM更新同步 */
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
}

int main(void)
{
    SystemInit();
    nvic_init();
    FOC_Init();
    tim1_update_irq_init();

    /* 演示：目标机械位置=90° */
    g_foc_state.position_ref = 1.5707963f;

    while (1)
    {
        /*
         * 主循环可放通信、参数整定、状态机等。
         * 实时控制在中断中完成。
         */
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

# STM32F103 有感FOC（SPL）示例：MA732角度/速度 + 磁栅SPI位置 + 三环控制

本版本按工程需求实现：

- **MA732（SPI1）**：机械角度 + 机械速度计算（速度由角度差分得到）
- **磁栅SPI（SPI2）**：仅用于机械位置闭环
- **ADC三相电流采样**：Ia/Ib/Ic 全采样
- **完整变换链路**：Clarke -> Park -> 电流PI -> 逆Park -> SVPWM
- **找电角度**：提供 `FOC_RunElecAngleCalibration()` 做零电角偏移标定
- **三环调节**：位置环(外) -> 速度环(中) -> 电流环(内)

## 文件

- `inc/foc_control.h`：参数、结构体、接口
- `src/foc_control.c`：硬件初始化 + 控制算法 + 传感器读取
- `src/main.c`：启动、找电角度、TIM1中断调用控制环

## 关键说明

1. 示例使用标准外设库（SPL）接口，需在你的工程中提供 `stm32f10x.h` 和对应库文件。
2. ADC三相采样通道默认示例为 `PA0/PA1/PA2`，请按硬件修改。
3. MA732/磁栅SPI时序、寄存器格式在不同器件上可能不同，`spi_read16` 和掩码需按手册确认。
4. `FOC_RunElecAngleCalibration()` 给出了找电角度的基础流程，量产时建议加入超时与电流保护。

#ifndef _TREZOR_T3T1_H
#define _TREZOR_T3T1_H

#define DISPLAY_RESX 240
#define DISPLAY_RESY 240

#define VDD_1V8 1
#define HSE_16MHZ 1

#define USE_SD_CARD 1
#define USE_I2C 1
#define USE_TOUCH 1
#define USE_SBU 1
#define USE_RGB_COLORS 1
#define USE_DISP_I8080_8BIT_DW 1
#define USE_HAPTIC 1
#define USE_BACKLIGHT 1
#define USE_HASH_PROCESSOR 1

#include "displays/panels/lx154a2422.h"
#include "displays/st7789v.h"
#define DISPLAY_IDENTIFY 1
#define DISPLAY_TE_PORT GPIOD
#define DISPLAY_TE_PIN GPIO_PIN_12
#define DISPLAY_TE_INTERRUPT_HANDLER EXTI12_IRQHandler
#define DISPLAY_TE_INTERRUPT_NUM EXTI12_IRQn
#define DISPLAY_TE_INTERRUPT_GPIOSEL EXTI_GPIOD
#define DISPLAY_TE_INTERRUPT_EXTI_LINE EXTI_LINE_12

#define DISPLAY_PANEL_INIT_SEQ lx154a2422_init_seq
#define DISPLAY_PANEL_ROTATE lx154a2422_rotate
#define TRANSFORM_TOUCH_COORDS lx154a2422_transform_touch_coords

#define BACKLIGHT_PWM_FREQ 12500
#define BACKLIGHT_PWM_TIM TIM8
#define BACKLIGHT_PWM_TIM_CLK_EN __HAL_RCC_TIM8_CLK_ENABLE
#define BACKLIGHT_PWM_TIM_AF GPIO_AF3_TIM8
#define BACKLIGHT_PWM_TIM_OCMODE TIM_OCMODE_PWM1
#define BACKLIGHT_PWM_TIM_CHANNEL TIM_CHANNEL_1
#define BACKLIGHT_PWM_TIM_CCR CCR1
#define BACKLIGHT_PWM_PIN GPIO_PIN_6
#define BACKLIGHT_PWM_PORT GPIOC
#define BACKLIGHT_PWM_PORT_CLK_EN __HAL_RCC_GPIOC_CLK_ENABLE

#define I2C_COUNT 2
#define I2C_INSTANCE_0 I2C1
#define I2C_INSTANCE_0_CLK_EN __HAL_RCC_I2C1_CLK_ENABLE
#define I2C_INSTANCE_0_CLK_DIS __HAL_RCC_I2C1_CLK_DISABLE
#define I2C_INSTANCE_0_PIN_AF GPIO_AF4_I2C1
#define I2C_INSTANCE_0_SDA_PORT GPIOB
#define I2C_INSTANCE_0_SDA_PIN GPIO_PIN_7
#define I2C_INSTANCE_0_SDA_CLK_EN __HAL_RCC_GPIOB_CLK_ENABLE
#define I2C_INSTANCE_0_SCL_PORT GPIOB
#define I2C_INSTANCE_0_SCL_PIN GPIO_PIN_6
#define I2C_INSTANCE_0_SCL_CLK_EN __HAL_RCC_GPIOB_CLK_ENABLE
#define I2C_INSTANCE_0_RESET_REG &RCC->APB1RSTR1
#define I2C_INSTANCE_0_RESET_BIT RCC_APB1RSTR1_I2C1RST

#define I2C_INSTANCE_1 I2C2
#define I2C_INSTANCE_1_CLK_EN __HAL_RCC_I2C2_CLK_ENABLE
#define I2C_INSTANCE_1_CLK_DIS __HAL_RCC_I2C2_CLK_DISABLE
#define I2C_INSTANCE_1_PIN_AF GPIO_AF4_I2C2
#define I2C_INSTANCE_1_SDA_PORT GPIOB
#define I2C_INSTANCE_1_SDA_PIN GPIO_PIN_14
#define I2C_INSTANCE_1_SDA_CLK_EN __HAL_RCC_GPIOB_CLK_ENABLE
#define I2C_INSTANCE_1_SCL_PORT GPIOB
#define I2C_INSTANCE_1_SCL_PIN GPIO_PIN_13
#define I2C_INSTANCE_1_SCL_CLK_EN __HAL_RCC_GPIOB_CLK_ENABLE
#define I2C_INSTANCE_1_RESET_REG &RCC->APB1RSTR1
#define I2C_INSTANCE_1_RESET_BIT RCC_APB1RSTR1_I2C2RST

#define TOUCH_SENSITIVITY 0x40
#define TOUCH_I2C_INSTANCE 0
#define TOUCH_RST_PORT GPIOC
#define TOUCH_RST_PIN GPIO_PIN_5
#define TOUCH_INT_PORT GPIOC
#define TOUCH_INT_PIN GPIO_PIN_4
#define TOUCH_ON_PORT GPIOB
#define TOUCH_ON_PIN GPIO_PIN_0

#define DRV2625_I2C_INSTANCE 1
#define HAPTIC_ACTUATOR "actuators/vg1040003d.h"

#define OPTIGA_I2C_INSTANCE 1
#define OPTIGA_RST_PORT GPIOB
#define OPTIGA_RST_PIN GPIO_PIN_1
#define OPTIGA_RST_CLK_EN __HAL_RCC_GPIOB_CLK_ENABLE

#define SD_DETECT_PORT GPIOC
#define SD_DETECT_PIN GPIO_PIN_13
#define SD_ENABLE_PORT GPIOC
#define SD_ENABLE_PIN GPIO_PIN_0

#endif  //_TREZOR_T_H

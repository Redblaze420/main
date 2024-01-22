#ifndef _TREZOR_R_V10_H
#define _TREZOR_R_V10_H

#define USE_BUTTON 1
#define USE_SBU 1
#define USE_I2C 1
#define USE_CONSUMPTION_MASK 1

#include "displays/vg-2864ksweg01.h"

#define BTN_LEFT_PIN GPIO_PIN_10
#define BTN_LEFT_PORT GPIOC
#define BTN_LEFT_CLK_ENA __HAL_RCC_GPIOC_CLK_ENABLE
#define BTN_RIGHT_PIN GPIO_PIN_15
#define BTN_RIGHT_PORT GPIOE
#define BTN_RIGHT_CLK_ENA __HAL_RCC_GPIOE_CLK_ENABLE

#define OLED_DC_PORT GPIOD
#define OLED_DC_PIN GPIO_PIN_0  // PD0 | Data/Command
#define OLED_DC_CLK_ENA __HAL_RCC_GPIOD_CLK_ENABLE
#define OLED_CS_PORT GPIOE
#define OLED_CS_PIN GPIO_PIN_4  // PE4 | SPI Select
#define OLED_CS_CLK_ENA __HAL_RCC_GPIOE_CLK_ENABLE
#define OLED_RST_PORT GPIOD
#define OLED_RST_PIN GPIO_PIN_1  // PD1 | Reset display
#define OLED_RST_CLK_ENA __HAL_RCC_GPIOD_CLK_ENABLE

#define OLED_SPI SPI4
#define OLED_SPI_AF GPIO_AF5_SPI4
#define OLED_SPI_CLK_ENA __HAL_RCC_SPI4_CLK_ENABLE
#define OLED_SPI_SCK_PORT GPIOE
#define OLED_SPI_SCK_PIN GPIO_PIN_2  // PE2 | SPI SCK
#define OLED_SPI_SCK_CLK_ENA __HAL_RCC_GPIOE_CLK_ENABLE
#define OLED_SPI_MOSI_PORT GPIOE
#define OLED_SPI_MOSI_PIN GPIO_PIN_6  // PE6 | SPI MOSI
#define OLED_SPI_MOSI_CLK_ENA __HAL_RCC_GPIOE_CLK_ENABLE

#define I2C_COUNT 1
#define I2C_INSTANCE_1 I2C2
#define I2C_INSTANCE_1_CLK_EN __HAL_RCC_I2C2_CLK_ENABLE
#define I2C_INSTANCE_1_CLK_DIS __HAL_RCC_I2C2_CLK_DISABLE
#define I2C_INSTANCE_1_PIN_AF GPIO_AF4_I2C2
#define I2C_INSTANCE_1_SDA_PORT GPIOB
#define I2C_INSTANCE_1_SDA_PIN GPIO_PIN_11
#define I2C_INSTANCE_1_SDA_CLK_EN __HAL_RCC_GPIOB_CLK_ENABLE
#define I2C_INSTANCE_1_SCL_PORT GPIOB
#define I2C_INSTANCE_1_SCL_PIN GPIO_PIN_10
#define I2C_INSTANCE_1_SCL_CLK_EN __HAL_RCC_GPIOB_CLK_ENABLE
#define I2C_INSTANCE_1_RESET_FLG RCC_APB1RSTR_I2C2RST

#define OPTIGA_I2C_INSTANCE 0

#endif  //_TREZOR_R_V10_H

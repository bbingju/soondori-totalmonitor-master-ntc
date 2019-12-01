/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SD_LED_Pin GPIO_PIN_2
#define SD_LED_GPIO_Port GPIOE
#define POWER_LED_Pin GPIO_PIN_3
#define POWER_LED_GPIO_Port GPIOE
#define MODE_BUTTON_Pin GPIO_PIN_4
#define MODE_BUTTON_GPIO_Port GPIOE
#define MODE_BUTTON_EXTI_IRQn EXTI4_IRQn
#define UP_BUTTON_Pin GPIO_PIN_5
#define UP_BUTTON_GPIO_Port GPIOE
#define UP_BUTTON_EXTI_IRQn EXTI9_5_IRQn
#define DOWN_BUTTON_Pin GPIO_PIN_6
#define DOWN_BUTTON_GPIO_Port GPIOE
#define DOWN_BUTTON_EXTI_IRQn EXTI9_5_IRQn
#define RESET_BUTTON_Pin GPIO_PIN_0
#define RESET_BUTTON_GPIO_Port GPIOC
#define BUZZER_SEL_Pin GPIO_PIN_1
#define BUZZER_SEL_GPIO_Port GPIOC
#define RELAY_SEL_Pin GPIO_PIN_2
#define RELAY_SEL_GPIO_Port GPIOC
#define RTD_ADC_Pin GPIO_PIN_0
#define RTD_ADC_GPIO_Port GPIOA
#define BD_TEMP_Pin GPIO_PIN_1
#define BD_TEMP_GPIO_Port GPIOA
#define BD_HUMI_Pin GPIO_PIN_2
#define BD_HUMI_GPIO_Port GPIOA
#define END_REG1_Pin GPIO_PIN_4
#define END_REG1_GPIO_Port GPIOA
#define CSP_SCK_Pin GPIO_PIN_5
#define CSP_SCK_GPIO_Port GPIOA
#define CSP_SCKA6_Pin GPIO_PIN_6
#define CSP_SCKA6_GPIO_Port GPIOA
#define CSP_MOSI_Pin GPIO_PIN_7
#define CSP_MOSI_GPIO_Port GPIOA
#define END_REG2_Pin GPIO_PIN_4
#define END_REG2_GPIO_Port GPIOC
#define CSP_1DE_Pin GPIO_PIN_5
#define CSP_1DE_GPIO_Port GPIOC
#define CSP_2DE_Pin GPIO_PIN_0
#define CSP_2DE_GPIO_Port GPIOB
#define SLAVE_OE1_Pin GPIO_PIN_7
#define SLAVE_OE1_GPIO_Port GPIOE
#define UART_EN_BT_Pin GPIO_PIN_12
#define UART_EN_BT_GPIO_Port GPIOE
#define UART_EN_SLOT_Pin GPIO_PIN_13
#define UART_EN_SLOT_GPIO_Port GPIOE
#define DIGI2_LATCH_Pin GPIO_PIN_14
#define DIGI2_LATCH_GPIO_Port GPIOE
#define DIGI2_ENABLE_Pin GPIO_PIN_15
#define DIGI2_ENABLE_GPIO_Port GPIOE
#define DIGI2_SCL_Pin GPIO_PIN_10
#define DIGI2_SCL_GPIO_Port GPIOB
#define DIGI2_SDA_Pin GPIO_PIN_11
#define DIGI2_SDA_GPIO_Port GPIOB
#define SLAVE_DEBUGE_Pin GPIO_PIN_12
#define SLAVE_DEBUGE_GPIO_Port GPIOB
#define SLAVE_CS5_Pin GPIO_PIN_14
#define SLAVE_CS5_GPIO_Port GPIOB
#define SLAVE_CS4_Pin GPIO_PIN_15
#define SLAVE_CS4_GPIO_Port GPIOB
#define SLAVE_CS0_Pin GPIO_PIN_8
#define SLAVE_CS0_GPIO_Port GPIOD
#define SLAVE_CS1_Pin GPIO_PIN_9
#define SLAVE_CS1_GPIO_Port GPIOD
#define SLAVE_CS2_Pin GPIO_PIN_10
#define SLAVE_CS2_GPIO_Port GPIOD
#define SLAVE_CS3_Pin GPIO_PIN_11
#define SLAVE_CS3_GPIO_Port GPIOD
#define SLAVE_OE_Pin GPIO_PIN_12
#define SLAVE_OE_GPIO_Port GPIOD
#define DEV_DEBUGE_Pin GPIO_PIN_13
#define DEV_DEBUGE_GPIO_Port GPIOD
#define MCU1_MCU2_CHECK_Pin GPIO_PIN_14
#define MCU1_MCU2_CHECK_GPIO_Port GPIOD
#define DEV_BEAT_OUT_Pin GPIO_PIN_15
#define DEV_BEAT_OUT_GPIO_Port GPIOD
#define DEV_BEAT_IN_Pin GPIO_PIN_6
#define DEV_BEAT_IN_GPIO_Port GPIOC
#define DEV_ENABLE_OUT_Pin GPIO_PIN_7
#define DEV_ENABLE_OUT_GPIO_Port GPIOC
#define SMPS2_DECTECT_Pin GPIO_PIN_8
#define SMPS2_DECTECT_GPIO_Port GPIOA
#define SMPS1_DECTECT_Pin GPIO_PIN_9
#define SMPS1_DECTECT_GPIO_Port GPIOA
#define BUFFER_OE_Pin GPIO_PIN_10
#define BUFFER_OE_GPIO_Port GPIOA
#define SDIO_POWER_Pin GPIO_PIN_0
#define SDIO_POWER_GPIO_Port GPIOD
#define SDIO_DECTECT_Pin GPIO_PIN_1
#define SDIO_DECTECT_GPIO_Port GPIOD
#define BLUETOOTH_FACTORY_RESET_Pin GPIO_PIN_3
#define BLUETOOTH_FACTORY_RESET_GPIO_Port GPIOD
#define BLUETOOTH_MODE_Pin GPIO_PIN_4
#define BLUETOOTH_MODE_GPIO_Port GPIOD
#define BLUETOOTH_TX_Pin GPIO_PIN_5
#define BLUETOOTH_TX_GPIO_Port GPIOD
#define BLUETOOTH_RX_Pin GPIO_PIN_6
#define BLUETOOTH_RX_GPIO_Port GPIOD
#define RS485_EN_Pin GPIO_PIN_5
#define RS485_EN_GPIO_Port GPIOB
#define RS485_TX_Pin GPIO_PIN_6
#define RS485_TX_GPIO_Port GPIOB
#define RS485_RX_Pin GPIO_PIN_7
#define RS485_RX_GPIO_Port GPIOB
#define DIGI1_SCL_Pin GPIO_PIN_8
#define DIGI1_SCL_GPIO_Port GPIOB
#define DIGI1_SDA_Pin GPIO_PIN_9
#define DIGI1_SDA_GPIO_Port GPIOB
#define DIGI1_ENABLE_Pin GPIO_PIN_0
#define DIGI1_ENABLE_GPIO_Port GPIOE
#define DIGI1_LATCH_Pin GPIO_PIN_1
#define DIGI1_LATCH_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

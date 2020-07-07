/*
 * Copyright (c) 2016-2018 Granite Devices Oy
 * ---------------------------------------------------------------------------
 * This file is made available under the terms of Granite Devices Software
 * End-User License Agreement, available at https://granitedevices.com/legal
 *
 * Contributions and modifications are allowed only under the terms of Granite
 * Devices Contributor License Agreement, available at
 * https://granitedevices.com/legal
 * ---------------------------------------------------------------------------
 * 3rd-party contributors:
 *
 *
 *
 * ---------------------------------------------------------------------------
*/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "stdio.h"
#ifndef WIN32
#include "fw_version.h"
#endif

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define X12_LOWER_1_Pin GPIO_PIN_2
#define X12_LOWER_1_GPIO_Port GPIOE
#define X12_LOWER_2_Pin GPIO_PIN_3
#define X12_LOWER_2_GPIO_Port GPIOE
#define X12_LOWER_4_Pin GPIO_PIN_4
#define X12_LOWER_4_GPIO_Port GPIOE
#define X12_UPPER_6_Pin GPIO_PIN_5
#define X12_UPPER_6_GPIO_Port GPIOE
#define STM_PE6_Pin GPIO_PIN_6
#define STM_PE6_GPIO_Port GPIOE
#define X12_LOWER_6_Pin GPIO_PIN_13
#define X12_LOWER_6_GPIO_Port GPIOC
#define X12_LOWER_7_Pin GPIO_PIN_14
#define X12_LOWER_7_GPIO_Port GPIOC
#define X11_LOWER_1_Pin GPIO_PIN_15
#define X11_LOWER_1_GPIO_Port GPIOC
#define EXT_POT_IN_3_Pin GPIO_PIN_0
#define EXT_POT_IN_3_GPIO_Port GPIOC
#define EXT_POT_IN_2_Pin GPIO_PIN_1
#define EXT_POT_IN_2_GPIO_Port GPIOC
#define EXT_POT_IN_1_Pin GPIO_PIN_2
#define EXT_POT_IN_1_GPIO_Port GPIOC
#define X11_LOWER_2_Pin GPIO_PIN_3
#define X11_LOWER_2_GPIO_Port GPIOC
#define STM_ENC_IN_A_Pin GPIO_PIN_0
#define STM_ENC_IN_A_GPIO_Port GPIOA
#define STM_ENC_IN_B_Pin GPIO_PIN_1
#define STM_ENC_IN_B_GPIO_Port GPIOA
#define STM_ENC_IN_C_Pin GPIO_PIN_2
#define STM_ENC_IN_C_GPIO_Port GPIOA
#define X11_LOWER_3_Pin GPIO_PIN_3
#define X11_LOWER_3_GPIO_Port GPIOA
#define X11_LOWER_5_Pin GPIO_PIN_4
#define X11_LOWER_5_GPIO_Port GPIOA
#define X11_LOWER_6_Pin GPIO_PIN_5
#define X11_LOWER_6_GPIO_Port GPIOA
#define X11_LOWER_7_Pin GPIO_PIN_6
#define X11_LOWER_7_GPIO_Port GPIOA
#define X11_UPPER_6_Pin GPIO_PIN_7
#define X11_UPPER_6_GPIO_Port GPIOA
#define X11_UPPER_5_Pin GPIO_PIN_4
#define X11_UPPER_5_GPIO_Port GPIOC
#define X11_UPPER_3_Pin GPIO_PIN_5
#define X11_UPPER_3_GPIO_Port GPIOC
#define X11_UPPER_2_Pin GPIO_PIN_0
#define X11_UPPER_2_GPIO_Port GPIOB
#define X11_UPPER_1_Pin GPIO_PIN_1
#define X11_UPPER_1_GPIO_Port GPIOB
#define GP01_STM_Pin GPIO_PIN_2
#define GP01_STM_GPIO_Port GPIOB
#define STM_PE7_Pin GPIO_PIN_7
#define STM_PE7_GPIO_Port GPIOE
#define STM_PE8_Pin GPIO_PIN_8
#define STM_PE8_GPIO_Port GPIOE
#define STM_IONI_PWM_OUT_HSIN2_Pin GPIO_PIN_9
#define STM_IONI_PWM_OUT_HSIN2_GPIO_Port GPIOE
#define STM_PE10_Pin GPIO_PIN_10
#define STM_PE10_GPIO_Port GPIOE
#define STM_IONI_DIR_OUT_HSIN1_Pin GPIO_PIN_11
#define STM_IONI_DIR_OUT_HSIN1_GPIO_Port GPIOE
#define HX711_CLKOUT_Pin GPIO_PIN_12
#define HX711_CLKOUT_GPIO_Port GPIOE
#define STM_PE13_Pin GPIO_PIN_13
#define STM_PE13_GPIO_Port GPIOE
#define STM_PE14_Pin GPIO_PIN_14
#define STM_PE14_GPIO_Port GPIOE
#define STM_PE15_Pin GPIO_PIN_15
#define STM_PE15_GPIO_Port GPIOE
#define RS485_TXEN_STM_Pin GPIO_PIN_8
#define RS485_TXEN_STM_GPIO_Port GPIOD
#define STM_PD9_Pin GPIO_PIN_9
#define STM_PD9_GPIO_Port GPIOD
#define STM_PD10_Pin GPIO_PIN_10
#define STM_PD10_GPIO_Port GPIOD
#define STM_PD11_Pin GPIO_PIN_11
#define STM_PD11_GPIO_Port GPIOD
#define LED4_OUT_Pin GPIO_PIN_12
#define LED4_OUT_GPIO_Port GPIOD
#define LED3_OUT_Pin GPIO_PIN_13
#define LED3_OUT_GPIO_Port GPIOD
#define LED2_OUT_Pin GPIO_PIN_14
#define LED2_OUT_GPIO_Port GPIOD
#define LED1_CLIPPING_OUT_Pin GPIO_PIN_15
#define LED1_CLIPPING_OUT_GPIO_Port GPIOD
#define X12_UPPER_2_Pin GPIO_PIN_6
#define X12_UPPER_2_GPIO_Port GPIOC
#define X12_UPPER_3_Pin GPIO_PIN_7
#define X12_UPPER_3_GPIO_Port GPIOC
#define X12_UPPER_4_Pin GPIO_PIN_8
#define X12_UPPER_4_GPIO_Port GPIOC
#define X12_UPPER_5_Pin GPIO_PIN_9
#define X12_UPPER_5_GPIO_Port GPIOC
#define X12_UPPER_7_Pin GPIO_PIN_10
#define X12_UPPER_7_GPIO_Port GPIOC
#define X12_UPPER_1_Pin GPIO_PIN_11
#define X12_UPPER_1_GPIO_Port GPIOC
#define X12_LOWER_5_Pin GPIO_PIN_12
#define X12_LOWER_5_GPIO_Port GPIOC
#define IDSEL_0_Pin GPIO_PIN_0
#define IDSEL_0_GPIO_Port GPIOD
#define IDSEL_1_Pin GPIO_PIN_1
#define IDSEL_1_GPIO_Port GPIOD
#define IDSEL_2_Pin GPIO_PIN_2
#define IDSEL_2_GPIO_Port GPIOD
#define FTDI_USB_SLEEP_IN_Pin GPIO_PIN_3
#define FTDI_USB_SLEEP_IN_GPIO_Port GPIOD
#define USB_GRANITY_VCC_SENSE_Pin GPIO_PIN_4
#define USB_GRANITY_VCC_SENSE_GPIO_Port GPIOD
#define USB_HID_VCC_SENSE_Pin GPIO_PIN_5
#define USB_HID_VCC_SENSE_GPIO_Port GPIOD
#define STM_PD6_Pin GPIO_PIN_6
#define STM_PD6_GPIO_Port GPIOD
#define STM_PD7_Pin GPIO_PIN_7
#define STM_PD7_GPIO_Port GPIOD
#define GPI4_CLEAR_FAULTS_Pin GPIO_PIN_4
#define GPI4_CLEAR_FAULTS_GPIO_Port GPIOB
#define DIPSW_2_Pin GPIO_PIN_8
#define DIPSW_2_GPIO_Port GPIOB
#define STM_PB9_Pin GPIO_PIN_9
#define STM_PB9_GPIO_Port GPIOB
#define X15_3_Pin GPIO_PIN_0
#define X15_3_GPIO_Port GPIOE
#define X12_LOWER_3_Pin GPIO_PIN_1
#define X12_LOWER_3_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

#define NUM_ADC_CHANNELS 13
#define ADC_NUM_AVERAGING 32
#define ADC1_BUFFER_LENGTH NUM_ADC_CHANNELS*ADC_NUM_AVERAGING

#define X11_upper_1 1  //adc1_chan9 (brake)
#define X11_upper_2 2  //adc1_chan8 (gas)
#define X11_upper_3 3  //adc1_chan15
#define X11_upper_5 4  //adc1_chan14 (clutch)
#define X11_upper_6 5  //adc1_chan7

#define X11_lower_2 6  //adc1_chan13
#define X11_lower_3 7  //adc1_chan3
#define X11_lower_5 8  //adc1_chan4
#define X11_lower_6 9  //adc1_chan5
#define X11_lower_7 10  //adc1_chan6

#define ex_pot_1    11  //adc1_chan12
#define ex_pot_2    12  //adc1_chan11
#define ex_pot_3    13  //adc1_chan10

#define IONI_FW_VER 10716
/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 
#endif /* __MAIN_H */


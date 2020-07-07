/*
 * Copyright (c) 2016-2020 Granite Devices Oy
 * ---------------------------------------------------------------------------
 * This file is made available under the terms of Granite Devices Software
 * End-User License Agreement, available at https://granitedevices.com/legal
 *
 * Contributions and modifications are allowed only under the terms of Granite
 * Devices Contributor License Agreement, available at
 * https://granitedevices.com/legal
 * ---------------------------------------------------------------------------
 * 3rd-party contributors:
 * Etienne Saint-Paul
 *
 *
 * ---------------------------------------------------------------------------
*/

/* This file contains all the hardware-specific function implementations from
 * different classes. This enables somewhat hardware independent development
 * and allows to share files/implementations for only some hardware variants.
 */

#include "hw_functions.h"
#include "main.h"

#include "USBGameController.h"
#include "cFFBDevice.h"
#include "simplemotioncomms.h"

extern USBGameController joystick;
extern SystemStatus currentSystemStatus;
extern void simucube_bootloader(uint32_t magic_code);
extern bool enableSMWatchdog();

void cFFBDevice::initButtonInputs() {
	int i = 0;
	button_port[i]	= X12_UPPER_1_GPIO_Port; button_pin[i]	= X12_UPPER_1_Pin;
	button_lastUpdate[i] = 0; button_currentState[i] = GPIO_PIN_RESET; button_debouncedState[i] = GPIO_PIN_RESET;
	i++;

	button_port[i]	= X12_UPPER_2_GPIO_Port; button_pin[i]	= X12_UPPER_2_Pin;
	button_lastUpdate[i] = 0; button_currentState[i] = GPIO_PIN_RESET; button_debouncedState[i] = GPIO_PIN_RESET;
	i++;

	button_port[i]	= X12_UPPER_3_GPIO_Port; button_pin[i]	= X12_UPPER_3_Pin;
	button_lastUpdate[i] = 0; button_currentState[i] = GPIO_PIN_RESET; button_debouncedState[i] = GPIO_PIN_RESET;
	i++;

	button_port[i]	= X12_UPPER_4_GPIO_Port; button_pin[i]	= X12_UPPER_4_Pin;
	button_lastUpdate[i] = 0;button_currentState[i] = GPIO_PIN_RESET;button_debouncedState[i] = GPIO_PIN_RESET;
	i++;

	button_port[i]	= X12_UPPER_5_GPIO_Port;button_pin[i]	= X12_UPPER_5_Pin;
	button_lastUpdate[i] = 0;button_currentState[i] = GPIO_PIN_RESET;button_debouncedState[i] = GPIO_PIN_RESET;
	i++;

	button_port[i]	= X12_UPPER_6_GPIO_Port;button_pin[i]	= X12_UPPER_6_Pin;
	button_lastUpdate[i] = 0;button_currentState[i] = GPIO_PIN_RESET;button_debouncedState[i] = GPIO_PIN_RESET;
	i++;

	button_port[i]	= X12_UPPER_7_GPIO_Port;button_pin[i]	= X12_UPPER_7_Pin;
	button_lastUpdate[i] = 0;button_currentState[i] = GPIO_PIN_RESET;button_debouncedState[i] = GPIO_PIN_RESET;
	i++;

	encx12u1.setInputPin(1);
	encx12u2.setInputPin(3);
	encx12u3.setInputPin(5);

	button_port[i]	= X12_LOWER_1_GPIO_Port;button_pin[i]	= X12_LOWER_1_Pin;
	button_lastUpdate[i] = 0;button_currentState[i] = GPIO_PIN_RESET;button_debouncedState[i] = GPIO_PIN_RESET;
	i++;

	button_port[i]	= X12_LOWER_2_GPIO_Port;button_pin[i]	= X12_LOWER_2_Pin;
	button_lastUpdate[i] = 0;button_currentState[i] = GPIO_PIN_RESET;button_debouncedState[i] = GPIO_PIN_RESET;
	i++;

	button_port[i]	= X12_LOWER_3_GPIO_Port;button_pin[i]	= X12_LOWER_3_Pin;
	button_lastUpdate[i] = 0;button_currentState[i] = GPIO_PIN_RESET;button_debouncedState[i] = GPIO_PIN_RESET;
	i++;

	button_port[i]	= X12_LOWER_4_GPIO_Port;button_pin[i]	= X12_LOWER_4_Pin;
	button_lastUpdate[i] = 0;button_currentState[i] = GPIO_PIN_RESET;button_debouncedState[i] = GPIO_PIN_RESET;
	i++;

	button_port[i]	= X12_LOWER_5_GPIO_Port;button_pin[i]	= X12_LOWER_5_Pin;
	button_lastUpdate[i] = 0;button_currentState[i] = GPIO_PIN_RESET;button_debouncedState[i] = GPIO_PIN_RESET;
	i++;

	button_port[i]	= X12_LOWER_6_GPIO_Port;button_pin[i]	= X12_LOWER_6_Pin;
	button_lastUpdate[i] = 0;button_currentState[i] = GPIO_PIN_RESET;button_debouncedState[i] = GPIO_PIN_RESET;
	i++;

	button_port[i]	= X12_LOWER_7_GPIO_Port;button_pin[i]	= X12_LOWER_7_Pin;
	button_lastUpdate[i] = 0;button_currentState[i] = GPIO_PIN_RESET;button_debouncedState[i] = GPIO_PIN_RESET;

	encx12l1.setInputPin(8);
	encx12l2.setInputPin(10);
	encx12l3.setInputPin(12);

	x12LowerEncodersMode = 1;
	x12UpperEncodersMode = 1;
}


void cFFBDevice::initHwEncoderButtons() {
	// set encoder modes
	int32_t setting = mConfig.hardwareConfig.hwSettings[addrX12buttonConfigs];
	uint8_t lowerMode = setting & 0xFF;
	switch(lowerMode) {
		case 0:
			x12LowerEncodersMode = 0;
			encx12l1.setDeviceType(DEVICE_TYPE_NOTHING);
			encx12l2.setDeviceType(DEVICE_TYPE_NOTHING);
			encx12l3.setDeviceType(DEVICE_TYPE_NOTHING);
			break;
		case 1:
			x12LowerEncodersMode = 1;
			encx12l1.setDeviceType(DEVICE_TYPE_ENCODER_1);
			encx12l2.setDeviceType(DEVICE_TYPE_ENCODER_1);
			encx12l3.setDeviceType(DEVICE_TYPE_ENCODER_1);
			break;
		case 2:
			x12LowerEncodersMode = 2;
			encx12l1.setDeviceType(DEVICE_TYPE_ENCODER_2);
			encx12l2.setDeviceType(DEVICE_TYPE_ENCODER_2);
			encx12l3.setDeviceType(DEVICE_TYPE_ENCODER_2);

			break;
		case 3:
			x12LowerEncodersMode = 4;
			encx12l1.setDeviceType(DEVICE_TYPE_ENCODER_4);
			encx12l2.setDeviceType(DEVICE_TYPE_ENCODER_4);
			encx12l3.setDeviceType(DEVICE_TYPE_ENCODER_4);
			break;
		default:
			x12LowerEncodersMode=0;
			break;
	}
	setting = setting >> 8;
	uint8_t upperMode = setting & 0xFF;
	switch(upperMode) {
		case 0:
			x12UpperEncodersMode = 0;
			encx12u2.setDeviceType(DEVICE_TYPE_NOTHING);
			encx12u2.setDeviceType(DEVICE_TYPE_NOTHING);
			encx12u3.setDeviceType(DEVICE_TYPE_NOTHING);
			break;
		case 1:
			x12UpperEncodersMode = 1;
			encx12u1.setDeviceType(DEVICE_TYPE_ENCODER_1);
			encx12u2.setDeviceType(DEVICE_TYPE_ENCODER_1);
			encx12u3.setDeviceType(DEVICE_TYPE_ENCODER_1);
			break;
		case 2:
			x12UpperEncodersMode = 2;
			encx12u1.setDeviceType(DEVICE_TYPE_ENCODER_2);
			encx12u2.setDeviceType(DEVICE_TYPE_ENCODER_2);
			encx12u3.setDeviceType(DEVICE_TYPE_ENCODER_2);
			break;
		case 3:
			x12UpperEncodersMode = 4;
			encx12u1.setDeviceType(DEVICE_TYPE_ENCODER_4);
			encx12u2.setDeviceType(DEVICE_TYPE_ENCODER_4);
			encx12u3.setDeviceType(DEVICE_TYPE_ENCODER_4);
			break;
		default:
			x12UpperEncodersMode = 0;
			break;
	}
	setting = setting >> 8;
	pin7shift = setting;
}

int cFFBDevice::getNumberOfButtons() {
	return 16;
}

void cFFBDevice::readHighTorqueModeSetting() {
	// do nothing
}

// Buttons[0] contains first the upper X12, then the lower X12.
#define maskX12Upper 				0b11111111111111111111111110000001
#define maskX12Lower 				0b11111111111111111100000011111111
//#define maskX12UpperForShift		0b00000000000000000000000000111111
#define maskX12UpperShiftPressed	(1<<6)
//#define maskX12LowerForShift		0b00000000000000000000000000111111
#define maskX12LowerShiftPressed	(1<<13)

void USBGameController::addHwButtonsToReport(int &i) {
	Buttons[0]=0;
	for(int j=0; j<14; j++) {
		if(gFFBDevice.button_debouncedState[j]==GPIO_PIN_RESET) {
			Buttons[0] |= (0x01<<j);
		}
	}
	int32_t backupbuttons = Buttons[0];
	uint32_t result1,result2,result3;
	if(gFFBDevice.x12LowerEncodersMode) {
		Buttons[0] &= maskX12Lower;
		gFFBDevice.encx12l1.process(backupbuttons);
		gFFBDevice.encx12l2.process(backupbuttons);
		gFFBDevice.encx12l3.process(backupbuttons);
		result1 = gFFBDevice.encx12l1.getOutputValue();
		result1 = result1 << 8;
		int result2 = gFFBDevice.encx12l2.getOutputValue();
		result2 = result2 << 10;
		int result3 = gFFBDevice.encx12l3.getOutputValue();
		result3 = result3 << 12;
		Buttons[0] |= result1 | result2 | result3;
	}
	backupbuttons = Buttons[0];
	if(gFFBDevice.x12UpperEncodersMode) {
		Buttons[0] &= maskX12Upper;
		gFFBDevice.encx12u1.process(backupbuttons);
		gFFBDevice.encx12u2.process(backupbuttons);
		gFFBDevice.encx12u3.process(backupbuttons);
		result1 = gFFBDevice.encx12u1.getOutputValue();
		result1 = result1 << 1;
		result2 = gFFBDevice.encx12u2.getOutputValue();
		result2 = result2 << 3;
		result3 = gFFBDevice.encx12u3.getOutputValue();
		result3 = result3 << 5;
		Buttons[0] |= result1 | result2 | result3;
	}
	backupbuttons = Buttons[0];
	if((gFFBDevice.pin7shift == 2) && (Buttons[0] & maskX12UpperShiftPressed)) {
		Buttons[0] &= ~maskX12UpperShiftPressed;
		Buttons[0] = Buttons[0] << 16;
	} else if ((gFFBDevice.pin7shift == 1) && (Buttons[0] & maskX12LowerShiftPressed)) {
		Buttons[0] &= ~maskX12LowerShiftPressed;
		Buttons[0] = Buttons[0] << 16;
	}

	// simucube x12 button inputs (14)+shift requires (7+6)*2 = 26 -> reserve first 32 buttons
	currentreport->data[i++] = ((Buttons[0] & 0xff) >>0);
	currentreport->data[i++] = ((Buttons[0] & 0xff00) >>8);
	currentreport->data[i++] = ((Buttons[0] & 0xff0000) >>16);
	currentreport->data[i++] = ((Buttons[0] & 0xff000000) >>24);  // 32
}

void USBGameController::fillRestOfReport(int &i) {
	currentreport->data[i++] = 0;
	currentreport->data[i++] = 0;
	currentreport->data[i++] = 0;
	currentreport->data[i++] = 0; // 128
}









#define clipping_led_threshold 16300
void cFFBDevice::setClippingLed(s32 inputValue) {
	bool clippingLedOn = abs(inputValue) > clipping_led_threshold;
	if(clippingLedOn) {
		HAL_GPIO_WritePin(LED1_CLIPPING_OUT_GPIO_Port, LED1_CLIPPING_OUT_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(LED1_CLIPPING_OUT_GPIO_Port, LED1_CLIPPING_OUT_Pin, GPIO_PIN_RESET);
	}
}

void cFFBDevice::setEndstopLed(bool high) {
	if(high) {
		HAL_GPIO_WritePin(LED3_OUT_GPIO_Port, LED3_OUT_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(LED3_OUT_GPIO_Port, LED3_OUT_Pin, GPIO_PIN_RESET);
	}
}


void cFFBDevice::toggleEndstopLed() {
	HAL_GPIO_TogglePin(LED3_OUT_GPIO_Port, LED3_OUT_Pin);
}









#ifdef __cplusplus
extern "C" {
#endif
extern void Error_Handler(void);
#ifdef __cplusplus
}
#endif

void readHwVersion() {

	uint8_t id = 0;
	if(HAL_GPIO_ReadPin(IDSEL_0_GPIO_Port,IDSEL_0_Pin)==GPIO_PIN_RESET) {
		id=id+1;
	}
	if(HAL_GPIO_ReadPin(IDSEL_1_GPIO_Port,IDSEL_1_Pin)==GPIO_PIN_RESET) {
		id=id+2;
	}
	if(HAL_GPIO_ReadPin(IDSEL_2_GPIO_Port,IDSEL_2_Pin)==GPIO_PIN_RESET) {
		id=id+4;
	}
	switch(id) {
	case 0:
		joystick.gFFBDevice.scHWVersion = hwv1;
		break;
	case 1:
		joystick.gFFBDevice.scHWVersion = hwv2;
		break;
	default:
		joystick.gFFBDevice.scHWVersion = hwunknown;
		break;
	}
}

void setPWMDIRPinslow() {
	HAL_GPIO_WritePin(STM_IONI_PWM_OUT_HSIN2_GPIO_Port, STM_IONI_PWM_OUT_HSIN2_Pin, GPIO_PIN_RESET); // sets pin low
	HAL_GPIO_WritePin(STM_IONI_DIR_OUT_HSIN1_GPIO_Port, STM_IONI_DIR_OUT_HSIN1_Pin, GPIO_PIN_RESET); // sets pin low
}

void initBLEUART() {
	//static void MX_USART1_UART_Init_BLE(void)
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_8;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
}

extern UART_HandleTypeDef *debugHuart;
extern UART_HandleTypeDef huart4;
void testBLE() {
	joystick.gFFBDevice.bledetected=0;
	// Init huart1 for BLE
	initBLEUART();

	/* Check if BLE device exists */
	HAL_UART_Receive_DMA(&huart1, (uint8*)joystick.gFFBDevice.BLEConn.rxBuffer, BLERXBUFSIZE);
	// need quite a delay from cold boot, to reliably start the ble device.
	HAL_Delay(2000);
	joystick.gFFBDevice.BLEConn.init();
	resetUSART1vars();

	for (int i = 0; i < 100; ++i) {
		joystick.gFFBDevice.BLEConn.loop();
		if (joystick.gFFBDevice.BLEConn.appBooted) break;
		HAL_Delay(2);
	}

	if (joystick.gFFBDevice.BLEConn.appBooted) {
#ifdef UART4DEBUG
		MX_UART4_Init_Debug(); // Init uart4 for debug prints
		debugHuart = &huart4;
#else
		debugHuart = 0;
#endif
		printf("\r\nBLE Device found, NCP version %d.%d.%d-%d\r\n",joystick.gFFBDevice.BLEConn.NCPMajorVersion,joystick.gFFBDevice.BLEConn.NCPMinorVersion,joystick.gFFBDevice.BLEConn.NCPPatchVersion,joystick.gFFBDevice.BLEConn.NCPBuildVersion);
		joystick.gFFBDevice.bledetected=1;
	} else {
		HAL_UART_DeInit(&huart1);
		MX_USART1_UART_Init_Debug(); // Init uart 1 for debug prints
		debugHuart = &huart1;
		printf("BLE Device not found\r\n");
	}
}

void testAndInitBLE() {
	// sc1 needs test if found or not
	testBLE();
}


void resetUSART1vars() {
	int counter = DMA2_Stream2->NDTR;
	int dmaCounter = BLERXBUFSIZE - counter; //counts downwards as data is received
	joystick.gFFBDevice.BLEConn.rxBufPos=dmaCounter;
}

/* USART1 init function */
void MX_USART1_UART_Init_Debug()
{

	  huart1.Instance = USART1;
	  huart1.Init.BaudRate = 230400;
	  huart1.Init.WordLength = UART_WORDLENGTH_8B;
	  huart1.Init.StopBits = UART_STOPBITS_1;
	  huart1.Init.Parity = UART_PARITY_NONE;
	  huart1.Init.Mode = UART_MODE_TX_RX;
	  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	  huart1.Init.OverSampling = UART_OVERSAMPLING_8;
	  if (HAL_UART_Init(&huart1) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* UART4 init function */
void MX_UART4_Init_Debug(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 230400;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
}


extern ADC_HandleTypeDef hadc1;
/* ADC1 init function */
void MX_ADC1_Init(void)
{

	ADC_ChannelConfTypeDef sConfig;

	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	*/
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_LEFT;
	hadc1.Init.NbrOfConversion = NUM_ADC_CHANNELS;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
	Error_Handler();
	}

	// X11 UPPER

	// x11 upper 1
	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}

	// x11 upper 2
	sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = 2;
	sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}
	// x11 upper 3
	sConfig.Channel = ADC_CHANNEL_15;
	sConfig.Rank = 3;
	sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}
	// x11 upper 5
	sConfig.Channel = ADC_CHANNEL_14;
	sConfig.Rank = 4;
	sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}
	// x11 upper 6
	sConfig.Channel = ADC_CHANNEL_7;
	sConfig.Rank = 5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}


	// X11 LOWER

	// x11 lower 2
	sConfig.Channel = ADC_CHANNEL_13;
	sConfig.Rank = 6;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}
	// x11 lower 3
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = 7;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}
	// x11 lower 5
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = 8;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}
	// x11 lower 6
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = 9;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}
	// x11 lower 7
	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = 10;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}


	// EXT POT

	// EXT POT 1
	sConfig.Channel = ADC_CHANNEL_12;
	sConfig.Rank = 11;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}

	// EXT POT 2
	sConfig.Channel = ADC_CHANNEL_11;
	sConfig.Rank = 12;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}

	// EXT POT 3
	sConfig.Channel = ADC_CHANNEL_10;
	sConfig.Rank = 13;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}
}

void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : X12_LOWER_1_Pin X12_LOWER_2_Pin X12_LOWER_4_Pin X12_UPPER_6_Pin
                           STM_PE6_Pin STM_PE7_Pin STM_PE8_Pin STM_PE10_Pin
                           STM_PE13_Pin STM_PE14_Pin STM_PE15_Pin X15_3_Pin
                           X12_LOWER_3_Pin */
  GPIO_InitStruct.Pin = X12_LOWER_1_Pin|X12_LOWER_2_Pin|X12_LOWER_4_Pin|X12_UPPER_6_Pin
                          |STM_PE6_Pin|STM_PE7_Pin|STM_PE8_Pin|STM_PE10_Pin
                          |STM_PE13_Pin|STM_PE14_Pin|STM_PE15_Pin|X15_3_Pin
                          |X12_LOWER_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : X12_LOWER_6_Pin X12_LOWER_7_Pin X11_LOWER_1_Pin X12_UPPER_2_Pin
                           X12_UPPER_3_Pin X12_UPPER_4_Pin X12_UPPER_5_Pin X12_UPPER_7_Pin
                           X12_UPPER_1_Pin X12_LOWER_5_Pin */
  GPIO_InitStruct.Pin = X12_LOWER_6_Pin|X12_LOWER_7_Pin|X11_LOWER_1_Pin|X12_UPPER_2_Pin
                          |X12_UPPER_3_Pin|X12_UPPER_4_Pin|X12_UPPER_5_Pin|X12_UPPER_7_Pin
                          |X12_UPPER_1_Pin|X12_LOWER_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : STM_ENC_IN_A_Pin STM_ENC_IN_B_Pin STM_ENC_IN_C_Pin PA8 */
  GPIO_InitStruct.Pin = STM_ENC_IN_A_Pin|STM_ENC_IN_B_Pin|STM_ENC_IN_C_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : GP01_STM_Pin PB12 PB13 PB14
                           PB15 DIPSW_2_Pin STM_PB9_Pin */
  GPIO_InitStruct.Pin = GP01_STM_Pin|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|DIPSW_2_Pin|STM_PB9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : STM_IONI_PWM_OUT_HSIN2_Pin STM_IONI_DIR_OUT_HSIN1_Pin  */
  GPIO_InitStruct.Pin = STM_IONI_PWM_OUT_HSIN2_Pin|STM_IONI_DIR_OUT_HSIN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : HX711_CLKOUT_Pin */
  GPIO_InitStruct.Pin = HX711_CLKOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : RS485_TXEN_STM_Pin LED4_OUT_Pin LED3_OUT_Pin LED2_OUT_Pin
                           LED1_CLIPPING_OUT_Pin */
  GPIO_InitStruct.Pin = RS485_TXEN_STM_Pin|LED4_OUT_Pin|LED3_OUT_Pin|LED2_OUT_Pin
                          |LED1_CLIPPING_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : STM_PD9_Pin STM_PD10_Pin STM_PD11_Pin IDSEL_0_Pin
                           IDSEL_1_Pin IDSEL_2_Pin FTDI_USB_SLEEP_IN_Pin USB_GRANITY_VCC_SENSE_Pin
                           USB_HID_VCC_SENSE_Pin STM_PD6_Pin STM_PD7_Pin */
  GPIO_InitStruct.Pin = STM_PD9_Pin|STM_PD10_Pin|STM_PD11_Pin|
                          FTDI_USB_SLEEP_IN_Pin|USB_GRANITY_VCC_SENSE_Pin
                          |USB_HID_VCC_SENSE_Pin|STM_PD6_Pin|STM_PD7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* Configure GPIO IDSEL pins with Internal Pull-up */
  GPIO_InitStruct.Pin = IDSEL_0_Pin | IDSEL_1_Pin | IDSEL_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : GPI4_CLEAR_FAULTS_Pin PB5 */
  GPIO_InitStruct.Pin = GPI4_CLEAR_FAULTS_Pin|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, STM_IONI_PWM_OUT_HSIN2_Pin|STM_IONI_DIR_OUT_HSIN1_Pin|HX711_CLKOUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RS485_TXEN_STM_Pin|LED4_OUT_Pin|LED3_OUT_Pin|LED2_OUT_Pin
                          |LED1_CLIPPING_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPI4_CLEAR_FAULTS_Pin|GPIO_PIN_5, GPIO_PIN_RESET);

}

int init_test() {
	return 1;
}

void resetSettings() {
	joystick.gFFBDevice.SetDefault();
	joystick.gFFBDevice.mConfig.hardwareConfig.hwSettings[addrVariousSettingsBits1]&=~(1<<bitInitialConfigDone);
	if(!joystick.gFFBDevice.saveConfigsToFlash()) {
		currentSystemStatus = FlashFault;
	} else {
		// reboot MCU here
		enableSMWatchdog();
		simucube_bootloader(0xb007b007);
	}
}

void configureSystem() {
	return;
}

#ifdef __cplusplus
extern "C" {
#endif
void printfTransmit(char* ptr) {
	if (debugHuart) HAL_UART_Transmit(debugHuart, (uint8_t *)ptr, 1, 0xFFFF);
}
#ifdef __cplusplus
}
#endif


void setSTO(int reason) {
	// SC does not support STO setting
	return;
}

void disableSTO() {
	// SC does not support STO setting
	joystick.gFFBDevice.STOStatus=0;
}

void cFFBDevice::doLeds() {
	// SC does not support RGB led
	// HAL_GPIO_TogglePin(LED1_CLIPPING_OUT_GPIO_Port, LED1_CLIPPING_OUT_Pin);
}


void cFFBDevice::ledYellow() {
}

void cFFBDevice::ledBlue() {
}

void cFFBDevice::ledRed() {
}

void cFFBDevice::ledGreen() {
}

void cFFBDevice::ledHighTorque() {

}
void cFFBDevice::ledOff() {

}

void postInitDrive() {
    joystick.gFFBDevice.driveParamTracker.reset();
    joystick.gFFBDevice.driveParamTracker.addTrackedParameter(SMP_TORQUE_LPF_BANDWIDTH, addrIoniLPF);
    joystick.gFFBDevice.driveParamTracker.addTrackedParameter(SMP_TORQUE_NOTCH_FILTER, addrIoniNotch);
    joystick.gFFBDevice.driveParamTracker.addTrackedParameter(SMP_TORQUE_EFFECT_DAMPING, addrIoniDamping);
    joystick.gFFBDevice.driveParamTracker.addTrackedParameter(SMP_TORQUE_EFFECT_FRICTION, addrIoniFriction);
    joystick.gFFBDevice.driveParamTracker.addTrackedParameter(SMP_TORQUE_EFFECT_INERTIA, addrIoniInertia);
    joystick.gFFBDevice.driveParamTracker.addTrackedParameter(SMP_SETPOINT_FILTER_MODE, addrIoniFilter1);
    joystick.gFFBDevice.driveParamTracker.addTrackedParameter(SMP_TORQUE_EFFECT_STATIC_TORQUE_REDUCTION, addrStaticForceReduction);

}

//enum ledcolor {OFF, RED, GREEN, BLUE, YELLOW, BLUEREDBLINK};
void cFFBDevice::startLed(ledcolor setcolor) {
	switch(setcolor) {
	case OFF:
	case BLUE:
	case RED:
	case GREEN:
	case YELLOW:
		for(int i=0; i<8; i++) {
			ledCycle[i]=setcolor;
		}
		break;
	case BLUEREDBLINK:
		ledCycle[0]=BLUE;
		ledCycle[1]=RED;
		ledCycle[2]=BLUE;
		ledCycle[3]=RED;
		ledCycle[4]=BLUE;
		ledCycle[5]=BLUE;
		ledCycle[6]=BLUE;
		ledCycle[7]=BLUE;
		break;
	case BLUEBLANKBLINK:
		ledCycle[0]=OFF;
		ledCycle[1]=OFF;
		ledCycle[2]=OFF;
		ledCycle[3]=OFF;
		ledCycle[4]=BLUE;
		ledCycle[5]=BLUE;
		ledCycle[6]=BLUE;
		ledCycle[7]=BLUE;
	default:
		break;
	}
}


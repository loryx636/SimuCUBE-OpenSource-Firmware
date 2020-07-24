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
 *
 *
 *
 * ---------------------------------------------------------------------------
*/


#ifndef INPUT_DEVICES_H_
#define INPUT_DEVICES_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include "input.h"

class InputDevices {

	static const int INPUT_ARRAY_LENGTH = 28;
	Input inputs[INPUT_ARRAY_LENGTH];

	uint8_t amountOfUsedPins = 0;
	uint64_t buttonStates = 0;

	uint8_t shiftButtonIndex = -1;
	int8_t torqueButtonIndex = -1;

	int8_t torqueButtonState = 0; // -1 = torque off, 0 = no torque button, 1 = torque on
	uint8_t paddlesPressedOver5s = 0;

	uint8_t deviceCounter = 0;


	public:
	InputDevices();
	~InputDevices();

	void clearConfiguration();
	void setConfiguration(uint8_t *inputConfiguration);
	void process(uint32_t pinStates); // Process pin states
	uint64_t getButtonStates(); // Return states processed by process()
	uint8_t getAmountOfUsedPins();
	int8_t getTorqueButtonState();
	bool getPaddlesPressedOver5s();
};

#ifdef __cplusplus
};
#endif

#endif /* INPUT_DEVICES_H_ */

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


#ifndef INPUT_H_
#define INPUT_H_

#define DEVICE_TYPE_NOTHING 0

// Buttons (1 bit output)
#define DEVICE_TYPE_PADDLE 1
#define DEVICE_TYPE_BUTTON 2
#define DEVICE_TYPE_SHIFT_BUTTON 3
#define DEVICE_TYPE_TORQUE_OFF_BUTTON 4
#define DEVICE_TYPE_BUTTON_IGNORE_THIS_BY_THAT 5
#define DEVICE_TYPE_BUTTON_IGNORE_THAT_BY_THIS 6

// Encoders (2 bit output)
#define DEVICE_TYPE_ENCODER_1 10
#define DEVICE_TYPE_ENCODER_2 11
#define DEVICE_TYPE_ENCODER_4 12

#define ENCODER_OUTPUT_DELAY 20


#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

extern uint64_t millis; // milliseconds from the system start

class Input {

	uint8_t deviceType = DEVICE_TYPE_NOTHING;
 	uint8_t inputPin = 0;
	uint8_t amountOfOutputBits = 0;
 	int8_t stateChangesPerDetent = 1;
 	uint8_t extraData = 0;

 	uint32_t state, lastState = 0; // Current and previous input states

 	int8_t pulseCounter = 0; // Use the amount of pulses per detent when calculating outputs

	uint8_t outputValue = 0; // The output value of an input object
	uint8_t lastOutputValue = 0; // The previous output value of an input object
	uint64_t outputChanged = 0; // Timestamp when the output has changed

	bool ignored = false; // This can be set true by setIgnoredOnce(). It is set false every time when calling process()

	void setAmountOfOutputBits(uint8_t amount);
 	int8_t get_encoder_direction(uint8_t value, uint8_t last_value);
	void setOutputValue(uint8_t value);

	static const int8_t getEncoderDirection(uint8_t value, uint8_t last_value);

public:

	Input() {}
	~Input() {}

 	void setupPins(uint8_t _pin1, uint8_t _pin2);
 	void process(uint32_t states);

	uint8_t getOutputValue();
	uint64_t getOutputChangedMillis();
	uint8_t getAmountOfOutputBits();

	void setInputPin(uint8_t pin);
	uint8_t getInputPin();

	void setDeviceType(uint8_t type);
	uint8_t getDeviceType();

	void setExtraData(uint8_t data);
	uint8_t getExtraData();

	void setIgnoredOnce();
	bool getIgnored();
};

#ifdef __cplusplus
};
#endif

#endif /* INPUT_H_ */

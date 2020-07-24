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

#ifndef ENCODERANGLE_H_
#define ENCODERANGLE_H_

#include <cstdint>

#include "simplemotioncomms.h"

class cFFBDevice;

class encoderAngle {
public:
	encoderAngle();
	~encoderAngle();


	// gets minsteerningangle to maxsteeringangle limited angle in degrees as float
	float limited_steering_angle(float minSteeringAngle, float maxSteeringAngle, float steeringAngleUnlimited);

	// centering/indexing mode set
	void setCenteringMode(int32_t mode);

	// centering/indexing mode readback
	int32_t getCenteringMode();
	// updates internal steering angle calculations
	void calcSteeringAngle(const TorqueResponse& tr);

	// gets current encoder counter value (unwrapped if needed)
	int32_t getCurrentEncoderValue();

	// gets last calculated unlimited steering angle in float
	float getUnlimitedSteeringAngle();

	// set pointer to cFFBDevice object
	void setDevice(cFFBDevice* device);

	// read start position from servo drive.
	bool readStartPosition();

	// resets internal offset for index point indexing
	void setIndexpointOffset(int32_t offset);

	// set CPR to the object.
	void setCPR(uint32_t cpr);

	// get CPR value
	int32_t getCPR();

	// reset internal parameters
	void reset();

	// set angle reported to USB to angle.
	void setUSBAngle(uint16_t angle);

	// get raw angle in USB pos, 0..65535
	uint16_t getUSBAngle();

	// get filtered angle in USB pos, 0..65535
	int32_t getAxisPos();

	// get min and max steering angles
	float getMinSteeringAngle();
	float getMaxSteeringAngle();

	// set min and max steering angles based on one lock-to-lock angle
	void setMinMaxSteeringAngles(float angle);

private:
	int32_t _turnCounter;
	int32_t _previous;
	int32_t _current;
	float _steeringAngleUnlimited; // unlimited angles

	uint32_t _centeringMode;

	int32_t _indexpointEncOffset;

	int32_t _cpr;
	float _cprFloat;
	cFFBDevice* _ffbdevice;
	bool _reset;

	// minimum and maximum steering angle in degrees. E.g, -900.0 - 900.0
	float _minSteeringAngle;
	float _maxSteeringAngle;

	// calculates and returns -inf - +inf steering angle in degrees as float
	float calculate_unlimited_steering_angle(const int32_t& position);

	// sample timestamp
	uint8_t _last_encoder_sampled_at;

	float speed_estimate(float distance, const uint8_t& encoder_sampled_at, float& duration);
	int32_t unwrap_partial(int32_t& accumulator, int32_t& previous, int32_t current);

	// axis position
	int32_t _axisPos; // 0..65535, see CalcSteeringAngle
	float _axisPosFloat; // 0f..65535f, see CalcSteeringAngle
	uint16_t _usb_angle; // filtered usb units per ms
};

#endif //ENCODERANGLE_H_

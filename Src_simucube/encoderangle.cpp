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

#include "encoderangle.h"
#include "stm32f4xx.h"
#define ARM_MATH_CM4
#include <math.h>
#include "arm_math.h"
#include "simplemotion.h"

#include "cFFBDevice.h"

#include "config_comm_defines.h"


#define CPR24BIT 16777216


encoderAngle::encoderAngle() {
	_turnCounter = 0;
	_current = 0;
	_previous = 0;
	_indexpointEncOffset = 0;
	_ffbdevice = nullptr;
	_cpr = CPR24BIT;
	_cprFloat = 16777216.0f;
	_last_encoder_sampled_at = 0;
	_steeringAngleUnlimited = 0.0f;
	_usb_angle = 0;
	_axisPos = 0;
	_axisPosFloat = 0.0f;
	_centeringMode = noIndexing;
}

encoderAngle::~encoderAngle() {

}

void encoderAngle::setCenteringMode(int32_t mode) {
	_centeringMode = mode;
}

int32_t encoderAngle::getCenteringMode() {
	return _centeringMode;
}

void encoderAngle::setDevice(cFFBDevice* device) {
	_ffbdevice = device;
}

int32_t encoderAngle::getCurrentEncoderValue() {
	return _current;
}

bool encoderAngle::readStartPosition() {
	return true;
}

void encoderAngle::setIndexpointOffset(int32_t offset) {
	_indexpointEncOffset = offset;
}

int32_t encoderAngle::getCPR() {
	return _cpr;
}

#define ANGLEDEBUG 0
#if ANGLEDEBUG
#define angleprintf(...) printf(__VA_ARGS__);
#else
#define angleprintf(...)
#endif




int32_t encoderAngle::unwrap_partial(int32_t& accumulator, int32_t& previous, int32_t encoderValue) {
	// on reset, set previous early so that the following ifs will be valid and hopefully not trigger
	int32_t latestEncoderValue = encoderValue;
	if(_centeringMode == indexPointIndexing) {
		latestEncoderValue -= _indexpointEncOffset;
		if(latestEncoderValue>CPR24BIT) {
			latestEncoderValue -= CPR24BIT;
			angleprintf("overflowing+\r\n");
		} else if(latestEncoderValue<0) {
			latestEncoderValue += CPR24BIT;
			angleprintf("overflowing-\r\n");
		}
	}
	if(_reset) {
		angleprintf("ANGLE: reset, read encoder value %ld\r\n", encoderValue);
		// wheel might have moved just to the left so that encoderValue is close to MAX(24BIT).
		// this needs to be handled so that the accumulator will be reset at 0 near the current position.
		if (latestEncoderValue > (3 * _cpr / 4)) {
			_turnCounter--;
			angleprintf("ANGLE: reset noise overlow\r\n");
		}
		previous = latestEncoderValue;
	}

	if((latestEncoderValue > (3 * _cpr / 4)) && (previous < (1 * _cpr  / 4))) {
		_turnCounter--;
		angleprintf("ANGLE: over 0 to the left, read encoder value %ld, reset=%d\r\n", latestEncoderValue, _reset);
	} else if( (latestEncoderValue < (1 * _cpr / 4)) && (previous > (3 * _cpr / 4))) {
		_turnCounter++;
		angleprintf("ANGLE: over 0 to the right, read encoder value %ld, reset=%d\r\n", latestEncoderValue, _reset);
	}

	previous = latestEncoderValue;
	accumulator = _cpr*_turnCounter + latestEncoderValue;
	angleprintf("%ld, %ld\r\n", _centeringMode, accumulator);
	return accumulator;
}

float encoderAngle::calculate_unlimited_steering_angle(const int32_t& position) {
	return 360.0f * (float)(position) / _cprFloat;
}

/// Limits and converts the steering angle into an "USB angle" which is inside 0..65535f.
float encoderAngle::limited_steering_angle(float minSteeringAngle, float maxSteeringAngle, float steeringAngleUnlimited) {
	const auto steeringAngle = fmax(minSteeringAngle, fmin(maxSteeringAngle, steeringAngleUnlimited));

	const auto angle_percent = steeringAngle / maxSteeringAngle;

	return fmax(0.0f, fmin(65535.0f, (angle_percent * 32768.0f) + 32768.0f));
}

void encoderAngle::setCPR(uint32_t CPR) {
	_cpr = CPR;
	_cprFloat = (float)CPR;
}

void encoderAngle::reset() {
	_turnCounter = 0;
	_previous = 0;
	_current = 0;
	_reset = true;
}

// for SWO tracing for the difference
uint32_t latency_comp_abs_difference = 0;

void encoderAngle::calcSteeringAngle(const TorqueResponse& tr) {
	if (_reset) {
		// tricky part: we have stateful components as part of our calculations
		_last_encoder_sampled_at = tr.encoder_sampled_at;

		// encoderCounter is a counter which is reset to a value with reset_position
		// previousEncoderPos is used as a delta to tr.position so we reset here now that
		// we have the latest tr
		_previous = tr.position;
	}

	// value inside TorqueResponse is expected to be 24 bit so we need to unwrap it
	// note that this will modify the two first arguments
	const int32_t current_position = unwrap_partial(_current, _previous, tr.position);

	const auto steering_angle_unlimited = calculate_unlimited_steering_angle(current_position);

	// originally the time between updates was calculated using a onboard timer peripheral. later the ALT3
	// mode was added for simplemotion which meant that we can use the 6-bit timestamps on each SetTorque
	// know how much in time our two encoder readings (or any angle derived from them) differ.
	// this variable should contain a duration which is invariant on any ble/uart communication and usb hiccups.
	// precision was at the time of writing this 400 us.

	float millis_from_timestamps = 0.0f;

	const float raw_angles_per_millis = speed_estimate(
		steering_angle_unlimited - _steeringAngleUnlimited,
		tr.encoder_sampled_at,
		millis_from_timestamps);

	const float steering = limited_steering_angle(_minSteeringAngle, _maxSteeringAngle, steering_angle_unlimited);

	_usb_angle = (uint16_t) steering;

	// For some it might make more sense to have speed in angles per second.
	// The unit here will be "DirectX forcefeedback positions" per millisecond.
	// This is to ease calculating the damping effect and endstop damping in the spirit of the remarks of
	// struct DICONDITION MSDN documentation (now outdated).
	float speed;

	if (_reset) {
		speed = 0.0f;
	} else {
		// 400 us should be the minimum, it should be around 800 us typically
		if (fabs(millis_from_timestamps) >= 0.400f) {

			// instead of using the steering and this->axisPosFloat use the unlimited angle difference as this
			// allows tracking speed and producing damping even outside the maximum steering range.
			// scale the unlimited values to axis counts; offset 32768 is not needed as it subtracts away from the difference.
			float a = (steering_angle_unlimited / _maxSteeringAngle) * 32768.0f;
			float b = (_steeringAngleUnlimited / _maxSteeringAngle) * 32768.0f;

			speed = (a - b) / millis_from_timestamps;
		} else {
			speed = 0.0f;
		}
	}

	// rest of member variables are consumed by effects

	_ffbdevice->axisSpeedPerMs = _ffbdevice->axisSpeedMsLPF.process(speed);
	_steeringAngleUnlimited = steering_angle_unlimited;
	_axisPos = (uint32_t) steering;
	_axisPosFloat = steering;

	{
		// FIXME: at least the axisSpeedPerMs can be moved into torque calc
		_ffbdevice->endstop_effect.update(steering_angle_unlimited, this);
		_ffbdevice->setEndstopLed(_ffbdevice->endstop_effect.direction() != 0);
	}

	// no early exits please. I couldn't figure out a more clear way to have the reset request information available
	// while at the same time making sure reset happens always.
	_reset = false;
}

float encoderAngle::getUnlimitedSteeringAngle() {
	return _steeringAngleUnlimited;
}

float encoderAngle::speed_estimate(float distance, const uint8_t& encoder_sampled_at, float& duration) {

	constexpr float micros_to_millis = 1000.0f;
	constexpr float timestamp_step = 400.0f /* us */;

	// typical values 2..3
	const uint8_t delta = (encoder_sampled_at - _last_encoder_sampled_at) & ((1 << (6 - 1)) - 1);

	duration = (timestamp_step * (float)delta) / micros_to_millis;

	_last_encoder_sampled_at = encoder_sampled_at;

	if (delta != 0) {
		return distance / duration;
	}

	return 0;
}

float encoderAngle::getMinSteeringAngle() {
	return _minSteeringAngle;
}


float encoderAngle::getMaxSteeringAngle() {
	return _maxSteeringAngle;
}

void encoderAngle::setUSBAngle(uint16_t angle) {
	_usb_angle = angle;
	_axisPos = angle;
	_axisPosFloat = (float)angle;
}

uint16_t encoderAngle::getUSBAngle() {
	return _usb_angle;
}

int32_t encoderAngle::getAxisPos() {
	return _axisPos;
}

void encoderAngle::setMinMaxSteeringAngles(float angle) {
	_maxSteeringAngle = angle/2.0f;
	_minSteeringAngle = -_maxSteeringAngle;
}

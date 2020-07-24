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

#include "swwautodisconnecthandler.h"
#include "encoderangle.h"
#include "bleconnection.h"
#include "cFFBDevice.h"

extern uint64_t millis;

#define IDLEDEBUG 0
#if IDLEDEBUG
#define idleprintf(...) printf(__VA_ARGS__);
#else
#define idleprintf(...)
#endif

SWWAutoDisconnectHandler::SWWAutoDisconnectHandler() {
	_angle = nullptr;
	_conn = nullptr;
	_dev = nullptr;
	_lastAngle = 0.0f;
	_lastRun = 0;
	_idleStartTime = 0;
}

SWWAutoDisconnectHandler::~SWWAutoDisconnectHandler() {

}

void SWWAutoDisconnectHandler::init(encoderAngle* angle, BLEConnection* connection, cFFBDevice* dev) {
	_angle = angle;
	_conn = connection;
	_dev = dev;
	_lastAngle = _angle->getUnlimitedSteeringAngle();
	_lastRun = millis;
	_idleStartTime = millis;
}

void SWWAutoDisconnectHandler::reset() {

	_lastAngle = _angle->getUnlimitedSteeringAngle();
	_idleStartTime = millis;
	idleprintf("reset!\r\n");
}

bool SWWAutoDisconnectHandler::run() {
	if(!_conn->getConnectionState()) {
		// not connected
		reset();
		idleprintf("state:disc\r\n");
		return true;
	}
	// do checks at _interval defined periods only
	if(abs((uint32_t)millis - _lastRun) < _interval) {

		return true;
	}
	idleprintf("interval\r\n");
	_lastRun = (uint32_t) millis;



	// check if there are FFB effecs running.
	if(_dev->haveEffects()) {
		// effects in use = game running.
		_idleStartTime = millis;
		idleprintf("effects!\r\n");
		return true;
	}
	// check if wheel has turned enough.
	if(abs(_angle->getUnlimitedSteeringAngle() - _lastAngle) > _deadZone) {
		_lastAngle = _angle->getUnlimitedSteeringAngle();
		_idleStartTime = millis;
		idleprintf("wheel has turned!\r\n");
		return true;
	}

	// check if idled long enough
	if(abs((uint32_t)millis - _idleStartTime) < _timeout) {
		// still idling
		idleprintf("idling\r\n");
		return true;
	}

	// time to act
	idleprintf("disconnect\r\n");
	_conn->disconnectFromDevice();
	_dev->setSWWIdleDisconnectStatus(1);
	reset();
	return false;
}

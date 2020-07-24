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

#ifndef SWWAUTODISCONNECTHANDLER_H_
#define SWWAUTODISCONNECTHANDLER_H_

#include <cstdint>

class encoderAngle;
class BLEConnection;
class cFFBDevice;

class SWWAutoDisconnectHandler {
public:
	SWWAutoDisconnectHandler();
	~SWWAutoDisconnectHandler();

	void init(encoderAngle* angle, BLEConnection* connection, cFFBDevice* dev);

	/* returns true when wheel is OK, false when idle disconnect was done. */
	bool run();



private:
	/* to be called when no wheel connected. */
	void reset();

	encoderAngle* _angle;
	BLEConnection* _conn;
	cFFBDevice* _dev;
	float _lastAngle;
	uint32_t _lastRun;
	uint32_t _idleStartTime;
	const uint32_t _interval = 10000; // update 10s
	const uint32_t _timeout = 3600000; // 1h
	const float _deadZone = 10.0f;
};



#endif /* SWWAUTODISCONNECTHANDLER_H_ */

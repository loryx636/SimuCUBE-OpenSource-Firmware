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

#ifndef DRIVEPARAMETERTRACKER_H_
#define DRIVEPARAMETERTRACKER_H_

#include <cstdint>

#define MAXPARAMETERS 15

class DriveParameterTracker {
public:
	DriveParameterTracker();
	~DriveParameterTracker();
	void reset();
	void updateParameters(bool forced);
	void addTrackedParameter(uint32_t SMAddr, uint32_t profileParameterAddr);
private:
	int32_t lastParameters[MAXPARAMETERS];
	uint16_t parameterAddrs[MAXPARAMETERS];
	uint32_t profileAddrs[MAXPARAMETERS];
	int numberOfParameters;
	bool setOnce;
};

#endif /* DRIVEPARAMETERTRACKER_H_ */

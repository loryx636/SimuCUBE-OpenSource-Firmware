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

#ifndef CDEVICECONFIG_H_
#define CDEVICECONFIG_H_

#include "cHardwareConfig.h"
#include "cProfileConfig.h"
#include "cAnalogConfig.h"
#include "cButtonConfig.h"
#include <cstring>



class cDeviceConfig {
public:
	cDeviceConfig();
	virtual ~cDeviceConfig();

	void SetDefault();

	void setReadOnlyDefaultProfileValues();
	cHardwareConfig hardwareConfig;
	cAnalogConfig analogConfig[maxnumanalogconfigs];
	cProfileConfig profileConfigs[maxnumprofiles_v11];
	cButtonConfig buttonConfig;


	uint8_t * GetProfileConfigAddr(uint16_t profileindex);
	uint8_t * GetHardwareConfigAddr();
	uint8_t * GetAnalogConfigAddr();

	void getProfile(int *conf, uint16_t profileindex);
	void setProfile(int *conf, uint16_t profileindex);
	void getHardware(int *conf);
	void setHardware(int *conf);
	void getAnalog(int *conf);
	void setAnalog(int *conf);

	void replaceOutOfRangeWithDefaultProfileParameter(int32_t& parameter, int32_t low, int32_t high, int32_t def);
	void validateProfile(uint32_t profileindex);
private:


};

#endif /* CDEVICECONFIG_H_ */

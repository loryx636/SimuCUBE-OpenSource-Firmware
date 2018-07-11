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
 * Etienne Saint-Paul
 *
 *
 * ---------------------------------------------------------------------------
*/

/*
 * cDeviceConfig.cpp
 *
 *  Created on: Jan 30, 2017
 *      Author: Mika
 */

#include <cDeviceConfig.h>

cDeviceConfig::cDeviceConfig() {
	// Auto-generated constructor stub
	SetDefault();
}

cDeviceConfig::~cDeviceConfig() {
	// Auto-generated destructor stub
}

void cDeviceConfig::SetDefault() {
	for(int i=0; i<maxnumprofiles; i++) {
		profileConfigs[i].SetDefault();
	}
	const char* defaulttext = "Read-only safe profile";
	memcpy(profileConfigs[0].profilename, defaulttext, 22);
	profileConfigs[0].profilename[22] = 0;
	profileConfigs[0].mMainGain=20;
	profileConfigs[0].ioni_lpf = 6;
	//profileConfig.SetDefault();
	hardwareConfig.SetDefault();
	analogConfig.setDefault();
}

void cDeviceConfig::setReadOnlyDefaultProfileValues() {
	profileConfigs[0].SetDefault();

	const char* defaulttext = "Read-only safe profile";
	memcpy(profileConfigs[0].profilename, defaulttext, 22);
	profileConfigs[0].profilename[22] = 0;
	profileConfigs[0].mMainGain=20;
	profileConfigs[0].ioni_lpf = 6;
}

uint8_t * cDeviceConfig::GetProfileConfigAddr(uint16_t profileindex) {
	return ((uint8_t*)&profileConfigs[profileindex].mMaxAngle);
}

uint8_t * cDeviceConfig::GetHardwareConfigAddr() {
	return ((uint8_t*)&hardwareConfig.mEncoderOffset);
}

uint8_t * cDeviceConfig::GetAnalogConfigAddr() {
	return ((uint8_t*)&analogConfig.Y);
	//return ((uint8_t*)&(analogConfig.Y.pin));
}

									// dest, src, size

// gets profile config data to conf
void cDeviceConfig::getProfile(int *conf, uint16_t profileindex) {
	memcpy(conf, GetProfileConfigAddr(profileindex), sizeof(cProfileConfig));
}

// sets profile config data from conf
void cDeviceConfig::setProfile(int *conf, uint16_t profileindex) {
	memcpy(GetProfileConfigAddr(profileindex), conf, sizeof(cProfileConfig));
}

// gets hardware config data to conf
void cDeviceConfig::getHardware(int *conf) {
	memcpy(conf, GetHardwareConfigAddr(), sizeof(cHardwareConfig));
}

// sets hardware config data from conf
void cDeviceConfig::setHardware(int *conf) {
	memcpy(GetHardwareConfigAddr(), conf, sizeof(cHardwareConfig));
}

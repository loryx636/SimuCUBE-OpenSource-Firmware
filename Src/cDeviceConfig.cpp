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

#include <cDeviceConfig.h>

cDeviceConfig::cDeviceConfig() {
	// Auto-generated constructor stub
	SetDefault();
}

cDeviceConfig::~cDeviceConfig() {
	// Auto-generated destructor stub
}

void cDeviceConfig::SetDefault() {
	for(int i=0; i<maxnumprofiles_v11; i++) {
		profileConfigs[i].SetDefault();
	}
	const char* defaulttext = "Read-only safe profile";
	memcpy(profileConfigs[0].profilename, defaulttext, 22);
	profileConfigs[0].profilename[22] = 0;
	profileConfigs[0].settings[addrMainGain]=20;
	profileConfigs[0].settings[addrIoniLPF] = 6;
	//profileConfig.SetDefault();
	hardwareConfig.SetDefault();
	for(int i = 0; i< maxnumanalogconfigs; i++){
		analogConfig[i].setDefault();
	}
	buttonConfig.setDefault();

}

void cDeviceConfig::setReadOnlyDefaultProfileValues() {
	profileConfigs[0].SetDefault();

	const char* defaulttext = "Read-only safe profile";
	memcpy(profileConfigs[0].profilename, defaulttext, 22);
	profileConfigs[0].profilename[22] = 0;

	/* set read-only default profile parameters to match
	 * one of the simple settings mode possibilities
	 * (average filtering, average smoothness)
	 */

	profileConfigs[0].settings[addrMainGain]=20;
	profileConfigs[0].settings[addrIoniLPF] = 8;

	profileConfigs[0].settings[addrFrictionGain]=8;
	profileConfigs[0].settings[addrDamperGain]=8;
	profileConfigs[0].settings[addrInertiaGain]=100;

	profileConfigs[0].settings[addrIoniDamping]=81;
	profileConfigs[0].settings[addrIoniFriction]=59;
	profileConfigs[0].settings[addrIoniInertia]=99;
	profileConfigs[0].settings[addrIoniFilter1]=5;

}


void cDeviceConfig::replaceOutOfRangeWithDefaultProfileParameter(int32_t& parameter, int32_t low, int32_t high, int32_t def) {
	if(parameter<low) parameter=def;
	if(parameter>high) parameter=def;
}

void cDeviceConfig::validateProfile(uint32_t profileindex) {

}


uint8_t * cDeviceConfig::GetProfileConfigAddr(uint16_t profileindex) {
	return ((uint8_t*)&profileConfigs[profileindex].settings[addrMaxAngle]);
}

uint8_t * cDeviceConfig::GetHardwareConfigAddr() {
	return ((uint8_t*)&hardwareConfig.hwSettings[0]);
}

uint8_t * cDeviceConfig::GetAnalogConfigAddr() {
	return ((uint8_t*)&analogConfig[0].axises[0].settings[addrAnalogPin]);
}

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

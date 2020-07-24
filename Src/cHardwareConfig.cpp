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

#include "cHardwareConfig.h"

cHardwareConfig::cHardwareConfig() {
	// Auto-generated constructor stub
	SetDefault();
}

cHardwareConfig::~cHardwareConfig() {
	// Auto-generated destructor stub
}

void cHardwareConfig::SetDefault() {
	// reset everything to zero here.
#if 0 // defaults for 0.10 and earlier with old settings data format
	mEncoderOffset = 0;
	mEncoderCPR=0;

	mIndexingMode = startAtCenterPhasedIndexing;
	mDesktopSpringGain = 0;
	mDesktopDamperGain = 0;
	mDesktopAutoCenter = 0;
	mDesktopSpringSaturation = 0;


	mStopsDamperGain = 10;
	mStopsRangeDegrees = 10;
	mStopsEnabled = 1;
	mStopsMaxForce = 20;

	mInitialConfigDone = InitialConfigNotDone;
	invertSteeringValue = false;
	audibleNotificationsEnabled = true;
	buttonDebounceMillis = 10;
	mMaxMotorCurrent = 0;
	motorTorqueConstant=0;
	motorMR=0;
	usbResetsEnabled=1;
	autoCommutationStatus=Busy;
	mAutoCommutationMode=0;
	mAbsoluteEncoder=0;
	variousSettingsBits=0;
#endif
	for(int i = 0; i< numberOfHwSettingsAddrs; i++) {
		hwSettings[i]=0;
	}
	for(int j = 0; j<256; j++) {
		bytestore[j]=0;
	}
	hwSettings[addrButtonDebounceMs]=5;
	// the rest will be set at factory test bench or when user resets settings to default.
	// Some of the settings are device dependent.
	hwSettings[addrAutoConnectBLEDevice] = static_cast<int32_t>(ConnectAutomaticallyToWirelessWheel::ToPairedDevice);
}


void cHardwareConfig::convertDesktopSpring() {
	/* this is called if flash included <0.50.0 version desktop spring settings, where
	 * additional bit was used to toggle spring on/off  compared to 0.50 and up, where
	 * desktop spring is always on, and 0 means off/no torque.
	 */
	if((hwSettings[addrVariousSettingsBits1]&(1<<bitDesktopAutoCenterEnabled)) == 0) {
		// if tests against 0, so if not set, this is done:
		hwSettings[addrDesktopSpringGain]=0;
	}

}

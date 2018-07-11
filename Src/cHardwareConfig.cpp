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
 * cHardwareConfig.cpp
 *
 *  Created on: Jan 30, 2017
 *      Author: Mika
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
	// pointers to objects have to be deleted.
#if 0
	mEncoderOffset = 0;
	mEncoderCPR=0;
	mDesktopSpringGain = 0;
	mDesktopDamperGain = 0;
	mDesktopAutoCenter = 0;
	mStopsSpringGain = 0;
	mStopsFrictionGain = 0;
	mConfigDone = false;
	invertSteeringValue = false;
#endif
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
}


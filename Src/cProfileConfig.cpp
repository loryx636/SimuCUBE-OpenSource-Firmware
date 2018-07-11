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
 * cProfileConfig.cpp
 *
 *  Created on: Feb 1, 2017
 *      Author: Mika
 */

#include "cProfileConfig.h"

cProfileConfig::cProfileConfig() {
	// Auto-generated constructor stub
	SetDefault();
}

cProfileConfig::~cProfileConfig() {
	// Auto-generated destructor stub
}

void cProfileConfig::SetDefault() {
	// reset everything to zero here.
	// pointers to objects have to be deleted.

	// remember: percentages.
	mMaxAngle = 900;
	mSineGain = 100;
	mSquareGain = 100;
	mMainGain = 60;
	mSpringGain = 100;
	mFrictionGain = 100;
	mDamperGain = 100;
	mInertiaGain = 0;
	endstopOffsetAngleDegrees = 0;
	ioni_lpf = 0;
	ioni_notch = 0;
	ioni_damping = 0;
	ioni_friction = 0;
	ioni_inertia = 0;
	ioni_filter1 = 0;
	filteringModes = 0;
	for(int i = 0; i<20; i++) {
		profilename[i]=0;
	}
	mSawtoothGain = 100;
	mTriangleGain = 100;
}

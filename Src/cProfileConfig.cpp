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


#include "cProfileConfig.h"
#include "config_comm_defines.h"

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
	for (size_t i = 0; i < sizeof(settings) / sizeof(settings[0]); ++i) {
		settings[i]=0;
	}
	settings[addrMaxAngle]=900;
	settings[addrSineGain]=100;
	settings[addrSquareGain]=100;
	settings[addrMainGain]=60;
	settings[addrSpringGain]=100;
	settings[addrFrictionGain]=15;
	settings[addrDamperGain]=15;
	settings[addrInertiaGain]=0;
	settings[addrEndstopOffset]=0;
	settings[addrIoniLPF]=0;
	settings[addrIoniNotch]=0;
	settings[addrIoniDamping]=0;
	settings[addrIoniFriction]=0;
	settings[addrIoniInertia]=0;
	settings[addrIoniFilter1]=1;
	settings[addrFilteringModes]=0;
	settings[addrSawtoothGain]=100;
	settings[addrTriangleGain]=100;
	for(int i = 0; i<32; i++) {
		profilename[i]=0;
		bytestore[i]=0;
	}
#if 0 // default settings for 0.10 and earlier firmware with old data formats
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
	for(int i = 0; i<32; i++) {
		profilename[i]=0;
	}
	mSawtoothGain = 100;
	mTriangleGain = 100;
#endif
}

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
 * cProfileConfig.h
 *
 *  Created on: Feb 1, 2017
 *      Author: Mika
 */

#ifndef CPROFILECONFIG_H_
#define CPROFILECONFIG_H_

#include "../SimpleMotion/simplemotion.h"
#include "types.h"

class cProfileConfig {
public:
	cProfileConfig();
	virtual ~cProfileConfig();
	void SetDefault();


	u16 mMaxAngle;
	u8 mSineGain;
	u8 mSquareGain;
	// in percentages!
	s8 mMainGain;
	u8 mSpringGain;
	u8 mFrictionGain;
	u8 mDamperGain;
	u8 mInertiaGain;
	s16 endstopOffsetAngleDegrees;
	u8 ioni_lpf;
	u32 ioni_notch;
	u16 ioni_damping;
	u16 ioni_friction;
	u16 ioni_inertia;
	u8 ioni_filter1;
	u16 filteringModes;
	unsigned char profilename[25];
	u8 mSawtoothGain;
	u8 mTriangleGain;
	u8 pad[3];
} __attribute__((packed));

#endif /* CPROFILECONFIG_H_ */

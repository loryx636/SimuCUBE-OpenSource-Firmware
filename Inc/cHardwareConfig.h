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
 *
 *
 *
 * ---------------------------------------------------------------------------
*/
#ifndef CHARDWARECONFIG_H_
#define CHARDWARECONFIG_H_

#include "types.h"
#include "simplemotion.h"
#include "config_comm_defines.h"


class cHardwareConfig {
public:
	cHardwareConfig();
	virtual ~cHardwareConfig();

	smint32 mEncoderOffset;

	smint32 mEncoderCPR;
	// analog axis things

    u8 mIndexingMode;
    u8 mDesktopSpringGain;
	u8 mDesktopDamperGain;
	b8 mDesktopAutoCenter;

	u8 mStopsEnabled;
	u8 mStopsDamperGain;
	u8 mStopsRangeDegrees;
	u8 mStopsMaxForce;

	u8 mInitialConfigDone;
	u8 invertSteeringValue;
	u8 audibleNotificationsEnabled;
	u8 buttonDebounceMillis;
	u16 mMaxMotorCurrent;
	u16 motorTorqueConstant;
	u16 motorMR;
	u8 mDesktopSpringSaturation;
	u8 usbResetsEnabled;
	s32 autoCommutationStatus;
	u8 mAutoCommutationMode;
	u8 mAbsoluteEncoder;
	u32 variousSettingsBits;
	u8 pad[23];

	void SetDefault();

private:
	uint16_t ScaleAnalogAxis(uint16_t raw, bool invert, uint16_t minvalue, uint16_t maxvalue);
} __attribute__((packed));

#endif /* CHARDWARECONFIG_H_ */

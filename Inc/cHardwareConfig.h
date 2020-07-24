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
#ifndef CHARDWARECONFIG_H_
#define CHARDWARECONFIG_H_

#include "types.h"
#include "config_comm_defines.h"
#include <cstdint>


class cHardwareConfig {
public:
	cHardwareConfig();
	virtual ~cHardwareConfig();

	int32_t hwSettings[numberOfHwSettingsAddrs];
	uint8_t bytestore[256];
	void SetDefault();
	void convertDesktopSpring();

private:
	uint16_t ScaleAnalogAxis(uint16_t raw, bool invert, uint16_t minvalue, uint16_t maxvalue);
};

#endif /* CHARDWARECONFIG_H_ */

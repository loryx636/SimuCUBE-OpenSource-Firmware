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

#ifndef CPROFILECONFIG_H_
#define CPROFILECONFIG_H_

#include "types.h"
#include <cstdint>
#include "config_comm_defines.h"

class cProfileConfig {
public:
	cProfileConfig();
	virtual ~cProfileConfig();
	void SetDefault();

	int32_t settings[numberOfProfAddrs];
    unsigned char profilename[32];
    u8 bytestore[32];

};

#endif /* CPROFILECONFIG_H_ */

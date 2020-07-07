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

#ifndef CBUTTONCONFIG_H_
#define CBUTTONCONFIG_H_

#include "config_comm_defines.h"

// button configs are not really used for anything at the moment.

class cButtonConfig {
public:
	cButtonConfig();
	virtual ~cButtonConfig();
	int32_t settings[numberofbuttonsettings];
	void setDefault();
};

#endif /* CBUTTONCONFIG_H_ */

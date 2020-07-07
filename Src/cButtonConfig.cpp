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

#include <cButtonConfig.h>

cButtonConfig::cButtonConfig() {
	// TODO Auto-generated constructor stub
	setDefault();
}

void cButtonConfig::setDefault() {
	for(int i = 0; i<numberofbuttonsettings; i++) {
		settings[i]=0;
	}
}
cButtonConfig::~cButtonConfig() {
	// TODO Auto-generated destructor stub
}


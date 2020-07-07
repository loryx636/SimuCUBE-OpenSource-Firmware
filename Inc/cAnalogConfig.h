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

#ifndef CANALOGCONFIG_H_
#define CANALOGCONFIG_H_

#include "config_comm_defines.h"
#include "main.h"
#include "Biquad1storder.h"

class cAnalogConfig {
public:
	cAnalogConfig();
	virtual ~cAnalogConfig();
	void setDefault();
	AnalogAxelNew axises[7];
	uint8_t bytestore[32];
	void updateAnalogAxises(bool rawmode, uint16_t &y, uint16_t &z, uint16_t &brake, uint16_t &throttle, uint16_t &clutch, uint16_t &rudder, uint16_t &t);

private:
	uint16_t scaleAnalogAxis(uint16_t raw, bool invert, uint16_t minvalue, uint16_t maxvalue);
	void updateAvgs(uint16_t (&avgs)[NUM_ADC_CHANNELS]);
	Biquad1StOrder inputfilters[7];
};

#endif /* CANALOGCONFIG_H_ */

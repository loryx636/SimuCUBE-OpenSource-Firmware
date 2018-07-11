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
/*
 * cAnalogConfig.h
 *
 *  Created on: Jun 7, 2017
 *      Author: Mika
 */

#ifndef CANALOGCONFIG_H_
#define CANALOGCONFIG_H_

#include "config_comm_defines.h"
#define NUM_ADC_CHANNELS 13
#define ADC_NUM_AVERAGING 32
#define ADC1_BUFFER_LENGTH NUM_ADC_CHANNELS*ADC_NUM_AVERAGING


class cAnalogConfig {
public:
	cAnalogConfig();
	virtual ~cAnalogConfig();
	void setDefault();
	//AnalogAxel X;
	AnalogAxel Y;
	AnalogAxel Z;
	AnalogAxel Brake;
	AnalogAxel Throttle;
	AnalogAxel Clutch;
	AnalogAxel Rudder;
	AnalogAxel T;

	void updateAnalogAxises(bool rawmode, uint16_t &y, uint16_t &z, uint16_t &brake, uint16_t &throttle, uint16_t &clutch, uint16_t &rudder, uint16_t &t);

private:
	uint16_t scaleAnalogAxis(uint16_t raw, bool invert, uint16_t minvalue, uint16_t maxvalue);
	uint16_t calculateAveragedValue(uint8_t adcDmaTableIndex);
	void sort_samples(uint16_t tab[], uint8_t lenght);
} __attribute__((packed));

#endif /* CANALOGCONFIG_H_ */

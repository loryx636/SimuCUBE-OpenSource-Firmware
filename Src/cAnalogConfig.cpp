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
 * cAnalogConfig.cpp
 *
 *  Created on: Jun 7, 2017
 *      Author: Mika
 */

#include "cAnalogConfig.h"

extern uint16_t ADC1_Buffer[ADC1_BUFFER_LENGTH];


cAnalogConfig::cAnalogConfig() {
	// TODO Auto-generated constructor stub
	setDefault();
}

cAnalogConfig::~cAnalogConfig() {
	// TODO Auto-generated destructor stub
}

void cAnalogConfig::setDefault() {
	//X.maxvalue = 0;  	X.minvalue = 0; 	X.pin      = 0;
	Y.maxvalue = 65535-defaultAnalogDeadzone; 		Y.minvalue = defaultAnalogDeadzone; 		Y.pin = unUsed; 		Y.invert = false;
	Z.maxvalue = 65535-defaultAnalogDeadzone; 		Z.minvalue = defaultAnalogDeadzone; 		Z.pin = unUsed; 		Z.invert = false;
	Brake.maxvalue = 65535-defaultAnalogDeadzone;		Brake.minvalue = defaultAnalogDeadzone;		Brake.pin = unUsed; 	Y.invert = false;
	Throttle.maxvalue = 65535-defaultAnalogDeadzone;	Throttle.minvalue = defaultAnalogDeadzone;	Throttle.pin = unUsed;	Throttle.invert = false;
	Clutch.maxvalue = 65535-defaultAnalogDeadzone;	Clutch.minvalue = defaultAnalogDeadzone;	Clutch.pin = unUsed; 	Clutch.invert = false;
	Rudder.maxvalue = 65535-defaultAnalogDeadzone;	Rudder.minvalue = defaultAnalogDeadzone;	Rudder.pin = unUsed; 	Rudder.invert = false;
	T.maxvalue = 65535-defaultAnalogDeadzone;			T.minvalue = defaultAnalogDeadzone;	 		T.pin = unUsed; 		T.invert = false;
}

void cAnalogConfig::updateAnalogAxises(bool rawmode, uint16_t &y, uint16_t &z, uint16_t &brake, uint16_t &throttle, uint16_t &clutch, uint16_t &rudder, uint16_t &t) {
	//  is used, because .pin = 0 means unUsed, and the value is used in indexing a table.
	if(Y.pin != unUsed) {
		if(rawmode) { y = calculateAveragedValue(Y.pin); }
		else { y = scaleAnalogAxis(calculateAveragedValue(Y.pin), Y.invert, Y.minvalue, Y.maxvalue );}
	} else {
		y = 32768;
	}
	if(Z.pin != unUsed)  {
		if(rawmode) { z = calculateAveragedValue(Z.pin); }
		else { z = scaleAnalogAxis(calculateAveragedValue(Z.pin), Z.invert, Z.minvalue, Z.maxvalue );}
	}
	if(Brake.pin != unUsed)  {
		if(rawmode) { brake = calculateAveragedValue(Brake.pin); }
		else { brake = scaleAnalogAxis(calculateAveragedValue(Brake.pin), Brake.invert, Brake.minvalue, Brake.maxvalue );}
	}
	if(Throttle.pin != unUsed)  {
		if(rawmode) { throttle = calculateAveragedValue(Throttle.pin); }
		else { throttle = scaleAnalogAxis(calculateAveragedValue(Throttle.pin), Throttle.invert, Throttle.minvalue, Throttle.maxvalue );}
	}
	if(Clutch.pin != unUsed)  {
		if(rawmode) { clutch = calculateAveragedValue(Clutch.pin); }
		else { clutch = scaleAnalogAxis(calculateAveragedValue(Clutch.pin), Clutch.invert, Clutch.minvalue, Clutch.maxvalue );}
	}
	if(Rudder.pin != unUsed)  {
		if(rawmode) { rudder = calculateAveragedValue(Rudder.pin); }
		else { rudder = scaleAnalogAxis(calculateAveragedValue(Rudder.pin), Rudder.invert, Rudder.minvalue, Rudder.maxvalue );}
	}
	if(T.pin != unUsed)  {
		if(rawmode) { t = calculateAveragedValue(T.pin); }
		else { t = scaleAnalogAxis(calculateAveragedValue(T.pin), T.invert, T.minvalue, T.maxvalue );}
	}
}

uint16_t cAnalogConfig::calculateAveragedValue(uint8_t adcpin) {
	// -1 is used, because .pin = 0 means unUsed, and the value is used in indexing a table.
	uint16_t index = adcpin-1;

#if 1
	uint32_t avg_sample = 0;
	uint8_t X = 16;  // how many to disregard
	uint16_t buffer[ADC_NUM_AVERAGING];
	for (uint8_t i = 0; i<ADC_NUM_AVERAGING; i++) {
		buffer[i] = ADC1_Buffer[index];
		index+=NUM_ADC_CHANNELS;
	}
	sort_samples(buffer, ADC_NUM_AVERAGING);
	/* Add the N ADC samples */
	for (index=X/2; index<ADC_NUM_AVERAGING-X/2; index++)
	{
		avg_sample += buffer[index];
	}
	/* Compute the average of N-X ADC sample */
	avg_sample /= ADC_NUM_AVERAGING-X;
	/* Return average value */
	return avg_sample;

#endif
#if 0
	uint32_t sum = 0;
	uint8_t i = 0;
	for (uint8_t i = 0; i<ADC_NUM_AVERAGING; i++) {
		sum+=ADC1_Buffer[index];
		index+=NUM_ADC_CHANNELS;
	}
	return sum/ADC_NUM_AVERAGING;
#endif
}

void cAnalogConfig::sort_samples(uint16_t tab[], uint8_t lenght)
{
	uint8_t l=0x00, exchange =0x01;
	uint16_t tmp=0x00;
	/* Sort tab */
	while(exchange==1)
	{
		exchange=0;
		for(l=0; l<lenght-1; l++)
		{
			if( tab[l] > tab[l+1] )
			{
				tmp = tab[l];
				tab[l] = tab[l+1];
				tab[l+1] = tmp;
				exchange=1;
			}
		}
	}
}

uint16_t cAnalogConfig::scaleAnalogAxis(uint16_t raw, bool invert, uint16_t minvalue, uint16_t maxvalue) {
    float outputfloat = (65535.0*((float)raw-(float)minvalue)/(float)(maxvalue-minvalue));
    uint16_t output;
    if(outputfloat < 0.0) {
        output = 0;
    } else if(outputfloat > 65535.0) {
        output=65535;
    }
    else {
        output = (uint16_t)(65535.0*((float)raw-(float)minvalue)/(float)(maxvalue-minvalue));
    }
    //uint16_t output = (uint16_t)(65535.0*((float)raw-(float)minvalue)/(float)(maxvalue-minvalue));
    uint16_t value;
    if(invert) {
        value = 65535-output;
    }
    else {
        value = output;
    }
    return value;
}



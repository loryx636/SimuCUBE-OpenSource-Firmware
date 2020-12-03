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

#include "cAnalogConfig.h"

extern uint16_t ADC1_Buffer[ADC1_BUFFER_LENGTH];


cAnalogConfig::cAnalogConfig() {

	setDefault();
}

cAnalogConfig::~cAnalogConfig() {

}

void cAnalogConfig::setDefault() {
	for(int i = 0; i<7; i++) {
		printf("%d", i);
		axises[i].settings[addrAnalogPin] = unUsed;
		axises[i].settings[addrAnalogEnabled] = 0;
		axises[i].settings[addrAnalogMin] = defaultAnalogDeadzone;
		axises[i].settings[addrAnalogMax] = 65535-defaultAnalogDeadzone;;
		axises[i].settings[addrAnalogInvert] = 0;
		axises[i].settings[addrAnalogFree1] = 0;
		axises[i].settings[addrAnalogFree2] = 0;
		axises[i].settings[addrAnalogFree3] = 0;
		axises[i].settings[addrAnalogFree4] = 0;
		axises[i].settings[addrAnalogFree5] = 0;
		inputfilters[i].setBiquad(Biquad1StOrder::bq_type_lowpass_1st_order,
				500.0/DRIVEUPDATERATE);
	}
	for(int j=0; j<32; j++) {
		bytestore[j] = 0;
	}
}

void cAnalogConfig::updateAnalogAxises(bool rawmode, uint16_t &y, uint16_t &z,
		uint16_t &brake, uint16_t &throttle, uint16_t &clutch, uint16_t &rudder,
		uint16_t &t) {
	// no zero-initialization to speed up even the -Og build
	uint16_t outputs[7];
	uint16_t avgs[NUM_ADC_CHANNELS];
	updateAvgs(avgs);

	for(int i=0; i<7; i++) {
		const auto pin = axises[i].settings[addrAnalogPin];
		const auto index = (pin == unUsed) ? 0 : pin - 1;
		const auto invert = axises[i].settings[addrAnalogInvert] == 1;
		const auto min = axises[i].settings[addrAnalogMin];
		const auto max = axises[i].settings[addrAnalogMax];

		if(rawmode) {
			outputs[i] = avgs[index];
		} else {
			uint16_t scaled = scaleAnalogAxis(
								avgs[index],
								invert,
								min,
								max);
			outputs[i] = (uint16_t)inputfilters[i].process(scaled);
		}
		if(pin == unUsed) {
			outputs[i] = 0;
		}
	}
	if(axises[0].settings[addrAnalogPin] == unUsed) {
		// special case to let joystick y axis idle at middle if unused.
		// games will get confused with this otherwise when assigning controls.
		outputs[0] = 32768;
	}
	y=outputs[0]; z=outputs[1]; brake=outputs[2]; throttle=outputs[3];
	clutch=outputs[4]; rudder=outputs[5]; t=outputs[6];
}

void cAnalogConfig::updateAvgs(uint16_t (&avgs)[NUM_ADC_CHANNELS]) {
	uint32_t sums[NUM_ADC_CHANNELS] = {0};
	int index = 0;
	for(auto i = 0; i<ADC_NUM_AVERAGING*NUM_ADC_CHANNELS; i++) {
		sums[index] += ADC1_Buffer[i];
		index++;
		if(index == NUM_ADC_CHANNELS) {
			index = 0;
		}
	}
	for(auto i = 0; i<NUM_ADC_CHANNELS; i++) {
		avgs[i] = sums[i] / ADC_NUM_AVERAGING;
	}
}

uint16_t cAnalogConfig::scaleAnalogAxis(
		uint16_t raw,
		bool invert,
		uint16_t minvalue,
		uint16_t maxvalue) {
    float outputfloat = (65535.0*((float)raw-(float)minvalue)/(float)(maxvalue-minvalue));

    // this constraint is to limit the value to uint16_t range
    uint16_t output = fmax(0, fmin(outputfloat, 65535));

    if(invert) {
    	output = 65535 - output;
    }
    return output;
}



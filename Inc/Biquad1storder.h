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
 * Etienne Saint-Paul
 *
 *
 * ---------------------------------------------------------------------------
*/

#ifndef Biquad1storder_h
#define Biquad1storder_h

#include <math.h>

class Biquad1StOrder {
public:

	enum {
		bq_type_passthrough=0, /*no operation*/
		bq_type_lowpass_1st_order=1,/*note 1st order filters have finite attenuation at rejection band (0 or inf depending if hpf/lpf)*/
		bq_type_highpass_1st_order=2/*note 1st order filters have finite attenuation at rejection band (0 or inf depending if hpf/lpf)*/
	};

	Biquad1StOrder();
	Biquad1StOrder( int type, float Fc);
    ~Biquad1StOrder();

    void setBiquad(int type, float Fc);
    float process(float in);
    void reset(float outputvalue=0);//sets internal state so that output has already been settled to a certain output level

protected:
    void calcBiquad(void);
    int type;
    float a0, b1;
    float Fc;
    float z1;
};



#endif // Biquad1storder_h

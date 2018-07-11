
#ifndef Biquad1storder_h
#define Biquad1storder_h

#include <Math.h>

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

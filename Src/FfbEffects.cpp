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

/*
 * FfbEffects.cpp
 *
 *  Created on: 11.5.2018
 *      Author: mikat
 */

#include "config_comm_defines.h"
#include "FfbEngine.h"

#include "stm32f4xx.h"
#define ARM_MATH_CM4
#include <math.h>
#include "arm_math.h"
#include "Biquad1storder.h"
#include "USBGameController.h"

extern USBGameController joystick;

extern uint64_t millis;


#define FFBDEBUG 0
#if FFBDEBUG
#define ffbprintf(...) printf(__VA_ARGS__);
#else
#define ffbprintf(...)
#endif


float constantForceEffect(volatile cEffectState* effect) {


	if(effect->state != MEffectState_Playing) return 0.0f;
	//command+=ef->magnitude*ef->gain;
	//calculate a basic effect strength:
	s32 constantCommand = ((s32)effect->magnitude)*((s32)effect->gain);
	//calculate gains based on the current profile percentages:
	//mConstantGain/100*mMainGain/100;
	//gains = (float)mConfig.profileConfig.mConstantGain*(float)mConfig.profileConfig.mMainGain/10000.0;
#if 0
	gains = (float)mConfig.profileConfigs[currentprofileindex].mMainGain/100.0;
#endif
#if 0
	gains = 1.0;
#endif
	float constantCommandFloat = (float)constantCommand;
	ffbprintf("%d - %d - ", effect->magnitude, effect->gain);
	ffbprintf("%f\r\n", constantCommandFloat);
	return constantCommandFloat;
	//torquecommand+=(constantCommandFloat*gains);
}


float squareEffect(volatile cEffectState* effect, uint8_t gain) {

	if(effect->state != MEffectState_Playing) return 0.0f;
	int32_t time = millis-effect->starttime;
	if((time>effect->duration) && (effect) && (effect->duration != USB_DURATION_INFINITE2)) {
		// effect has ended
		return 0.0f;
	}
	float freq = float(1.0f/effect->period);
	float phase = (float)effect->phase/(float)35999;
	float sign = sinf(2.0*(float)M_PI*(time*freq+phase));
	float out = 0.0f;
	if(sign>0.0) {
		out = effect->offset+effect->magnitude;
	} else {
		out = effect->offset-effect->magnitude;
	}
	out = out*(float)gain/100.0f;
	return out;
}

float sineEffect(volatile cEffectState* effect, uint8_t gain) {
	if(effect->magnitude==0) {
		// optimized version that uses just the offset
		if(effect->state != MEffectState_Playing) return 0.0f;
		//note: this supports only constant force effect made with sine offset and mag = 0.
		// offset is multiplied by 255 to make this match iRacing's always at 255 magnitude for constant force.
		int32_t time = millis-effect->starttime;
		if((time>effect->duration) && (effect->duration != USB_DURATION_INFINITE) && (effect->duration != USB_DURATION_INFINITE2)) {
			// effect has ended
			return 0.0f;
		}

		float sine_offset_constantforce = (float)effect->offset * 255.0f;
		float torquecommand=sine_offset_constantforce;// * gains;
		//printf("sine command before scaling: %d\r\n", (s32)command);
		return torquecommand;
	} else {
#if 1
		if(effect->state != MEffectState_Playing) return 0.0f;
		int32_t time = millis-effect->starttime;
		if((time>effect->duration) && (effect->duration != USB_DURATION_INFINITE) && (effect->duration != USB_DURATION_INFINITE2)) {
			// effect has ended
			return 0.0f;
		}

#if 0
		// basic version
		float freq = float(1.0/ef->period)*1000.0;
		float phase = (float)ef->phase*2.0*(float)M_PI/(float)35999;
		torquecommand+=ef->offset + float(ef->magnitude)*10000.0*sinf(2.0*(float)M_PI*time*freq+phase);
#endif
		// optimized
		float freq = float(1.0)/(float)effect->period;
		float phase = (float)effect->phase/(float)35999;
		float sineoffset = (float)effect->offset;
		float sineChangingPart = float(effect->magnitude)*sinf(2.0*(float)M_PI*(time*freq+phase));
		sineChangingPart = sineChangingPart*(float)gain/100.0f;
		float sinetorque =sineoffset+sineChangingPart;
		float torquecommand=sinetorque*255.0f; // to be in scale with all other forces such as constantforce.
		return torquecommand;
#endif
	}
}

float triangleEffect(volatile cEffectState* effect, uint8_t gain) {
	// fully implemented
#ifdef ffbDebug3
	printf("id,%d,triangle command!\r\n", id);
#endif

	if(effect->state != MEffectState_Playing) return 0.0f;
	int32_t time = millis-effect->starttime;
	if((time>effect->duration) && (effect->duration != USB_DURATION_INFINITE) && (effect->duration != USB_DURATION_INFINITE2)) {
		// effect has ended
		return 0.0f;
	}
	uint32_t period = effect->period; // 1000 to be in us units
	uint32_t phase = (effect->phase*period/35999);//*period;
	uint32_t mag = effect->magnitude;
#if 0
	uint32_t offset = ef->offset;

	uint32_t y = offset-mag+abs(((time+phase) % period) - mag);
	float scaledResult = (float)y*(float)(mConfig.profileConfigs[currentprofileindex].mTriangleGain/100);
	torquecommand+=(float)y;
#endif
	float offset = (float)effect->offset;
	uint32_t y = -mag+abs(((time+phase) % period) - mag);
	float scaledY = (float)y*(float)gain/100.0f;
	float torquecommand=offset+scaledY;
	return torquecommand;
}

float sawtoothDownEffect(volatile cEffectState* effect, uint8_t gain) {
#ifdef ffbDebug3
	printf("id,%d,sawtoothdown command!\r\n", id);
#endif

	if(effect->state != MEffectState_Playing) return 0.0f;

	int32_t time = millis-effect->starttime;
	if((time>effect->duration) && (effect->duration != USB_DURATION_INFINITE) && (effect->duration != USB_DURATION_INFINITE2)) {
		// effect has ended
		return 0.0f;
	}
	uint32_t period = effect->period;
	uint32_t phase = (effect->phase*period/35999);//)*period;
	uint32_t mag = effect->magnitude;
	int32_t offset = effect->offset;
	float tooth = (float)(mag-((time+phase)%period)*mag)*(float)gain/100.0f+(float)offset;
	float torquecommand=tooth*255.0f;
	return torquecommand;
}

float springEffect(volatile cEffectState* effect, uint8_t gain) {

	if(effect->state != MEffectState_Playing) return 0.0f;

	// pos is needed in axis units. Must use the one from last wheel pos update.
	// pos is in 0-65535
	// offset is -10000 - +10000

	// these simplifications are from Etienne's original code:
	// * negative saturation is not used separately (commented out in the interface descriptor)
	// * deadband is unsupported & not used (commented out in the interface descriptor)

	// debugging with ETS2:
	// spring effect gain doesn't change.
	// magnitude? DOES change according to car speed = basic ffb implementation of this game.

	s32 offset = effect->offset;
	offset+=10000; // 0-20000
	s32 axisoffset = offset*65535/20000;
	s32 err = axisoffset - joystick.gFFBDevice.angle.getAxisPos();
	float spring = (float)err*10000.0f/32768.0f*255.0f;  // this scales effect to have full max force
	int32_t spring_int=spring;
	spring_int=constrain(spring_int, -2550000, 2550000);
	spring = (float)spring_int*(float)gain/100.0f;
	// spring effect has also magnitude that goes from -1 to 127 (0x80 - 0x7F in descriptor).
	spring = spring*(float)effect->magnitude/128.0f;
	float torquecommand=-(float)spring;
	return torquecommand;
}

float sawtoothUpEffect(volatile cEffectState* effect, uint8_t gain) {

	if(effect->state != MEffectState_Playing) return 0.0f;
#ifdef ffbDebug3
	printf("id,%d,sawtoothup command!\r\n", id);
#endif
#if 1
	int32_t time = millis-effect->starttime;
	if((time>effect->duration) && (effect->duration != USB_DURATION_INFINITE) && (effect->duration != USB_DURATION_INFINITE2)) {
		// effect has ended
		return 0.0f;
	}
	uint32_t period = effect->period;
	uint32_t phase = (effect->phase*period/35999);//*period;
	uint32_t mag = effect->magnitude;
	int32_t offset = effect->offset;
	float tooth = (float)(((time+phase)%period)*mag)*(float)gain/100.0f+(float)offset;
#endif
	float torquecommand=tooth*255.0f;
	return torquecommand;
	//break;
}

float frictionEffect(int32_t readencoderpos, uint32_t cpr, volatile cEffectState* effect, uint8_t gain) {


	if(effect->state != MEffectState_Playing) return 0.0f;
	//printf("friction,mag:%d,offset:%d,period:%d,gain:%d,possat:%d,negsat:%d\r\n", effect->magnitude, effect->offset, effect->period, effect->gain, effect->positiveSaturation, effect->negativeSaturation);
    float frictionDistanceTurns = (float)effect->magnitude/256.0f;//=0.0030;//turns from 0% to 100% torque in   friction effect // from game
    int frictionDistanceCounts=cpr*frictionDistanceTurns;
    static int frictionLockPosFB=0;

    int frictionDiff = readencoderpos-frictionLockPosFB;

    if( frictionDiff > frictionDistanceCounts )
    {
        frictionLockPosFB = readencoderpos-frictionDistanceCounts;
        frictionDiff=frictionDistanceCounts;
    }
    else if( frictionDiff < -frictionDistanceCounts )
    {
        frictionLockPosFB = readencoderpos+frictionDistanceCounts;
        frictionDiff=-frictionDistanceCounts;
    }

    float frictionTorq = (float)frictionDiff * (1.0f/(float)cpr) * (float)gain;// do not divide this by /100.0; !!!!

    frictionTorq = joystick.gFFBDevice.frictionForceLPF.process(frictionTorq);
    return frictionTorq;// 0.0;//frictionTorq;
}

float calcDampingEffect(uint8_t gain, float speed, volatile cEffectState *effect, Biquad1StOrder& lpf) {

	// this smells funny but might not be totally wrong, as the scaling is usually 255 * 100, here it is 255 * 127
	// max is 0x7F = 127 dec

	// sounds wrong but it is to the right direction, as all of our forces are eventually negated, this
	// force in the direction of speed is damping it.
	const float force = (float)gain/100.0f * (float)effect->magnitude/127.0f * (float)effect->magnitude * speed * 256.0f;

	return lpf.process(force);
}

#define MAX_DESKTOP_SPRING_FORCE_AT_ANGLE 180.0f
float desktopSpringEffect(uint8_t gain) {
	/* use a defined MAX_DESKTOP_SPRING_FORCE_AT_ANGLE angle where the desktop centering spring will give maximum torque, so that the effect
	 * is consistant and does not depend on user's steering angle setting.
	 * Todo: try make it also consistent regardless of the current power level of the system and to drive the
	 * wheel in to center even if the force produced here is not enough in itself to do so.
	 */
	float err=fmax(-MAX_DESKTOP_SPRING_FORCE_AT_ANGLE, fmin(MAX_DESKTOP_SPRING_FORCE_AT_ANGLE, joystick.gFFBDevice.angle.getUnlimitedSteeringAngle()));
	float spring = err*10000.0f/MAX_DESKTOP_SPRING_FORCE_AT_ANGLE*255.0f;  // this scales effect to have full max force at MAX_DESKTOP_SPRING_FORCE_AT_ANGLE degrees
	spring = spring*(float)gain/100.0f;
	float springfiltered = joystick.gFFBDevice.desktopDampingLPF.process(spring);
	//printf("d gain %d err %f spring %f filtered %f\r\n", gain, err, spring, springfiltered);
	return springfiltered;
}

float irFFBEffects() {
	// IMPLEMENT SPECIAL IRFFB MODE HERE,
	// the latest force value is at latestIRFFBForce
	// in normal case, use the same way as constantforce.

	// IRFFB force command does not have gain and magnitude separately, so
	// multiply first with the same magnitude that is generally used (255).
	float constantCommand = (s32)joystick.gFFBDevice.latestIRFFBForce[0]*255;  // 0 = latest.
	//calculate gains based on the current profile percentages:
	//mConstantGain/100*mMainGain/100;
	float gains = (float)joystick.gFFBDevice.mConfig.profileConfigs[joystick.gFFBDevice.currentprofileindex].settings[addrMainGain]/100.0f;
	float constantCommandFloat = (float)constantCommand;

	float torquecommand=(constantCommandFloat*gains);
	return torquecommand;

}

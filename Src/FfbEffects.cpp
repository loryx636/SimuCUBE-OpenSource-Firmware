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
#ifdef ffbDebub4
	printf("%d - %d - ", effect->magnitude, effect->gain);
	printf("%f\r\n", constantCommandFloat);
#endif
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
	//torquecommand+=out;
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

#ifdef ffbDebug3
	//printf("spring command!\r\n");
	printf("id,%d,spring,mag,%d,offset,%d,period,%d\r\n", id, ef->magnitude, ef->offset, ef->period);
#endif
	//printf("spring    eff mag %d, %d\r\n", ef->magnitude, ef->gain);
	//torquecommand += constrain(SpringEffect(ef->offset - pos, (mag*mConfig.profileConfigs[currentprofileindex].mSpringGain) >> 7), -(ef->negativeSaturation << 8), ef->positiveSaturation << 8);

	// pos is needed in axis units. Must use the one from last wheel pos update.
	// pos is in 0-65535
	// offset is -10000 - +10000

	// these simplifications are from Etienne's original code:
	// * negative saturation is not used separately (commented out in the interface descriptor)
	// * deadband is unsupported & not used (commented out in the interface descriptor)

	// debugging with ETS2: spring effect gain doesn't change.
	// magnitude? does not change either.

	s32 offset = effect->offset;
	offset+=10000; // 0-20000
	s32 axisoffset = offset*65535/20000;
	s32 err = axisoffset - joystick.gFFBDevice.axisPos;//+32768;
	//s32 mag = ef->magnitude*mConfig.profileConfigs[currentprofileindex].mSpringGain/100;
	//printf("%ld:%ld:%ld:%ld\r\n", err, ef->offset, ef->positiveSaturation, ef->magnitude);

	float spring = (float)err*10000.0f/32768.0f*255.0f;  // this scales effect to have full max force
	//spring = constrain(spring, -2550000, 2550000);
	int32_t spring_int=spring;
	spring_int=constrain(spring_int, -2550000, 2550000);
	spring = (float)spring_int*(float)gain/100.0f;
	//printf("%ld,%ld\r\n", err, spring);
	//torquecommand+= (float)err*10000/32768
	float torquecommand=-(float)spring;
	return torquecommand;
	//printf("%d;%d\r\n", spring_int,pos);
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

#if 1
float frictionEffect(s32 *readencoderpos, float cntsperinverse, uint32_t cpr, volatile cEffectState* effect, uint8_t gain) {

	if(effect->state != MEffectState_Playing) return 0.0f;

    float frictionDistanceTurns = (float)effect->magnitude/256.0f;//=0.0030;//turns from 0% to 100% torque in   friction effect // from game
    int frictionDistanceCounts=cpr*frictionDistanceTurns;
    static int frictionLockPosFB=0;

    int frictionDiff=*readencoderpos-frictionLockPosFB;

    if( frictionDiff > frictionDistanceCounts )
    {
        frictionLockPosFB=*readencoderpos-frictionDistanceCounts;
        frictionDiff=frictionDistanceCounts;
    }
    else if( frictionDiff < -frictionDistanceCounts )
    {
        frictionLockPosFB=*readencoderpos+frictionDistanceCounts;
        frictionDiff=-frictionDistanceCounts;
    }

    float frictionTorq=float(frictionDiff)*cntsperinverse*(float)gain;// do not divide this by /100.0; !!!!

    //printf("%f\r\n", frictionTorq);
    frictionTorq = joystick.gFFBDevice.frictionForceLPF.process(frictionTorq);
    return frictionTorq;// 0.0;//frictionTorq;

	//stat = smSetParameter(mSMBusHandle, 1, SMP_TORQUE_EFFECT_FRICTION, (mag*mConfig.profileConfigs[currentprofileindex].mFrictionGain) >> 3);
}
#endif
#if 0
float cFFBDevice::frictionEffect(s32 *readencoderpos, float cntsperinverse, uint32_t cpr, volatile cEffectState* effect) {
	ffbEffectUsage |= (1<<FrictionEffectBit); updateEffectCounters((s32)effect->magnitude, FrictionEffectBit);
	if(effect->state != MEffectState_Playing) return 0.0f;
	float frictionCoeff = (float)effect->magnitude;

}
#endif
float calcDamperEffectGain(volatile cEffectState *effect, uint8_t gain) {


	if(effect->state != MEffectState_Playing) return 0.0f;
#ifdef ffbDebug3
	printf("id,%d,damper\r\n",id);
#endif
	//cumul_damper+=(mag*mConfig.profileConfigs[currentprofileindex].mDamperGain) >> 8;
	/*
	 * 		effect->magnitude = (s16)data->positiveCoefficient; int16_t
			effect->offset = data->cpOffset; int16_t
			effect->positiveSaturation = data->positiveSaturation; uint8_t
			effect->negativeSaturation = data->negativeSaturation; uint8_t */
	//printf("mag %d, off %d, pos %d, neg %d\r\n", ef->magnitude, ef->offset, ef->positiveSaturation, ef->negativeSaturation);
	//dampingEffectGain scale same as in simplemotion, 100=1% -> 0-10000 = 0-100%
	// magnitude = (s16)data->positiveCoefficient;
	float dampingEffectGain = (float)effect->magnitude/127.0f; // max is 0x7F = 127 dec
	dampingEffectGain = dampingEffectGain*(float)gain/100.0f;
	//dampingEffectGain = dampingEffectGain*-1;
	//printf("damping mag %d\r\n", effect->magnitude);
	return dampingEffectGain;

}

float calcDampingEffect(float gain, volatile cEffectState *effect) {
	float force = 0.0f;
	//int16_t loffset=effect->offset;
	// deadband not supported
	// int16_t ldeadband = effect->
	int16_t lPositiveCoefficient = effect->magnitude;
	// negative coefficient not supported
	//int16_t lNegativeCoefficient = effect->
	//int16_t dwPositiveSaturation = effect->positiveSaturation;
	//int16_t dwNegativeSaturation = effect->negativeSaturation;

	force = gain*(float)(lPositiveCoefficient)*joystick.gFFBDevice.axisSpeedPerMs*256.0f;
	force = joystick.gFFBDevice.dampingForceLPF.process(force);
	return force;
}

#if 0
float cFFBDevice::desktopSpringEffect() {
	//torquecommand += SpringEffect(-pos, mConfig.hardwareConfig.mDesktopSpringGain);
	//cumul_damper += mConfig.hardwareConfig.mDesktopDamperGain;
	s32 axisoffset = 32767;
	s32 err = axisoffset - axisPos;//+32768;
	float spring = (float)err*10000.0f/32768.0f*255.0f;  // this scales effect to have full max force
	//int32_t spring_int=spring;
	float saturation = 2550000.0*(float)mConfig.hardwareConfig.mDesktopSpringSaturation/100.0f;

	//spring_int=constrain(spring_int, -2550000, 2550000);
	//spring = (float)spring_int*(float)mConfig.hardwareConfig.mDesktopSpringGain/100.0f;
	spring = spring*(float)mConfig.hardwareConfig.mDesktopSpringGain/100.0f;
#if 1
	if(spring > 0.0) {
		spring = constrain(spring, 0, saturation);
	} else {
		spring = constrain(spring, -1.0f*saturation, 0);
	}
#endif
	//printf("%d ", (int)spring);
	//spring = spring;// - desktopDampingLPF.process1stOrder(wheelSpeedCount*(float)mConfig.hardwareConfig.mDesktopDamperGain*10.0f);
	//printf("%d\r\n", (int)spring);
	float torquecommand=-spring;
	//printf("%f   %f\r\n",spring,saturation);
	return desktopDampingLPF.process1stOrder(torquecommand);
}
#endif

#if 0
float cFFBDevice::frictionEffect(cEffectState* effect) {

}
#endif

float desktopSpringEffect(uint8_t saturation, uint8_t gain) {
	//torquecommand += SpringEffect(-pos, mConfig.hardwareConfig.mDesktopSpringGain);
	//cumul_damper += mConfig.hardwareConfig.mDesktopDamperGain;
	//s32 axisoffset = 32767;
	float err = joystick.gFFBDevice.steeringAngleUnlimited;
	err=constrain(joystick.gFFBDevice.steeringAngleUnlimited, -900.0, 900.0);
	float spring = err*10000.0f/900.0f*255.0f;  // this scales effect to have full max force
	//int32_t spring_int=spring;
	float saturationFloat = 2550000.0*(float)saturation/100.0f;

	//spring_int=constrain(spring_int, -2550000, 2550000);
	//spring = (float)spring_int*(float)mConfig.hardwareConfig.mDesktopSpringGain/100.0f;
	spring = spring*(float)gain/100.0f;
#if 1
	if(spring > 0.0) {
		spring = constrain(spring, 0, saturationFloat);
	} else {
		spring = constrain(spring, -1.0f*saturationFloat, 0);
	}
#endif
	//printf("%d ", (int)spring);
	//spring = spring;// - desktopDampingLPF.process1stOrder(wheelSpeedCount*(float)mConfig.hardwareConfig.mDesktopDamperGain*10.0f);
	//printf("%d\r\n", (int)spring);
	float torquecommand=spring;
	//printf("%f   %f\r\n",spring,saturation);
	return joystick.gFFBDevice.desktopDampingLPF.process(torquecommand);
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
	float gains = (float)joystick.gFFBDevice.mConfig.profileConfigs[joystick.gFFBDevice.currentprofileindex].mMainGain/100.0f;
	float constantCommandFloat = (float)constantCommand;

	float torquecommand=(constantCommandFloat*gains);
	return torquecommand;

}

float endstopEffect(float speed) {
	float endstopGain = 0.0;  // 0.0 - 1.0
	float endstopMaxTorque = 0.0; // 0 - 10000 * 255, same scale as other forces
	float endstopTorque_Dampened = 0.0;
	float endstopDampingGain = (float)joystick.gFFBDevice.mConfig.hardwareConfig.mStopsDamperGain/100.0;
	if(joystick.gFFBDevice.mConfig.hardwareConfig.mStopsEnabled == 1) {
		if(joystick.gFFBDevice.degrees_from_endstop < (float)joystick.gFFBDevice.mConfig.hardwareConfig.mStopsRangeDegrees) {
			if(joystick.gFFBDevice.degrees_from_endstop < 0.00f) {
				joystick.gFFBDevice.degrees_from_endstop = 0.0f;
			}
			endstopGain = ((float)joystick.gFFBDevice.mConfig.hardwareConfig.mStopsRangeDegrees-joystick.gFFBDevice.degrees_from_endstop)/(float)joystick.gFFBDevice.mConfig.hardwareConfig.mStopsRangeDegrees;
			if(endstopGain < 0.000f) endstopGain = 0.0f;
			endstopMaxTorque = (float)joystick.gFFBDevice.mConfig.hardwareConfig.mStopsMaxForce*100.0f*255.0f; // to be in the same scale as the other forces
			}

		if(joystick.gFFBDevice.endstopDirection) { // turning to right
			endstopTorque_Dampened = endstopGain*endstopMaxTorque-joystick.gFFBDevice.wheelSpeed*endstopMaxTorque*endstopDampingGain;
			//endstopTorque_Dampened = endstopMaxTorque*(endstopGain-wheelSpeed*endstopGain);
			//endstopTorque_Dampened = endstopMaxTorque*(endstopGain*(1.0-dampergain*wheelSpeed));

		}
		else {
			endstopTorque_Dampened = -endstopGain*endstopMaxTorque-speed*endstopMaxTorque*endstopDampingGain;
			//endstopTorque_Dampened = endstopMaxTorque*(-endstopGain*(1.0-dampergain*wheelSpeed));
		}
		return joystick.gFFBDevice.endStopDampingLPF.process(endstopTorque_Dampened);
	}
	return 0.0;
}

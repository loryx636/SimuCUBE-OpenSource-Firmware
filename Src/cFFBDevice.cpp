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
 * cFFBDevice.cpp
 *
 *  Created on: Jan 30, 2017
 *      Author: Mika
 */

#include <eventlog.h>
#include <ffbengine.h>
#include <FfbEffects.h>
#include "cFFBDevice.h"
//#include "command.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"
#include "config_comm_defines.h"

#include "usbd_customhid.h"
#include "usbgamecontroller.h"
#include "types.h"
#include "FfbEffects.h"

#define ARM_MATH_CM4
#include <math.h>
#include "arm_math.h"

#include "Biquad1storder.h"
#include "simplemotioncomms.h"

//#define ffbDebug3
//#define ffbDebug4
extern volatile SystemStatus currentSystemStatus;
extern volatile SystemStatus nextSystemStatus;
extern volatile SystemStatus previousSystemStatus;
extern volatile SystemStatus previousSystemStatus2;
extern USBD_HandleTypeDef hUsbDeviceFS;

extern uint8_t firmwareVersion[3];
extern uint8_t lowestCompatibleFlashSettingsMajorVersion;
extern uint8_t lowestCompatibleFlashSettingsMinorVersion;
extern uint32_t driveStatusBits;

//extern uint8_t firmwareVersion[3];
extern bool debugMode;
extern bool debugMode2;
extern void start_timer2_15ms_delay();
extern USBGameController joystick;

extern uint32_t globaldebugvalue2;
extern eventLog simucubelog;

extern uint64_t millis;
extern TIM_HandleTypeDef htim6_1MhzEffectClock;
static FLASH_EraseInitTypeDef EraseInitStruct;

#define clipping_led_threshold 16300

cFFBDevice::cFFBDevice() {
	// Auto-generated constructor stub
	SetDefault();
	rawAnalogMode = false;
	forcesEnabled = true; // only disabled for motorconfigwizard
	temporaryCenterPoint = false;

	temporarySteeringAngleIsSet = false;
	temporarySteeringLockToLock = 0;

	IoniDrcDataReceivedBytes=0;
	firmwareuploadingstatuspercentage = 0;
	firmwareuploadinprogress = false;
	memset(movingAvgFilterHistory, 0, sizeof(movingAvgFilterHistory));
	movingAvgFilterLatest =0;
	unsavedSettings = 0;
	indexpointfound = 0;
	motorFaultRegister = 0;
	latestIRFFBForce[0] = 0;
	latestIRFFBForce[1] = 0;
	latestIRFFBForce[2] = 0;
	latestIRFFBForce[3] = 0;
	latestIRFFBForce[4] = 0;
	latestIRFFBForce[5] = 0;
	IRFFBModeEnabled = false;
	IoniFWVersion = 0;
	scHWVersion = hwunknown;
	degrees_from_endstop = 500000.0;
	endstopDirection = false;
	wheelSpeed = 0.0;
	previousEncoderPos = 0;
	endstopdampergain = 0.0;
	endstopMaxTorque = 0.0;
	FFB2500HzWait = false;
	debugvalue1 = 0;
	currentprofileindex = 0;
	defaultprofileindex = 0;
	numberofprofiles = 1;
	rewriteDefault = false;

	ffbDevGain = 255;
	ffbEffectUsage = 0;
	driveInitSuccess=false;
	clippingLedOn=false;

	indexPointEncPos=0;
	counter=0;
	steeringAngleUnlimited=0.0;
	absoluteFullRotOffset=0;
	faultLocationID=0;
	driveStatusBits=0;
}

cFFBDevice::~cFFBDevice() {
	// Auto-generated destructor stub
}

void cFFBDevice::SetFFB(FfbEngine* handle) {
	ffbhandle = handle;
}

void cFFBDevice::SetDefault()	{
	mConfig.SetDefault();

	currentprofileindex = 0;
	defaultprofileindex = 0;
	numberofprofiles = 1;
	rewriteDefault = false;
	waitLastSimpleMotion = false;

	int i = 0;
	lastButtonUpdateMillis = 0;
	button_port[i]	= X12_UPPER_1_GPIO_Port; button_pin[i]	= X12_UPPER_1_Pin;
	button_lastUpdate[i] = 0; button_currentState[i] = GPIO_PIN_RESET; button_debouncedState[i] = GPIO_PIN_RESET;
	i++;

	button_port[i]	= X12_UPPER_2_GPIO_Port; button_pin[i]	= X12_UPPER_2_Pin;
	button_lastUpdate[i] = 0; button_currentState[i] = GPIO_PIN_RESET; button_debouncedState[i] = GPIO_PIN_RESET;
	i++;

	button_port[i]	= X12_UPPER_3_GPIO_Port; button_pin[i]	= X12_UPPER_3_Pin;
	button_lastUpdate[i] = 0; button_currentState[i] = GPIO_PIN_RESET; button_debouncedState[i] = GPIO_PIN_RESET;
	i++;

	button_port[i]	= X12_UPPER_4_GPIO_Port; button_pin[i]	= X12_UPPER_4_Pin;
	button_lastUpdate[i] = 0;button_currentState[i] = GPIO_PIN_RESET;button_debouncedState[i] = GPIO_PIN_RESET;
	i++;

	button_port[i]	= X12_UPPER_5_GPIO_Port;button_pin[i]	= X12_UPPER_5_Pin;
	button_lastUpdate[i] = 0;button_currentState[i] = GPIO_PIN_RESET;button_debouncedState[i] = GPIO_PIN_RESET;
	i++;

	button_port[i]	= X12_UPPER_6_GPIO_Port;button_pin[i]	= X12_UPPER_6_Pin;
	button_lastUpdate[i] = 0;button_currentState[i] = GPIO_PIN_RESET;button_debouncedState[i] = GPIO_PIN_RESET;
	i++;

	button_port[i]	= X12_UPPER_7_GPIO_Port;button_pin[i]	= X12_UPPER_7_Pin;
	button_lastUpdate[i] = 0;button_currentState[i] = GPIO_PIN_RESET;button_debouncedState[i] = GPIO_PIN_RESET;
	i++;

	button_port[i]	= X12_LOWER_1_GPIO_Port;button_pin[i]	= X12_LOWER_1_Pin;
	button_lastUpdate[i] = 0;button_currentState[i] = GPIO_PIN_RESET;button_debouncedState[i] = GPIO_PIN_RESET;
	i++;

	button_port[i]	= X12_LOWER_2_GPIO_Port;button_pin[i]	= X12_LOWER_2_Pin;
	button_lastUpdate[i] = 0;button_currentState[i] = GPIO_PIN_RESET;button_debouncedState[i] = GPIO_PIN_RESET;
	i++;

	button_port[i]	= X12_LOWER_3_GPIO_Port;button_pin[i]	= X12_LOWER_3_Pin;
	button_lastUpdate[i] = 0;button_currentState[i] = GPIO_PIN_RESET;button_debouncedState[i] = GPIO_PIN_RESET;
	i++;

	button_port[i]	= X12_LOWER_4_GPIO_Port;button_pin[i]	= X12_LOWER_4_Pin;
	button_lastUpdate[i] = 0;button_currentState[i] = GPIO_PIN_RESET;button_debouncedState[i] = GPIO_PIN_RESET;
	i++;

	button_port[i]	= X12_LOWER_5_GPIO_Port;button_pin[i]	= X12_LOWER_5_Pin;
	button_lastUpdate[i] = 0;button_currentState[i] = GPIO_PIN_RESET;button_debouncedState[i] = GPIO_PIN_RESET;
	i++;

	button_port[i]	= X12_LOWER_6_GPIO_Port;button_pin[i]	= X12_LOWER_6_Pin;
	button_lastUpdate[i] = 0;button_currentState[i] = GPIO_PIN_RESET;button_debouncedState[i] = GPIO_PIN_RESET;
	i++;

	button_port[i]	= X12_LOWER_7_GPIO_Port;button_pin[i]	= X12_LOWER_7_Pin;
	button_lastUpdate[i] = 0;button_currentState[i] = GPIO_PIN_RESET;button_debouncedState[i] = GPIO_PIN_RESET;

	/* ffb device state variables */
	resetFFBDeviceState();
}

float cFFBDevice::limit(float input, float min, float max) {
	if (input<min) {
		return min;
	} else if(input>max) {
		return max;
	}
	return input;
}

void cFFBDevice::resetFFBDeviceState() {
	if(ffbhandle!=0) {
		ffbhandle->FreeAllEffects();
	}
	ffbDevGain = 255;
	ffbEffectUsage = 0;
	ffbEffectActive = 0;
	for(int i = 0; i<32; i++) {
		ConstantEffectCounters[i]=0;
		LastEffectValue[i]=0;
	}
}

bool cFFBDevice::readEncoder32bit() {
	s32 encoder32 = 0;
	SM_STATUS test = smRead1Parameter(joystick.gFFBDevice.mSMBusHandle, 1, SMP_ACTUAL_POSITION_FB_NEVER_RESETTING , &encoder32);
	if(test != SM_OK) {
		return false;
	}

	if(mConfig.hardwareConfig.mIndexingMode == startAtCenterPhasedIndexing) {
		if(!temporaryCenterPoint) {
			resetPositionCountAt(encoder32);//+mConfig.hardwareConfig.mEncoderOffset);
		} else {
			resetPositionCountAt(encoder32);//+mEncoderTemporaryOffset);
		}
	}
	else if(mConfig.hardwareConfig.mIndexingMode==indexPointIndexing) {
		if(!temporaryCenterPoint) {
			resetPositionCountAt(encoder32);//+mConfig.hardwareConfig.mEncoderOffset);
		} else {
			resetPositionCountAt(encoder32);//+mEncoderTemporaryOffset);
		}
	} else {
		if(!temporaryCenterPoint) {
			resetPositionCountAt(encoder32);//+mConfig.hardwareConfig.mEncoderOffset);
		} else {
			resetPositionCountAt(encoder32);//+mEncoderTemporaryOffset);
		}
	}

	return true;
}


void cFFBDevice::updateEffectCounters(s32 newdata, uint32_t bit) {
	if(LastEffectValue[bit] != newdata) {
		LastEffectValue[bit] = newdata;
		// new data was received, so effect is active
		ffbEffectActive |= (1<<bit);
		ConstantEffectCounters[bit] = ffbActiveLowpassTimeCycles;
	} else {
		if(ConstantEffectCounters[bit]>0) {
			ConstantEffectCounters[bit]--;
		}
		if(ConstantEffectCounters[bit]== 0) {
			// effect was not active, toggle bit	 off
			ffbEffectActive = ffbEffectActive &~(1<<bit);
		}
	}
}

void cFFBDevice::updateEffectRegister() {
	for(int i = 0; i<32; i++) {
		if(ConstantEffectCounters[i]>0) {
			ffbEffectUsage |=(1<<i);
		}
	}
}




extern s32 encoderPos32;

bool cFFBDevice::CalcTorqueCommand(s32 *readEncoderPos) {
	// return if waiting
	if(FFB2500HzWait) {
		return false;
	}
	FFB2500HzWait = true;

#if 1
   	diffMs = (float)diffUs/1000.0;
   	axisSpeedPerMs= (lastAxisPosFloat-axisPosFloat)/diffMs;
   	lastAxisPosFloat=axisPosFloat;
   	axisSpeedPerMs = axisSpeedMsLPF.process(axisSpeedPerMs);
#endif

	// variables needes for effect calculation
    float countsPerRevInverse=1.0/(float)mConfig.hardwareConfig.mEncoderCPR;

	float torquecommand = 0.0;
	float dampingcommand = 0.0;
	float frictioncommand = 0.0;
	float endstopcommand = 0.0;
	float totalcommand = 0.0;
	uint8_t numEffects=0;
	float frictionTorq = 0.0;
	float dampingEffectGain = 0.0;

	// Speed required for damping and endstop effect
	// need to calculate back to degrees to be generally effective
	wheelSpeed = (float)(((previousEncoderPos-*readEncoderPos)/(float)mConfig.hardwareConfig.mEncoderCPR)*360.0);
	int wheelSpeedCount = previousEncoderPos -*readEncoderPos;
	static float prevVel=0;
    float acc=wheelSpeedCount-prevVel;
    prevVel=wheelSpeedCount;



	//printf("s %.2f\r\n", wheelSpeed);
	previousEncoderPos = *readEncoderPos;
	uint32_t cpr = mConfig.hardwareConfig.mEncoderCPR;


	/* END STOP EFFECT CALCULATION */

	endstopcommand = endstopEffect(wheelSpeed);


	ffbEffectUsage = 0;


	if(!IRFFBModeEnabled) {
		/* CALCULATE TORQUE BASED ON EFFECTS */


		for (u8 id = FIRST_EID; id <= MAX_EFFECTS; id++)
		{
			volatile cEffectState* ef = ffbhandle->getEffectState(id);
			if(ef->state == MEffectState_Playing || ef->state == MEffectState_Allocated)
			{
				numEffects++;
				// constant force gain is usually always 255.
				//s32 mag = (((s32)ef->magnitude)*((s32)ef->gain)) >> 6;
				switch (ef->type)
				{
				case USB_EFFECT_CONSTANT:
					ffbEffectUsage |= (1<<ConstantEffectBit); updateEffectCounters((s32)ef->magnitude, ConstantEffectBit);
					torquecommand+=constantForceEffect(ef);
					break;
				case USB_EFFECT_RAMP:
					// not implemented yet
					ffbEffectUsage |= (1<<RampEffectBit);
					updateEffectCounters((s32)ef->magnitude, RampEffectBit);
					if(ef->state != MEffectState_Playing) break;
					break;
				case USB_EFFECT_SQUARE:
					ffbEffectUsage |= (1<<SquareEffectBit);	updateEffectCounters((s32)ef->magnitude, SquareEffectBit);
					torquecommand+=squareEffect(ef, mConfig.profileConfigs[currentprofileindex].mSquareGain);
					break;
				case USB_EFFECT_SINE:
					if(ef->magnitude==0) {
						ffbEffectUsage |= (1<<PeriodSineConstantEffectBit); updateEffectCounters((s32)ef->offset, PeriodSineConstantEffectBit);
					} else {
						ffbEffectUsage |= (1<<PeriodSineChangingEffectBit); updateEffectCounters((s32)ef->magnitude, PeriodSineChangingEffectBit);
					}
					torquecommand+=sineEffect(ef, mConfig.profileConfigs[currentprofileindex].mSineGain);
					break;
				case USB_EFFECT_TRIANGLE:
					ffbEffectUsage |= (1<<TriangleEffectBit); updateEffectCounters((s32)ef->magnitude, TriangleEffectBit);
					torquecommand+=triangleEffect(ef, mConfig.profileConfigs[currentprofileindex].mTriangleGain);
					break;
				case USB_EFFECT_SAWTOOTHDOWN:
					ffbEffectUsage |= (1<<SawtoothDownEffectBit); updateEffectCounters((s32)ef->magnitude, SawtoothDownEffectBit);
					torquecommand+=sawtoothDownEffect(ef, mConfig.profileConfigs[currentprofileindex].mSawtoothGain);
					break;
				case USB_EFFECT_SAWTOOTHUP:
					ffbEffectUsage |= (1<<SawtoothUpEffectBit); updateEffectCounters((s32)ef->magnitude, SawtoothUpEffectBit);
					torquecommand+=sawtoothUpEffect(ef, mConfig.profileConfigs[currentprofileindex].mSawtoothGain);
					break;
				case USB_EFFECT_SPRING:
					ffbEffectUsage |= (1<<SpringEffectBit); updateEffectCounters((s32)ef->offset, SpringEffectBit);
					torquecommand+=springEffect(ef, mConfig.profileConfigs[currentprofileindex].mSineGain);
					break;
				case USB_EFFECT_FRICTION:
					ffbEffectUsage |= (1<<FrictionEffectBit); updateEffectCounters((s32)ef->magnitude, FrictionEffectBit);
					frictionTorq=frictionEffect(readEncoderPos, countsPerRevInverse, cpr, ef, mConfig.profileConfigs[currentprofileindex].mFrictionGain);
					break;
				case USB_EFFECT_DAMPER:
					ffbEffectUsage |= (1<<DampingEffectBit); updateEffectCounters((s32)ef->magnitude, DampingEffectBit);
					dampingEffectGain =calcDamperEffectGain(ef, mConfig.profileConfigs[currentprofileindex].mDamperGain);
					dampingcommand=calcDampingEffect(dampingEffectGain,ef);
					break;
				case USB_EFFECT_INERTIA:
					// not implemented yet
					ffbEffectUsage |= (1<<InertiaEffectBit); updateEffectCounters((s32)ef->magnitude, InertiaEffectBit);
					if(ef->state != MEffectState_Playing) break;
					break;
				case USB_EFFECT_CUSTOM:
					// not implemented yet.
					ffbEffectUsage |= (1<<CustomEffectBit); updateEffectCounters((s32)ef->magnitude, CustomEffectBit);
					if(ef->state != MEffectState_Playing) break;
					break;
				default:
					break;
				}
			}
		}

		/* FFB FILTERING STUFF */

		if (mConfig.profileConfigs[currentprofileindex].filteringModes & (1 << testFilter)) {
			// other filter development can be done here, for example. Define bits like testFilter, and assign new variables in profiles if required.
		}




		/* DESKTOP SPRING EFFECT */

		if (mConfig.hardwareConfig.mDesktopAutoCenter == 1 && numEffects == 0)
		{
			torquecommand+=desktopSpringEffect(mConfig.hardwareConfig.mDesktopSpringSaturation, mConfig.hardwareConfig.mDesktopSpringGain);
		}


		/* PROCESS INERTIA, FRICTION AND DAMPING EFFECTS */
		int inertiaEffectGain = 0;

		float torqsetpointAdder = -(dampingLPF.process( inertiaLPF.process(acc*float(inertiaEffectGain)*1500.0))*countsPerRevInverse + frictionTorq);
		torqsetpointAdder= torqsetpointAdder * 256.0 * 256.0;
		torquecommand-=dampingcommand;//+torqsetpointAdder
		torquecommand-=torqsetpointAdder;
} // END NORMAL MODE
	else {
		//IRFFB MODE
		torquecommand+=irFFBEffects();
	}


	/* Determine if torque should overcome the endstop force or not */
	if(mConfig.hardwareConfig.mStopsEnabled) {
		if(degrees_from_endstop < 0.0005) {
			totalcommand = endstopcommand;
		} else {
			totalcommand = endstopcommand+torquecommand;
		}
	} else {
		totalcommand = torquecommand;
	}

	/* EFFECT SCALING FOR SIMPLEMOTION */
	// this command needs to be scaled to suitable range for Simplemotion:
	totalcommand = (float)ffbDevGain*totalcommand/255.0f;
	int smcommand = scaleEffectForSimplemotion(totalcommand);


#ifdef ffbDebug4
	//printf(",%d\r\n", smcommand);
#endif

	if(!forcesEnabled) {
		*readEncoderPos = SetTorque(0);
	} else {
		*readEncoderPos=SetTorque(smcommand);
	}
	//printf("%d\r\n", pos);
	//	*readEncoderPos=pos;
	if(mConfig.hardwareConfig.invertSteeringValue) axisPos = 65535-axisPos;
	return true;
}

s32 cFFBDevice::ConstrainEffect(s32 val)
{
	return (constrain(val, -MAX_NORM_TORQUE, MAX_NORM_TORQUE));
	return val;
}

s32 cFFBDevice::scaleEffectForSimplemotion(float inputval) {

	//max possible value for val is +/-
	//10000*255 and it needs to scaled to +/- 16384
	// -> output = inputval/maxvalue*maxoutput
	//           = inputval*maxoutput/maxvalue
	s32 returnvalue = 0;
	const float maxoutvalue = 16384.0;
	const float maxinputvalue = 2550000.0; //255 * 10000
	float temp = inputval*maxoutvalue;
	float output = temp/maxinputvalue;
	if(output>=0.0) {
		returnvalue = (s32)(output);//+0.5);
	}
	else {
		returnvalue = (s32)(output);//-0.5);
	}
	if(returnvalue > 16384) {
		//clipping
		returnvalue = 16384;
	} else if (returnvalue < -16384) {
		returnvalue = -16384;
	}
	//printf("%d\r\n", returnvalue);
	if(returnvalue > clipping_led_threshold) {
		// possibly turn a led on here
		if(clippingLedOn==false) {
			clippingLedOn=true;
			HAL_GPIO_WritePin(LED1_CLIPPING_OUT_GPIO_Port, LED1_CLIPPING_OUT_Pin, GPIO_PIN_SET);  // sets pin high
		}

	} else if (returnvalue < -clipping_led_threshold) {
		if(clippingLedOn==false) {
			clippingLedOn=true;
			HAL_GPIO_WritePin(LED1_CLIPPING_OUT_GPIO_Port, LED1_CLIPPING_OUT_Pin, GPIO_PIN_SET);  // sets pin high
		}
	} else if (clippingLedOn==true) {
		clippingLedOn=false;
		HAL_GPIO_WritePin(LED1_CLIPPING_OUT_GPIO_Port, LED1_CLIPPING_OUT_Pin, GPIO_PIN_RESET);  // sets pin low
	}
	//printf("%d\r\n", returnvalue);


	return returnvalue;
}

// this function should be called when mMaxAngle or profile endstop settings is being changed.
void cFFBDevice::initVariables() {
	float halfAngle = 0.0f;
	if(!temporarySteeringAngleIsSet) {
		// init min and max steering angles based on user profile.
		//minSteeringAngle = (-1)*mConfig.profileConfig.mMaxAngle/2;
		halfAngle = (float)(mConfig.profileConfigs[currentprofileindex].mMaxAngle);
	} else {
		halfAngle = (float)(temporarySteeringLockToLock);
	}
	halfAngle = halfAngle/2.0;
	maxSteeringAngle = halfAngle;
	minSteeringAngle = halfAngle*(-1.0);
	minSteeringAngleForStop = minSteeringAngle + (float)mConfig.profileConfigs[currentprofileindex].endstopOffsetAngleDegrees;
	maxSteeringAngleForStop = maxSteeringAngle - (float)mConfig.profileConfigs[currentprofileindex].endstopOffsetAngleDegrees;
	float endstopdampergaintemp = ((float)mConfig.hardwareConfig.mStopsDamperGain);
	endstopdampergain = endstopdampergaintemp/100.0;
	endstopMaxTorque = (float)mConfig.hardwareConfig.mStopsMaxForce*100.0*255.0;
	int cpr=mConfig.hardwareConfig.mEncoderCPR;
	float LPFfreq=4*cpr/1000.0; //4 is good value for even 1024ppr
	limit(LPFfreq,4.0,30.0);
	inertiaLPF.setBiquad(Biquad1StOrder::bq_type_lowpass_1st_order,LPFfreq/2500);
	dampingLPF.setBiquad(Biquad1StOrder::bq_type_lowpass_1st_order, 1250.0/2500.0);
	dampingForceLPF.setBiquad(Biquad1StOrder::bq_type_lowpass_1st_order, 200.0/2500.0);
	frictionForceLPF.setBiquad(Biquad1StOrder::bq_type_lowpass_1st_order, 200.0/2500.0);
	axisSpeedMsLPF.setBiquad(Biquad1StOrder::bq_type_lowpass_1st_order, 100.0/2500.0);
	desktopDampingLPF.setBiquad(Biquad1StOrder::bq_type_lowpass_1st_order, 200.0/2500.0);
	endStopDampingLPF.setBiquad(Biquad1StOrder::bq_type_lowpass_1st_order, 200.0/2500.0);
	prevEffectCalcTime = 0-1;

	axisPos = 32768;

}

extern s32 encoderPos32;

uint16_t cFFBDevice::calcSteeringAngle(s32 encoderCounter)  {
	float steeringAngle = 0.0;
	if(temporaryCenterPoint == true) {
		steeringAngle = (float)(encoderCounter-mEncoderTemporaryOffset)/(float)mConfig.hardwareConfig.mEncoderCPR*360.0;
	} else {
		switch(mConfig.hardwareConfig.mIndexingMode ) {
		case startAtCenterPhasedIndexing:
			steeringAngle = (float)(encoderCounter+mConfig.hardwareConfig.mEncoderOffset-phasingEndOffset)/(float)mConfig.hardwareConfig.mEncoderCPR*360.0;
			break;
		case indexPointIndexing:
			steeringAngle = (float)(encoderCounter-indexPointEncPos+mConfig.hardwareConfig.mEncoderOffset)/(float)mConfig.hardwareConfig.mEncoderCPR*360.0;
			break;
		case autoCommutationIndexing:
			steeringAngle = (float)(encoderCounter-mConfig.hardwareConfig.mEncoderOffset-absoluteFullRotOffset)/(float)mConfig.hardwareConfig.mEncoderCPR*360.0;
			break;
		default:
			steeringAngle = 0.0;
			break;
		}
	}

	// calculations for endstop effect
	if(mConfig.hardwareConfig.mStopsEnabled == 1) {
		if(steeringAngle < (minSteeringAngleForStop + (float)mConfig.hardwareConfig.mStopsRangeDegrees)) {
			//steeringAngle = minSteeringAngle;
			HAL_GPIO_WritePin(LED3_OUT_GPIO_Port, LED3_OUT_Pin, GPIO_PIN_SET);
			degrees_from_endstop = -minSteeringAngleForStop + steeringAngle;
			endstopDirection = false;
			overendstop=1;
		} else if (steeringAngle > (maxSteeringAngleForStop - (float)mConfig.hardwareConfig.mStopsRangeDegrees)) {
			//steeringAngle = maxSteeringAngle;
			HAL_GPIO_WritePin(LED3_OUT_GPIO_Port, LED3_OUT_Pin, GPIO_PIN_SET);
			degrees_from_endstop = maxSteeringAngleForStop - steeringAngle;
			endstopDirection = true;
			overendstop=-1;
		} else {
			HAL_GPIO_WritePin(LED3_OUT_GPIO_Port, LED3_OUT_Pin, GPIO_PIN_RESET);
			degrees_from_endstop = 500000.0;
			overendstop=0;
		}
	}
	steeringAngleUnlimited = steeringAngle;
	// limit the actual angle visible to games
	if(steeringAngle < minSteeringAngle) {
		steeringAngle = minSteeringAngle;
	} else if (steeringAngle > maxSteeringAngle) {
		steeringAngle = maxSteeringAngle;
	}
	// this does things correctly if invertSteeringValue is
	// an int of -1 or 1:
	// int steering = mConfig.hardwareConfig.invertSteeringValue*(steeringAngle/maxSteeringAngle*32768)  + 32768;// + 0.5);
	// Now it is boolean. Code below fixes it.

	// with boolean value, have to make the calculation in separate steps.
	int steering = steeringAngle/maxSteeringAngle*32768;
	if(mConfig.hardwareConfig.invertSteeringValue == true) steering = -steering;
	steering = steering + 32768;

	if(steering < 0 ) {
		steering = 0;
	}
	else if (steering > 65535) {
		steering = 65535;
	}
	uint16_t steeringReturnValue = steering;

	axisPos = (s32)steeringReturnValue;
	axisPosFloat = steeringAngle/maxSteeringAngle*32768.0f+32768.0f;
	constrain(axisPosFloat, 0.0f, 65535.0f);
	return steeringReturnValue;
}


void cFFBDevice::setWheelCenter() {
	// 1) save current encoder pos to mConfig.hardwareConfig.mEncoderOffset
	smint32 positionFB=0;
	volatile int p1,p2;
	p1=SetTorque(0);//call this twice to have 16 bit differential encoder unwrapper initialized
	smRead1Parameter(mSMBusHandle, 1, SMP_ACTUAL_POSITION_FB_NEVER_RESETTING, &positionFB);//read 32 bit position
	simucubelog.addEventParam(96, positionFB, true);
	//return true;

	if(temporaryCenterPoint == true) {
		mEncoderTemporaryOffset = positionFB;//positionFB;
	}
	else if (mConfig.hardwareConfig.mIndexingMode == autoCommutationIndexing) {
		mConfig.hardwareConfig.mEncoderOffset = positionFB;
	}
	else if (mConfig.hardwareConfig.mIndexingMode == startAtCenterPhasedIndexing){
		simucubelog.addEventParam(97, phasingEndOffset-positionFB, true);
		mConfig.hardwareConfig.mEncoderOffset = phasingEndOffset-positionFB; //positionFB+phasingEndOffset;//positionFB;
	}
	else if (mConfig.hardwareConfig.mIndexingMode == indexPointIndexing) {
		simucubelog.addEventParam(98, phasingEndOffset-positionFB, true);
		mConfig.hardwareConfig.mEncoderOffset = indexPointEncPos-positionFB;
	}
}

void cFFBDevice::updateButtons() {
	if(lastButtonUpdateMillis == millis) {
		return;
	}
	lastButtonUpdateMillis = millis;
	uint8_t debounce = mConfig.hardwareConfig.buttonDebounceMillis;
	for(int i=0; i<16; i++) {
		button_currentState[i] = HAL_GPIO_ReadPin(button_port[i], button_pin[i]);
		if((button_currentState[i] != button_debouncedState[i]) && ((button_lastUpdate[i]+debounce) < millis)) {
			button_debouncedState[i] = button_currentState[i];
			button_lastUpdate[i]=lastButtonUpdateMillis;
		}
	}
}

bool cFFBDevice::loadConfigsFromFlash() {

#if 1
	volatile uint32_t idstring = *(uint32_t*)settingsStartAddress;
	uint8_t b = (idstring >> 24);
	uint8_t c = (idstring >> 16) & 0xFF;
	uint8_t m = (idstring >> 8)  & 0xFF;
	uint8_t s = (idstring) & 0xFF;
	if(s != 'S' || m != 'M' || c != 'C' || b != 'B') {
		if(debugMode) printf("no settings found in flash at all!\r\n");
		simucubelog.addEvent(flashEmpty);
		return false;
	}
	volatile uint32_t verstring = *(uint32_t*)(settingsStartAddress+4);

	uint8_t flashMajorVersion = (verstring) & 0xFF;
	uint8_t flashMinorVersion = (verstring >> 8)  & 0xFF;
	uint8_t flashBuildVersion = (verstring >> 16) & 0xFF;

	// too old checks
	if(flashMajorVersion < lowestCompatibleFlashSettingsMajorVersion) {
		if(debugMode) printf("reading flash failed: too old major settings on flash!\r\n");
		simucubelog.addEvent(flashTooOldMajor);
		return false;
	}
	if((flashMajorVersion == lowestCompatibleFlashSettingsMajorVersion)
			&& (flashMinorVersion < lowestCompatibleFlashSettingsMinorVersion)) {
		if(debugMode) printf("reading flash failed: too old minor settings on flash!\r\n");
		simucubelog.addEvent(flashTooOldMinor);
		return false;
	}

	// too new checks
	if(flashMajorVersion > firmwareVersion[majorVersionIdx]) {
		if(debugMode) printf("reading flash failed: too new major settings on flash!\r\n");
		simucubelog.addEvent(flashTooNewMajor);
		return false;
	}
	if((flashMinorVersion > firmwareVersion[minorVersionIdx])
			&& (flashMajorVersion == lowestCompatibleFlashSettingsMajorVersion)) {
		if(debugMode) printf("reading flash failed: too new minor settings on flash!\r\n");
		simucubelog.addEvent(flashTooNewMinor);
		return false;
	}

	// right major and minor versions found.

	if(flashBuildVersion<firmwareVersion[buildVersionIdx] ||  flashMinorVersion<firmwareVersion[minorVersionIdx]) {
		// need to write read-only default profile again.
		rewriteDefault = true;
	}

	uint32_t sourceaddr = settingsStartAddress+7;
	uint8_t* destAddress = mConfig.GetHardwareConfigAddr();
	for(uint32_t i = 0; i<sizeof(hwData); i++) {
		*destAddress = *(uint8_t*)(sourceaddr);
		destAddress++;
		sourceaddr++;
	}
	destAddress = mConfig.GetAnalogConfigAddr();
	for(uint32_t i = 0; i<sizeof(analogData); i++) {
		*destAddress = *(uint8_t*)(sourceaddr);
		destAddress++;
		sourceaddr++;
	}

	/* read number of profiles, low and high bytes */
	uint8_t bytelo = *(uint8_t*)sourceaddr; sourceaddr++;
	uint8_t bytehi = *(uint8_t*)sourceaddr; sourceaddr++;
	numberofprofiles = bytelo + (bytehi << 8);

	// read defaultprofile, low and high bytes */
	bytelo = *(uint8_t*)sourceaddr; sourceaddr++;
	bytehi = *(uint8_t*)sourceaddr; sourceaddr++;
	defaultprofileindex = bytelo + (bytehi << 8);
	currentprofileindex=defaultprofileindex;

#if 0
	for (int i = 0; i<maxnumprofiles; i++) {
		destAddress = mConfig.GetProfileConfigAddr(i);
		for(uint32_t i = 0; i<sizeof(profData); i++) {
			*destAddress = *(uint8_t*)(sourceaddr);
			destAddress++;
			sourceaddr++;
		}
	}
#endif
#if 1
	uint16_t size = sizeof(profData);
	uint8_t startbyteindex=profileDataByteSplitIndex;

	// 1st 55 byte pages
	for (int i = 0; i<maxnumprofiles; i++) {
		destAddress = mConfig.GetProfileConfigAddr(i);
		for(uint32_t i = 0; i<startbyteindex; i++) {
			*destAddress = *(uint8_t*)(sourceaddr);
			destAddress++;
			sourceaddr++;
		}
	}

	// 2nd pages
	for (int i = 0; i<maxnumprofiles; i++) {
		destAddress = mConfig.GetProfileConfigAddr(i)+startbyteindex;
		for(uint32_t i = startbyteindex; i<size; i++) {
			*destAddress = *(uint8_t*)(sourceaddr);
			destAddress++;
			sourceaddr++;
		}
	}
#endif

#endif

#if 0
	destAddress = mConfig.GetProfileConfigAddr();
	for(int i = 0; i<sizeof(profData); i++) {
		*destAddress = *(uint8_t*)(sourceaddr);
		destAddress++;
		sourceaddr++;
	}
#endif
	return true;

}

bool cFFBDevice::saveConfigsToFlash() {
	// unlock flash
	HAL_FLASH_Unlock();
	//HAL_FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);


	// erase flash page
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP|FLASH_FLAG_PGPERR|FLASH_FLAG_WRPERR);
	EraseInitStruct.TypeErase = TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange = VOLTAGE_RANGE_3;
	uint32_t FirstSector = GetSector(settingsStartAddress);
	uint32_t SectorError = 0;
	EraseInitStruct.Sector = FirstSector;
	EraseInitStruct.NbSectors = 1;//NbOfSectors;
	//HAL_FLASH_ErasePage(startAddress);
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
	{
		/*
		  Error occurred while sector erase.
		  User can add here some code to deal with this error.
		  SectorError will contain the faulty sector and then to know the code error on this sector,
		  user can call function 'HAL_FLASH_GetError()'
		*/
		/*
		  FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError();
		*/
		// Error_Handler();
		if(debugMode) printf("Flash Sector Erase Error occurred.\r\n");
		simucubelog.addEvent(flashWriteFailure);
		HAL_FLASH_Lock();
		return false;
	}

	// write data.
	uint32_t address = settingsStartAddress;

	uint8_t s='S';
	uint8_t m='M';
	uint8_t c='C';
	uint8_t b='B';
	// first write magic string
	if (HAL_FLASH_Program(TYPEPROGRAM_BYTE, address, s) != HAL_OK) {
		if(debugMode) printf("Flash Sector Write Error occurred.\r\n");
		HAL_FLASH_Lock();
		simucubelog.addEventParam(flashWriteFailure,1);
		return false;
	}
	address++;
	if (HAL_FLASH_Program(TYPEPROGRAM_BYTE, address, m) != HAL_OK) {
		if(debugMode) printf("Flash Sector Write Error occurred.\r\n");
		HAL_FLASH_Lock();
		simucubelog.addEventParam(flashWriteFailure,2);
		return false;
	}
	address++;
	if (HAL_FLASH_Program(TYPEPROGRAM_BYTE, address, c) != HAL_OK) {
		if(debugMode) printf("Flash Sector Write Error occurred.\r\n");
		HAL_FLASH_Lock();
		simucubelog.addEventParam(flashWriteFailure,3);
		return false;
	}
	address++;
	if (HAL_FLASH_Program(TYPEPROGRAM_BYTE, address, b) != HAL_OK) {
		if(debugMode) printf("Flash Sector Write Error occurred.\r\n");
		HAL_FLASH_Lock();
		simucubelog.addEventParam(flashWriteFailure,4);
		return false;
	}
	address++;


	// write version info
	if (HAL_FLASH_Program(TYPEPROGRAM_BYTE, address, firmwareVersion[majorVersionIdx]) != HAL_OK) {
		if(debugMode) printf("Flash Sector Write Error occurred.\r\n");
		HAL_FLASH_Lock();
		simucubelog.addEventParam(flashWriteFailure,5);
		return false;
	}
	address++;
	if (HAL_FLASH_Program(TYPEPROGRAM_BYTE, address, firmwareVersion[minorVersionIdx]) != HAL_OK) {
		if(debugMode) printf("Flash Sector Write Error occurred.\r\n");
		HAL_FLASH_Lock();
		simucubelog.addEventParam(flashWriteFailure,6);
		return false;
	}
	address++;
	if (HAL_FLASH_Program(TYPEPROGRAM_BYTE, address, firmwareVersion[buildVersionIdx]) != HAL_OK) {
		if(debugMode) printf("Flash Sector Write Error occurred.\r\n");
		HAL_FLASH_Lock();
		simucubelog.addEventParam(flashWriteFailure,7);
		return false;
	}
	address++;
	// then write hardware config
	uint16_t size = sizeof(hwData);
	uint8_t* pointer = mConfig.GetHardwareConfigAddr();
	//volatile uint8_t* originalpointer = pointer;
	for(int i=0; i<size; i++) {
		uint8_t data = *pointer;
		if (HAL_FLASH_Program(TYPEPROGRAM_BYTE, address, data) != HAL_OK) {
			if(debugMode) printf("Flash Sector Write Error occurred.\r\n");
			HAL_FLASH_Lock();
			simucubelog.addEventParam(flashWriteFailure,8);
			return false;
		}
		address++;
		pointer++;
	}
	//then write analog axis config
	size = sizeof(analogData);
	pointer = mConfig.GetAnalogConfigAddr();
	for(int i=0; i<size; i++) {
		uint8_t data = *pointer;
		if (HAL_FLASH_Program(TYPEPROGRAM_BYTE, address, data) != HAL_OK) {
			if(debugMode) printf("Flash Sector Write Error occurred.\r\n");
			HAL_FLASH_Lock();
			simucubelog.addEventParam(flashWriteFailure,9);
			return false;
		}
		address++;
		pointer++;
	}
	//then write number of profiles number lobyte
	uint8_t byte = numberofprofiles & 0xFF;
	if (HAL_FLASH_Program(TYPEPROGRAM_BYTE, address, byte) != HAL_OK) {
		if(debugMode) printf("Flash Sector Write Error occurred.\r\n");
		HAL_FLASH_Lock();
		simucubelog.addEventParam(flashWriteFailure,10);
		return false;
	}
	address++;
	// hibyte
	byte = (numberofprofiles >> 8) & 0xFF;
	if (HAL_FLASH_Program(TYPEPROGRAM_BYTE, address, byte) != HAL_OK) {
		if(debugMode) printf("Flash Sector Write Error occurred.\r\n");
		HAL_FLASH_Lock();
		simucubelog.addEventParam(flashWriteFailure,11);
		return false;
	}
	address++;
	//then write default profile number lobyte
	byte = defaultprofileindex & 0xFF;
	if (HAL_FLASH_Program(TYPEPROGRAM_BYTE, address, byte) != HAL_OK) {
		if(debugMode) printf("Flash Sector Write Error occurred.\r\n");
		HAL_FLASH_Lock();
		simucubelog.addEventParam(flashWriteFailure,12);
		return false;
	}
	address++;
	// hibyte
	byte = (defaultprofileindex >> 8) & 0xFF;
	if (HAL_FLASH_Program(TYPEPROGRAM_BYTE, address, byte) != HAL_OK) {
		if(debugMode) printf("Flash Sector Write Error occurred.\r\n");
		HAL_FLASH_Lock();
		simucubelog.addEventParam(flashWriteFailure,13);
		return false;
	}
	address++;
	//then write profile configs
#if 0
	size = sizeof(profData);

	for(int i = 0; i< maxnumprofiles; i++) {
		pointer = mConfig.GetProfileConfigAddr(i);
		for(int i=0; i<size; i++) {
			uint8_t data = *pointer;
			if (HAL_FLASH_Program(TYPEPROGRAM_BYTE, address, data) != HAL_OK) {
				if(debugMode) printf("Flash Sector Write Error occurred.\r\n");
				HAL_FLASH_Lock();
				simucubelog.addEventParam(flashWriteFailure,14);
				return false;
			}
			address++;
			pointer++;
		}
	}
#endif

	// 2-page profiledatas. Split to maintain compatibility with data where there was only 55 bytes of
	// configuration data.
#if 1
	size = sizeof(profData);
	uint8_t startbyteindex=profileDataByteSplitIndex;

	// 1st pages
	for(int i = 0; i< maxnumprofiles; i++) {
		pointer = mConfig.GetProfileConfigAddr(i);
		for(int i=0; i<startbyteindex; i++) {
			uint8_t data = *pointer;
			if (HAL_FLASH_Program(TYPEPROGRAM_BYTE, address, data) != HAL_OK) {
				if(debugMode) printf("Flash Sector Write Error occurred.\r\n");
				HAL_FLASH_Lock();
				return false;
			}
			address++;
			pointer++;
		}
	}

	//2nd pages
	for(int i = 0; i< maxnumprofiles; i++) {
		pointer = mConfig.GetProfileConfigAddr(i)+startbyteindex;
		for(int i=startbyteindex; i<size; i++) {
			uint8_t data = *pointer;
			if (HAL_FLASH_Program(TYPEPROGRAM_BYTE, address, data) != HAL_OK) {
				if(debugMode) printf("Flash Sector Write Error occurred.\r\n");
				HAL_FLASH_Lock();
				return false;
			}
			address++;
			pointer++;
		}
	}

#endif
	HAL_FLASH_Lock();
	return true;
}






/**
  * @brief  Gets the sector of a given address
  * @param  None
  * @retval The sector of a given address
  */
uint32_t cFFBDevice::GetSector(uint32_t Address)
{
  uint32_t sector = 0;

  if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_SECTOR_0;
  }
  else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_SECTOR_1;
  }
  else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_SECTOR_2;
  }
  else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_SECTOR_3;
  }
  else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_SECTOR_4;
  }
  else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
  {
    sector = FLASH_SECTOR_5;
  }
  else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
  {
    sector = FLASH_SECTOR_6;
  }
  else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
  {
    sector = FLASH_SECTOR_7;
  }
  else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
  {
    sector = FLASH_SECTOR_8;
  }
  else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
  {
    sector = FLASH_SECTOR_9;
  }
  else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
  {
    sector = FLASH_SECTOR_10;
  }
  else /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_11) */
  {
    sector = FLASH_SECTOR_11;
  }

  return sector;
}

bool cFFBDevice::handleIRFFBcommand(uint8_t* data) {
	setIRFFBForcePacket* packet = (setIRFFBForcePacket*) data;
	latestIRFFBForce[0] = packet->IRForce[0];
	latestIRFFBForce[1] = packet->IRForce[1];
	latestIRFFBForce[2] = packet->IRForce[2];
	latestIRFFBForce[3] = packet->IRForce[3];
	latestIRFFBForce[4] = packet->IRForce[4];
	latestIRFFBForce[5] = packet->IRForce[5];
	return true;
}

extern s32 encoderCounter;

// return false if main() should change to another mode.
// return true, if main() can continue in the same mode.
// If needed to just quickly visit another mode,
// set previousSystemStatus = currentSystemStatus before
// changing mode, so that main() can know what to do
// after completing a special operation / quick visit.
bool cFFBDevice::handleSimuCUBEAPIcommand(uint8_t* data) {
	//if(debugMode2) printf("processing SimuCUBE custom command: ");
	int returnvalue = USBD_BUSY;
	uint8_t command = data[1];
	switch(command) {
		case requestStatus:
		{
			//if(debugMode2)printf(" requeststatus\r\n");
			// build reply. Increment pointer accordingly. Start from reply[1]
			statusReplyPacket reply;
			reply.majorVersion = firmwareVersion[majorVersionIdx];
			reply.minorVersion = firmwareVersion[minorVersionIdx];
			reply.buildVersion = firmwareVersion[buildVersionIdx];
			reply.SimuCubeStatus = currentSystemStatus;
			reply.previousCommandSuccess = 1;
			reply.simucubeStatusBits |= (indexpointfound<<indexpointFoundBit);
			reply.drcReceivedBytes = IoniDrcDataReceivedBytes;
			reply.DriveFWUploadPercentage = firmwareuploadingstatuspercentage;
			reply.DriveFWVersion = IoniFWVersion;
			reply.hwVersion = scHWVersion;
			reply.motorfaults = motorFaultRegister;

			reply.activeProfileIndex = currentprofileindex;
			reply.defaultProfileIndex = defaultprofileindex;
			reply.numberofprofiles = numberofprofiles;
			// these are automatically calculated in torqueCommand generation. If not doing that
			// and thus not updated, need to interpret that in the config software side?
			reply.driveStatus = driveStatusBits;
			//uint8_t* pointer = reinterpret_cast<uint8_t*>(reply);

			reply.simucubeStatusBits |= (unsavedSettings<<unsavedSettingsBit);

			if(driveInitSuccess) reply.simucubeStatusBits |= (1<<initSuccessBit);

			reply.motorFaultLocationID = faultLocationID;
			reply.debugvalue1 = debugvalue1;
			reply.debugvalue2 = overendstop;//globaldebugvalue2;
			reply.logVerbosityMode = simucubelog.getVerbosity();

			//updateEffectRegister();
			reply.ffbEffectsInUse = ffbEffectUsage;
			reply.ffbEffectsInActiveUse = ffbEffectActive;
			reply.driveTypeID=DeviceTypeID;

			int counter = 0;
			//if(debugMode2)printf("sending reply\r\n");
			while (returnvalue != USBD_OK) {												   // remember to always send 61, otherwise microsoft hid discards it!
				counter++;
				returnvalue = USBD_CUSTOM_HID_SendReport( &hUsbDeviceFS, (uint8_t*)&reply, 61);//sizeof(statusReplyPacket));
				if(returnvalue == USBD_BUSY) {if(debugMode2)printf(".");}

				if(currentSystemStatus == Operational) {
					//printf("!\r\n");
					bool newCalculation = joystick.gFFBDevice.CalcTorqueCommand(&encoderCounter); //reads encoder counter too
					if(newCalculation) {
						joystick.gFFBDevice.calcSteeringAngle(encoderCounter);
					}
				}
			}
			if(debugMode) {
				//if(counter>370) printf("%d\r\n", counter);
			}
			//if(debugMode2)printf("sent reply\r\n");
			// this is important so that any next USBD_CUSTOM_HID_SendReport command won't get to overwrite the pointer. Also, this must be kept fast enough so that simplemotion won't reset.
			start_timer2_15ms_delay();
			//HAL_Delay(20);
			return true;
			break;
		}

		case requestEventLog:
		{
			//simucubelog.setDisabled();
			commandPacket* packet = (commandPacket*) data;
			uint16_t offset = packet->value;
			if(debugMode) printf(" eventlogRequest, offset %d\r\n",offset);
			simucubelog.addEvent(Command_eventLogRequest);

			uint16_t ev;
			int32_t param;
			eventLogReplyPacket reply;
			reply.latestEvent = simucubelog.lastEventIndex;
			for(int i=0;i<9;i++) {
				if(offset+i > logSize-1) {
					// log is at end.
					break;
				}
				simucubelog.getEvent(offset+i, ev, param);//reply.events[i], reply.parameters[i]);
				reply.events[i]=ev;
				reply.parameters[i]=param;
			}
			while (returnvalue != USBD_OK) {												   // remember to always send 61, otherwise microsoft hid discards it!
				returnvalue = USBD_CUSTOM_HID_SendReport( &hUsbDeviceFS, (uint8_t*)&reply, 61);//sizeof(statusReplyPacket));
				if(returnvalue == USBD_BUSY) {if(debugMode2)printf(".");}

				if(currentSystemStatus == Operational) {
					bool newCalculation = joystick.gFFBDevice.CalcTorqueCommand(&encoderCounter); //reads encoder counter too
					if(newCalculation) {
						joystick.gFFBDevice.calcSteeringAngle(encoderCounter);
					}
				}
			}
			start_timer2_15ms_delay();
			return true;
			break;
		}

		case startEventLogging:
			if(debugMode) printf(" startlogging\r\n");
			simucubelog.addEvent(Command_startLogging);
			simucubelog.setEnabled();
			return true;
			break;

		case stopEventLogging:
			if(debugMode) printf(" stoplogging\r\n");
			simucubelog.addEvent(Command_stopLogging);
			simucubelog.setDisabled();
			return true;
			break;

		case setEventLogVerbosity:
		{
			commandPacket* packet = (commandPacket*) data;
			uint16_t newverbosity = packet->value;
			if(debugMode) printf(" setLogVerbosity %d\r\n", newverbosity);
			simucubelog.addEventParam(Command_setLogVerbosity, newverbosity);
			simucubelog.setVerbosity(newverbosity);
			return true;
			break;
		}

		case saveToFlash:
		{
			if(debugMode) printf(" savetoflash\r\n");
			// this can take a long time. Do it in main.
			nextSystemStatus = currentSystemStatus;
			currentSystemStatus = saveSettingsToFlash;
			return false;
			break;
		}

		case startIRFFBMode:
			simucubelog.addEvent(Command_startIRFFBmode);
			IRFFBModeEnabled = true;
			return true;
			break;

		case stopIRFFBMode:
			IRFFBModeEnabled = false;
			latestIRFFBForce[0] = 0;
			latestIRFFBForce[1] = 0;
			latestIRFFBForce[2] = 0;
			latestIRFFBForce[3] = 0;
			latestIRFFBForce[4] = 0;
			latestIRFFBForce[5] = 0;
			simucubelog.addEvent(Command_stopIRFFBmode);
			return true;
			break;

		case reloadFromFlash:
		{
			if(debugMode) printf(" reloadfromflash\r\n");
			simucubelog.addEvent(Command_reloadFlash);
			if(!loadConfigsFromFlash()) {
				nextSystemStatus = currentSystemStatus;
				currentSystemStatus = FlashFault;
				return false;
				// TODO: handle error (invalid version or other faults?
			}

			// first have to set default full MMC to Ioni
			if(currentSystemStatus == Operational) {
				currentSystemStatus = BeforeOperational;
				unsavedSettings = 0;
				return false;
			}
			// else, was in some config or fault mode, and that doesn't need to be changed.
			unsavedSettings = 0;
			return true;
			break;
		}

		case unsetSettingsChanged:
			if(debugMode) printf(" unsetsettingschanged\r\n");
			simucubelog.addEvent(Command_unsetSettingChanged);
			unsavedSettings = 0;
			return true;
			break;

		case requestProfileConfig:
		{

			commandPacket* packet = (commandPacket*) data;
			uint16_t profileindex = packet->value;
			uint8_t profilePage = packet->value2;

			profileConfigReplyPacket reply;
#if 0
			memcpy(reply.data, mConfig.GetProfileConfigAddr(profileindex), sizeof(cProfileConfig));
#endif
#if 1
			uint8_t* source;
			if(profileindex>(numberofprofiles-1)) {
				source = mConfig.GetProfileConfigAddr(0)+profilePage*profileDataByteSplitIndex;
			} else {
				source = mConfig.GetProfileConfigAddr(profileindex)+profilePage*profileDataByteSplitIndex;
			}
			int8_t* target = reply.data;
			// count number of valid bytes
			uint16_t profdatasize = sizeof(profData);
			uint8_t validbytes;
			uint16_t fulldata = profileDataByteSplitIndex*(profilePage+1);
			if(fulldata>profdatasize) {
				validbytes = fulldata-profdatasize;
			} else {
				validbytes=profileDataByteSplitIndex;
			}
			memcpy(target, source, validbytes);
#endif
			if(debugMode) printf("requestprofile %dp%d\r\n", profileindex, profilePage);
			simucubelog.addEventParam(Command_requestProfile, profileindex);
			simucubelog.addEventParam(Command_requestProfilepage, profilePage);
			if(debugMode2)printf("sending reply\r\n");
			while (returnvalue != USBD_OK) {
				returnvalue = USBD_CUSTOM_HID_SendReport( &hUsbDeviceFS, (uint8_t*)&reply, 61);
				if(returnvalue == USBD_BUSY) {if(debugMode2)printf(".");}
			}
			if(debugMode)printf(" sent reply\r\n");
			// this is important so that any next USBD_CUSTOM_HID_SendReport command won't get to overwrite the pointer. Also, this must be kept fast enough so that simplemotion won't reset.
			start_timer2_15ms_delay();
			//HAL_Delay(20);
			return true;
			break;
		}

		case requestHardwareConfig:
		{
			if(debugMode) printf("requesthardware\r\n");
			simucubelog.addEvent(Command_requestHardware);
			hardwareConfigReplyPacket reply;
			memcpy(reply.data, mConfig.GetHardwareConfigAddr(), sizeof(cHardwareConfig));
			printf("cpr: %ld\r\n", mConfig.hardwareConfig.mEncoderCPR);
			if(debugMode2)printf("sending reply\r\n");
			while (returnvalue != USBD_OK) {
				returnvalue = USBD_CUSTOM_HID_SendReport( &hUsbDeviceFS, (uint8_t*)&reply, 61);
				if(returnvalue == USBD_BUSY) {if(debugMode2)printf("-");}
			}
			if(debugMode)printf(" sent reply\r\n");
			// this is important so that any next USBD_CUSTOM_HID_SendReport command won't get to overwrite the pointer. Also, this must be kept fast enough so that simplemotion won't reset.
			start_timer2_15ms_delay();
			//HAL_Delay(20);
			return true;
			break;
		}

		case requestAnalogConfig:
		{
			if(debugMode) printf("requestanalog\r\n");
			simucubelog.addEvent(Command_requestAnalog);
			analogConfigReplyPacket reply;
			memcpy(reply.data, mConfig.GetAnalogConfigAddr(), sizeof(cAnalogConfig));
			if(debugMode)printf("sending reply\r\n");
			while (returnvalue != USBD_OK) {
				returnvalue = USBD_CUSTOM_HID_SendReport( &hUsbDeviceFS, (uint8_t*)&reply, 61);
				if(returnvalue != USBD_OK) {if(debugMode2)printf("/");}
			}
			if(debugMode2)printf(" sent reply\r\n");
			// this is important so that any next USBD_CUSTOM_HID_SendReport command won't get to overwrite the pointer. Also, this must be kept fast enough so that simplemotion won't reset.
			start_timer2_15ms_delay();
			//HAL_Delay(20);
			return true;
			break;
		}

		case requestIoniConfig:
		{
			if(debugMode) printf(" requestioniconfig\r\n");
			simucubelog.addEvent(Command_requestIoniData);
			ioniConfigRequestPacket* request = (ioniConfigRequestPacket*) data;
			ioniConfigReplyPacket reply;
			// 1) parse needed parameters
			// 2) request them from Ioni

			reply.numberOfValues=request->numberOfValues;

			for (int i=0; i<request->numberOfValues; i++) {
				reply.parameters[i]=request->parameters[i];
				smRead1Parameter(mSMBusHandle, 1, request->parameters[i], &reply.values[i]);
			}

			// 3) send them over USB
			if(debugMode)printf("sending reply\r\n");
			while (returnvalue != USBD_OK) {
				returnvalue = USBD_CUSTOM_HID_SendReport( &hUsbDeviceFS, (uint8_t*)&reply, sizeof(ioniConfigReplyPacket));
				if(returnvalue == USBD_BUSY) {if(debugMode2)printf("_");}
			}
			if(debugMode)printf("sent reply\r\n");
			// this is important so that any next USBD_CUSTOM_HID_SendReport command won't get to overwrite the pointer. Also, this must be kept fast enough so that simplemotion won't reset.
			start_timer2_15ms_delay();
			//HAL_Delay(20);
			return true;
			break;
		}


		case setHardwareConfig:
		{
			if(debugMode) printf("sethardware\r\n");
			simucubelog.addEvent(Command_setHardware);
			//if(debugMode) printf("endstopdampergain %d\r\n", mConfig.hardwareConfig.mStopsDamperGain);
			setConfigPacket* packet = (setConfigPacket*) data;
			memcpy(mConfig.GetHardwareConfigAddr(), &packet->data[0], sizeof(hwData));
			//if(debugMode) printf("endstopdampergain %d\r\n ", mConfig.hardwareConfig.mStopsDamperGain);

			// set beeps according to mode
			s32 parametervalue;
			smRead1Parameter(mSMBusHandle, 1, SMP_DRIVE_FLAGS, &parametervalue);
			if(mConfig.hardwareConfig.audibleNotificationsEnabled == 1) {
				parametervalue = parametervalue | FLAG_ENABLE_MOTOR_SOUND_NOTIFICATIONS;
			} else {
				parametervalue = parametervalue & ~FLAG_ENABLE_MOTOR_SOUND_NOTIFICATIONS;
			}
			smSetParameter(mSMBusHandle, 1, SMP_DRIVE_FLAGS, parametervalue);

			if(currentSystemStatus==Operational) {
				currentSystemStatus = BeforeOperational;
			}
			unsavedSettings = 1;
			return false;
			break;
		}

		case setProfileConfig:
		{
			setConfigPacket* packet = (setConfigPacket*) data;
			uint16_t profileidx = packet->opt_number;
			uint8_t profilePage = packet->opt_page;
			if(debugMode) printf("setprofile %dp%d\r\n", profileidx, profilePage);
			simucubelog.addEventParam(Command_setProfile, profileidx);
			simucubelog.addEventParam(Command_SetProfilePage, profilePage);
			if(profileidx==0) {
				// do not overwrite the 0 profile
				return false;
				break;
			}
#if 0
			memcpy(mConfig.GetProfileConfigAddr(profileidx), &packet->data[0], sizeof(profData));
#endif
#if 1
			uint8_t* target = mConfig.GetProfileConfigAddr(profileidx)+profilePage*profileDataByteSplitIndex;
			uint8_t* source = &packet->data[0];
			// count number of valid bytes
			uint16_t profdatasize = sizeof(profData);
			uint8_t validbytes;
			uint16_t fulldata = profileDataByteSplitIndex*(profilePage+1);
			if(fulldata>profdatasize) {
				validbytes = fulldata-profdatasize;
			} else {
				validbytes=profileDataByteSplitIndex;
			}
			memcpy(target, source, validbytes);
#endif
			if (profileidx != currentprofileindex) {
				unsavedSettings = 1;
				return true;
				break;
			}
			setCurrentProfileMMC();
			currentSystemStatus = BeforeOperational; // on current profile config change, need to init at least steering angles again.
			temporarySteeringAngleIsSet = false;
			unsavedSettings = 1;
			return false;
			break;
		}

		// THIS WORKS
		case setAnalogConfig:
		{
			if(debugMode) printf("setanalog\r\n");
			simucubelog.addEvent(Command_setAnalog);
			setConfigPacket* packet = (setConfigPacket*) data;
			memcpy(mConfig.GetAnalogConfigAddr(),&packet->data[0], sizeof(analogData));
			//currentSystemStatus = DriveInit; // on profile config change, need to init at least steering angles again.
			// so go to BeforeOperational.
			//currentSystemStatus = BeforeOperational;
			//mConfig.analogConfig
			unsavedSettings = 1;
			return true;
			break;
		}

		case setIoniConfig:
		{
			if(debugMode) printf("setIoniConfig\r\n");
			simucubelog.addEvent(Command_setIoniConfig);
			setIoniConfigPacket* packet = (setIoniConfigPacket*) data;
			for(int i=0; i<packet->numberOfValues; i++) {
				smSetParameter(mSMBusHandle, 1, packet->parameters[i], packet->values[i]);
			}
			//todo: check if drive restart is needed!
			return false;
			break;
		}


		case transferIoniDrcData:
		{
			if(debugMode) printf("ionidrcdata");

			transferIoniDrcDataPacket* packet = (transferIoniDrcDataPacket*) data;
			simucubelog.addEventParam(294, IoniDrcDataReceivedBytes, true);
			if(packet->firstpacket == 1) {
				IoniDrcDataReceivedBytes = 0; // first packet
			}
			memcpy(&IoniDrcFile[IoniDrcDataReceivedBytes], &packet->data[0], packet->lastByte);
			IoniDrcDataReceivedBytes+= packet->lastByte;
			if(debugMode) printf(" %d\r\n",IoniDrcDataReceivedBytes);
			simucubelog.addEventParam(Command_tranferIoniDRCData, IoniDrcDataReceivedBytes);
			HAL_Delay(1);
			return true;
			break;
		}

		case applyIoniDrcData:
		{
			if(debugMode) printf("applyionidrcdata\r\n");
			simucubelog.addEvent(Command_ApplyIoniConfig);
			if(firmwareuploadinprogress) {
				simucubelog.addEventParam(Command_ApplyIoniConfigFail,0);
				if(debugMode2) printf("ioni firmware upgrading!! can't do this.\r\n");
				return true;
				break;
			}
			commandPacket* packet = (commandPacket*) data;
			if(packet->value != IoniDrcDataReceivedBytes) {
				// mismatch of received packets vs. expected
				currentSystemStatus = IoniDrcError;
				simucubelog.addEventParam(Command_ApplyIoniConfigFail,1);
				IoniDrcDataReceivedBytes = 0; // reset pointer to enable retransmissions
				return false;
				break;
			}
			else {
				// success. Can transfer Ioni Data to Ioni.
				currentSystemStatus = ApplyIoniDrcData;
				return false;
				break;
			}

		}
		case freshStart:
			if(debugMode) printf("freshstart\r\n");
			simucubelog.addEvent(Command_goFreshStart);
			currentSystemStatus = goFreshStart;
			return true;
			break;

		case enableSMUSB:
			if(debugMode) printf("enablesmusb\r\n");
			simucubelog.addEvent(Command_enableSMUSB);
			// set wheel forces to off!
			SetTorque(0);
			nextSystemStatus = currentSystemStatus;
			currentSystemStatus = releaseSMBus;
			return false;
			break;

		case disableSMUSB:
			if(debugMode) printf("disablesmusb\r\n");
			simucubelog.addEvent(Command_disableSMUSB);
			currentSystemStatus = regainSMBus;
			return false;
			break;

		case requestRawAnalogAxis:
			if(debugMode) printf("requestrawanalog\r\n");
			simucubelog.addEvent(Command_setRawAnalogMode);
			rawAnalogMode = true;
			return true;
			break;

		case requestCalibratedAnalogAxis:
			if(debugMode) printf("requestcalibratedanalog\r\n");
			simucubelog.addEvent(Command_setCalibAnalogMode);
			rawAnalogMode = false;
			return true;
			break;

		case startDriveInit:
			if(debugMode) printf("Going to init drive!\r\n");
			simucubelog.addEvent(Command_startDriveInit);
			driveInitSuccess=false;
			currentSystemStatus = DriveInit;
			return false;
			break;

		case restartDrive:
			if(debugMode) printf("restart drive command!\r\n");
			simucubelog.addEvent(Command_restartDrive);
			smSetParameter(mSMBusHandle, 1, SMP_SYSTEM_CONTROL, SMP_SYSTEM_CONTROL_RESTART);
			HAL_Delay(500);
			currentSystemStatus = SystemNotConfigured;
			return false;
			break;

		case setInitialConfigDone:
			if(debugMode) printf("setting initial config to done!\r\n");
			simucubelog.addEvent(Command_setInitialConfig);
			mConfig.hardwareConfig.mInitialConfigDone = InitialConfigDone;
			if(currentSystemStatus == DriveInitSuccessPause) {
				currentSystemStatus = BeforeOperational;
				return false;
			} else {
				return true;
			}
			break;

		case clearInitialConfigDone:
			if(debugMode) printf("clearing initial config, going to notconfigured mode!\r\n");
			simucubelog.addEvent(Command_unsetInitialConfig);
			mConfig.hardwareConfig.mInitialConfigDone = InitialConfigNotDone;
			currentSystemStatus = SystemNotConfigured;
			nextSystemStatus = SystemNotConfigured;
			temporaryCenterPoint = false;
			indexpointfound = 0;
			return false;
			break;
#if 0
		case connectDriveCommand:
			if(debugMode) printf("Only connecting drive right now\r\n");
			nextSystemStatus = currentSystemStatus;
			currentSystemStatus = Driveconnect;
			return true;
			break;
#endif
		case setWheelCenterHere:
		{
			if(debugMode) printf("setwheelcenterhere\r\n");
			simucubelog.addEvent(Command_setWheelCenter);
			setWheelCenter();
			return true;
			break;
		}

		case setTemporaryCenterMode:
			if(debugMode) printf("setting temporary centering for this session\r\n");
			simucubelog.addEvent(Command_setTempCenter);
			temporaryCenterPoint = true;
			return true;
			break;

		case unsetTemporaryCenterMode:
			if(debugMode) printf("unsetting temporary centering for this session\r\n");
			simucubelog.addEvent(Command_unsetTempCenter);
			temporaryCenterPoint = false;
			return true;
			break;

		case activateProfile:
		{
			commandPacket* packet = (commandPacket*) data;
			currentprofileindex = packet->value;
			if(debugMode) printf("activating profile index %u\r\n", currentprofileindex);
			simucubelog.addEventParam(Command_actProfile,currentprofileindex);
			temporarySteeringAngleIsSet = false;
			if(currentSystemStatus==Operational) {
				currentSystemStatus = BeforeOperational; // on profile config change, need to init at least steering angles again.
			}
			return false;
			break;
		}

		case setNumProfiles:
		{
			commandPacket* packet = (commandPacket*) data;
			if(packet->value < 1) {
				numberofprofiles = 1;
			} else if (numberofprofiles>500) {
				numberofprofiles = 500;
			} else {
				numberofprofiles = packet->value;
			}
			if(debugMode) printf("setting number of profiles to %u\r\n", numberofprofiles);
			simucubelog.addEventParam(Command_setNumProfiles,numberofprofiles);
			unsavedSettings = 1;
			return true;
			break;

		}

		case setDefaultProfile:
		{
			commandPacket* packet = (commandPacket*) data;
			uint16_t value = packet->value;
			if(value>500 || value < 1) {
				defaultprofileindex = 0;
			} else {
				defaultprofileindex = packet->value;
			}
			if(debugMode) printf("setting default profile index to %u\r\n", defaultprofileindex);
			simucubelog.addEventParam(Command_setDefProfile,defaultprofileindex);
			unsavedSettings = 1;
			return true;
			break;
		}




		case readIoniFiltersToProfile:
			if(debugMode) printf("reading Ioni filter settings to profile\r\n");
			simucubelog.addEvent(Command_readIoniToProf);
			smint32 temp;
			smRead1Parameter(mSMBusHandle, 1, SMP_TORQUE_LPF_BANDWIDTH, &temp);
			mConfig.profileConfigs[currentprofileindex].ioni_lpf = (uint8_t) temp;
			smRead1Parameter(mSMBusHandle, 1, SMP_TORQUE_NOTCH_FILTER, &temp);
			mConfig.profileConfigs[currentprofileindex].ioni_notch = temp;
			smRead1Parameter(mSMBusHandle, 1, SMP_TORQUE_EFFECT_DAMPING, &temp);
			mConfig.profileConfigs[currentprofileindex].ioni_damping = (uint16_t) temp;
			smRead1Parameter(mSMBusHandle, 1, SMP_TORQUE_EFFECT_FRICTION, &temp);
			mConfig.profileConfigs[currentprofileindex].ioni_friction = (uint16_t) temp;
			smRead1Parameter(mSMBusHandle, 1, SMP_TORQUE_EFFECT_INERTIA, &temp);
			mConfig.profileConfigs[currentprofileindex].ioni_inertia = (uint16_t) temp;
			smRead1Parameter(mSMBusHandle, 1, SMP_SETPOINT_FILTER_MODE, &temp);
			mConfig.profileConfigs[currentprofileindex].ioni_filter1 = (uint8_t) temp;
			return true;
			break;

		case jmpToBootloader:
			if(debugMode) printf("jumping to bootloader\r\n");
			simucubelog.addEvent(Command_jmpToBootLoader);
			currentSystemStatus = JumpToBootLoader;
			return false;
			break;

		case setForcesDisabled:
			if(debugMode) printf("set forces disabled\r\n");
			simucubelog.addEvent(Command_disableForces);
			forcesEnabled = false;
			return true;
			break;

		case setForcesEnabled:
			if(debugMode) printf("set forces enabled\r\n");
			simucubelog.addEvent(Command_enableForces);
			forcesEnabled = true;
			return true;
			break;

		case resetFFBvariables:
			if(debugMode) printf("force-reset ffb device\r\n");
			simucubelog.addEvent(Command_resetFFBStates);
			resetFFBDeviceState();
			return true;
			break;

		case startCommutationAutoSetup:
			if(debugMode) printf("going to autosetup commucation\r\n");
			simucubelog.addEvent(Command_setupCommutation);
			previousSystemStatus2 = currentSystemStatus;
			currentSystemStatus = AutoDetectCommutationSensors;
			return false;
			break;

		case clearCommutationAutoSetup:
			if(debugMode) printf("clear autosetup commucation\r\n");
			simucubelog.addEvent(Command_clearCommutation);
			clearCommutationConfig(); //also saves drive cfg
			mConfig.hardwareConfig.mAutoCommutationMode=0;
			mConfig.hardwareConfig.mIndexingMode=startAtCenterPhasedIndexing;
			unsavedSettings=1;
			return false;
			break;

		case setTemporaryVariable:
			{
			if(debugMode) printf("set temporary variable\r\n");
			commandPacket* packet = (commandPacket*) data;
			uint16_t parameter = packet->value;
			switch(parameter)
			{
				case temporarySteeringAngle:
					temporarySteeringAngleIsSet = true;
					temporarySteeringLockToLock = packet->value2;
					currentSystemStatus = BeforeOperational;
					return false;
					break;
				case unsetTemporarySteeringAngle:
					temporarySteeringAngleIsSet = false;
					currentSystemStatus = BeforeOperational;
					return false;
					break;
				default:
					break;

				}
			}
		default:
			break;
	}
	return true;
}



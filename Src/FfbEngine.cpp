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
 * FfbEngine.cpp
 *
 *  Created on: Feb 21, 2017
 *      Author: Mika
 */

#include "FfbEngine.h"
#include "stdio.h"
#include "usb_device.h"


#include "config_comm_defines.h"
#include "cHardwareConfig.h"
#include "cProfileConfig.h"
#include "USBGameController.h"
//#include "command.h"
#include "eventLog.h"

//#define ffbDebug1 1
//#define ffbDebug2 1
extern bool debugMode;
extern USBGameController joystick;
extern uint64_t millis;
extern eventLog simucubelog;

FfbEngine::FfbEngine() {
	FreeAllEffects();
	printf("..\r\n");
}

FfbEngine::~FfbEngine() {
	// Auto-generated destructor stub
}

#define LEDs_SetAllLEDs(l)

volatile cEffectState* FfbEngine::getEffectState(u8 id) {
	return &gEffectStates[id];
}


void FfbEngine::SendPidStateForEffect(uint8_t eid, uint8_t effectState)
{
	pidState.effectBlockIndex = effectState;
	pidState.effectBlockIndex = 0;
}


uint8_t FfbEngine::GetNextFreeEffect(void)
{

#ifdef ffbDebug1
	printf("find free eff\r\n");
#endif
	simucubelog.addEvent(FFB_FindFreeEffect, true);
	// Find the next free effect ID for next time
	for (u8 i = FIRST_EID; i <= MAX_EFFECTS; i++)
	{
		if (gEffectStates[i].state == MEffectState_Free)
		{
#ifdef ffbDebug1
	printf("next free eff: %d\r\n", i);
#endif
			//printf("4\r\n");
			simucubelog.addEventParam(FFB_FoundFreeEffectSlot, i, true);
			gEffectStates[i].state = MEffectState_Allocated;
			return i;
		}
	}
	return 0;
 }

void FfbEngine::StopAllEffects(void)
{
#ifdef ffbDebug1
	printf("stopalleffs\r\n");
#endif
	simucubelog.addEvent(FFB_StopAllEffects, true);
	//SetTorque(0);
	for (uint8_t id = FIRST_EID; id <= MAX_EFFECTS; id++)

		StopEffect(id);

}

void FfbEngine::StartEffect(uint8_t id)
{
#ifdef ffbDebug1
	printf("st eff %d\r\n",id);

#endif
	simucubelog.addEventParam(FFB_StartEffect, id, true);
	if ((id > MAX_EFFECTS) || (gEffectStates[id].state==MEffectState_Free)) {
#ifdef ffbDebug1
		printf("too high effectid or not allocated\r\n");
#endif
		simucubelog.addEventParam(FFB_InvalidEffect, id, true);
		return;
	}
	InitEffect(id);
	gEffectStates[id].state = MEffectState_Playing;
	gEffectStates[id].starttime = millis;
}

void FfbEngine::StopEffect(uint8_t id)
{
#ifdef ffbDebug1
	printf("stop eff %d\r\n",id);
#endif
	simucubelog.addEventParam(FFB_StopEffect, id, true);
	if ((id > MAX_EFFECTS) || (gEffectStates[id].state == 0))
		return;
	//gEffectStates[id].state &= ~MEffectState_Playing;
	gEffectStates[id].state = MEffectState_Allocated;
}

void FfbEngine::FreeEffect(uint8_t id)
{
	simucubelog.addEventParam(FFB_FreeEffect, id, true);
#ifdef ffbDebug1

#endif
	if (id > MAX_EFFECTS)
		return;

	//debugPrint(DMid, "Free effect id %d",id);
	//LogBinaryLf(&id,1);
	gEffectStates[id].state = MEffectState_Free;
}

void FfbEngine::FreeAllEffects(void)
{
	//SetTorque(0);
#ifdef ffbDebug1
	printf("fr allr\n");
#endif
	simucubelog.addEvent(FFB_FreeAllEffects, true);
	//LogTextLf("Free All effects");
 	//memset((void*) gEffectStates, 0, sizeof(gEffectStates));
	for(int i=FIRST_EID; i< MAX_EFFECTS; i++) {
		if(i==1) {
			//printf("??\r\n");
		}
		gEffectStates[i].attackLevel = 0;
		gEffectStates[i].attackTime = 0;
		gEffectStates[i].counter = 0;
		gEffectStates[i].duration = 0;
		gEffectStates[i].end_mag = 0;
		gEffectStates[i].fadeLevel = 0;
		gEffectStates[i].fade_start_time =0;
		gEffectStates[i].fcounter = 0.0;
		gEffectStates[i].freq = 0.0;
		gEffectStates[i].gain = 0;
		gEffectStates[i].magnitude = 0;
		gEffectStates[i].negativeSaturation = 0;
		gEffectStates[i].offset = 0;
		gEffectStates[i].pad0 = 0;
		gEffectStates[i].period = 0;
		gEffectStates[i].phase = 0;
		gEffectStates[i].positiveSaturation = 0;
		gEffectStates[i].start_mag = 0;
		gEffectStates[i].state = MEffectState_Free;
		gEffectStates[i].type = 0xFF;
	}
}


// Handle incoming data from USB
bool FfbEngine::handleReceivedHIDReport(HID_REPORT report)
{
	u8 *data = report.data;
	uint8_t effectId = data[1]; // effectBlockIndex is always the second byte.
#ifdef ffbDebug1
	//printf("HID FFB input rep\r\n");
#endif


	// if got this far (wasn't a simucube-specific command)
	// check for ffb report and handle them.

	switch (data[0])	// reportID
	{
	case 1:
#ifdef ffbDebug1
		printf("SEFF:");
#endif
		FfbHandle_SetEffect((USB_FFBReport_SetEffect_Output_Data_t *) data);

		break;
	case 2:
#ifdef ffbDebug1
		printf("ENVCMD\r\n");
#endif
		SetEnvelope((USB_FFBReport_SetEnvelope_Output_Data_t*) data, effectId);
		break;
	case 3:
#ifdef ffbDebug1
		printf("SCOND:");
#endif
		SetConditional((USB_FFBReport_SetCondition_Output_Data_t*) data, effectId);
		break;
	case 4:
#ifdef ffbDebug1
		printf("SPER\r\n");
#endif
		SetPeriodic((USB_FFBReport_SetPeriodic_Output_Data_t*) data, effectId);
		break;
	case 5:
	{
		SetConstantForce((USB_FFBReport_SetConstantForce_Output_Data_t*) data, effectId);
		break;
	}
	case 6:
#ifdef ffbDebug1
		printf("SRAMP\r\n");
#endif
		SetRampForce((USB_FFBReport_SetRampForce_Output_Data_t*)data, effectId);
		break;
	case 7:
#ifdef ffbDebug1
		printf("SCUSFDT\r\n");
#endif
		FfbHandle_SetCustomForceData((USB_FFBReport_SetCustomForceData_Output_Data_t*) data);
		break;
	case 8:
#ifdef ffbDebug1
		printf("SDWNLDFSMP\r\n");
#endif
		FfbHandle_SetDownloadForceSample((USB_FFBReport_SetDownloadForceSample_Output_Data_t*) data);
		break;
	case 9:
#ifdef ffbDebug1
		printf("C9donothing\r\n");
#endif
		break;
	case 10:
#ifdef ffbDebug1
		printf("EFOP\r\n");
#endif
		FfbHandle_EffectOperation((USB_FFBReport_EffectOperation_Output_Data_t*) data);
		break;
	case 11:
#ifdef ffbDebug1
		printf("BLFR\r\n");
#endif
		FfbHandle_BlockFree((USB_FFBReport_BlockFree_Output_Data_t *) data);
		break;
	case 12:
#ifdef ffbDebug1
		printf("DC\r\n");
#endif
		FfbHandle_DeviceControl((USB_FFBReport_DeviceControl_Output_Data_t*) data);
		break;
	case 13:
#ifdef ffbDebug1
		printf("DG\r\n");
#endif
		FfbHandle_DeviceGain((USB_FFBReport_DeviceGain_Output_Data_t*) data);
		break;
	case 14:
		FfbHandle_SetCustomForce((USB_FFBReport_SetCustomForce_Output_Data_t*) data);
		break;
	default:
		break;
	}
	return true;
}

void FfbEngine::FfbOnCreateNewEffect (USB_FFBReport_CreateNewEffect_Feature_Data_t* inData, USB_FFBReport_PIDBlockLoad_Feature_Data_t *outData)
{
	outData->reportId = 6;//2;//= 6;
	//printf("3\r\n");
	__disable_irq();
	outData->effectBlockIndex = GetNextFreeEffect();
	__enable_irq();
	//printf("5\r\n");
	if (outData->effectBlockIndex == 0)
	{
		outData->loadStatus = 2;	// 1=Success,2=Full,3=Error
		//LogText("Could not create effect");
		simucubelog.addEvent(FFB_MemFull, true);
	}
	else
	{
		outData->loadStatus = 1;	// 1=Success,2=Full,3=Error
		printf("new eff: slot %d, type: %d\r\n", outData->effectBlockIndex, inData->effectType);
		simucubelog.addEventParam(FFB_CreatedNewEffect, inData->effectType, true);
#ifdef ffbDebug1
		printf("new eff: slot %d, type: %d\r\n", outData->effectBlockIndex, inData->effectType);
#endif
		//printf("7\r\n");
		CreateNewEffect(inData, outData->effectBlockIndex);
		//printf("8\r\n");
		//LogText("Created effect ");
		//LogBinary(&outData->effectBlockIndex,1);
		//LogText(", type ");
		//LogBinaryLf(&inData->effectType,1);
	}
	outData->ramPoolAvailable = 0x01FF;//0xFFFF;	// =0 or 0xFFFF - don't really know what this is used for?
//	WaitMs(5);
}

void FfbEngine::FfbHandle_SetEffect(USB_FFBReport_SetEffect_Output_Data_t *data)
{
	SetEffect(data,data->effectBlockIndex);
}

void FfbEngine::FfbOnPIDPool(USB_FFBReport_PIDPool_Feature_Data_t *data)
{
#ifdef ffbDebug1
	printf("ONPDPL\r\n");
#endif
	FreeAllEffects();
	printf("**\r\n");
	data->reportId = 7;
	data->ramPoolSize = 0xFFFF;
	data->maxSimultaneousEffects = MAX_EFFECTS;
	data->memoryManagement = 3;
}

void FfbEngine::FfbHandle_SetCustomForceData(USB_FFBReport_SetCustomForceData_Output_Data_t *data)
{
	//debugPrint(DMid, "SetCustomForceData");
}

void FfbEngine::FfbHandle_SetDownloadForceSample(USB_FFBReport_SetDownloadForceSample_Output_Data_t *data)
{
	//debugPrint(DMid, "SetDownloadForceSample");
}

void FfbEngine::FfbHandle_EffectOperation(USB_FFBReport_EffectOperation_Output_Data_t *data)
{
	uint8_t eid = data->effectBlockIndex;

	if (eid == 0xFF)
		eid = 0x7F;	// All effects

	if (data->operation == 1)
	{	// Start
		//debugPrint(DLow,"Start effect id %d",eid);
		StartEffect(eid);
	}
	else if (data->operation == 2)
	{	// StartSolo
		// Stop all first
		//debugPrint(DLow,"Start solo effect id %d",eid);
		StopAllEffects();
		// Then start the given effect
		StartEffect(eid);
	}
	else if (data->operation == 3)
	{	// Stop
		//debugPrint(DLow,"Stop effect id %d",eid);
		StopEffect(eid);
	}
}


void FfbEngine::FfbHandle_BlockFree (USB_FFBReport_BlockFree_Output_Data_t *data)
{
	uint8_t eid = data->effectBlockIndex;

	if (eid == 0xFF)
	{	// all effects
#ifdef ffbDebug1
		printf("BLCK_FRALL\r\n");
#endif
		FreeAllEffects();
		printf(",,\r\n");
// 		FreeEffect(0x7f); // Question: does this work with the wheel?
	}
	else
	{
		if(eid==1) {
			asm("nop");
		}
		FreeEffect(eid);
	}
}

#define DEVICE_PAUSED			0x01
#define ACTUATORS_ENABLED		0x02
#define SAFETY_SWITCH			0x04
#define ACTUATOR_OVERRIDE		0x08
#define ACTUATOR_POWER			0x10


void FfbEngine::FfbHandle_DeviceControl(USB_FFBReport_DeviceControl_Output_Data_t *data)
{
	//	LogTextP(PSTR("Device Control: "));

	uint8_t control = data->control;
	// 1=Enable Actuators, 2=Disable Actuators, 3=Stop All Effects, 4=Reset, 5=Pause, 6=Continue

	// PID State Report:
	//	uint8_t	reportId;	// =2
	//	uint8_t	status;	// Bits: 0=Device Paused,1=Actuators Enabled,2=Safety Switch,3=Actuator Override Switch,4=Actuator Power
	//	uint8_t	effectBlockIndex;	// Bit7=Effect Playing, Bit0..7=EffectId (1..40)

	pidState.reportId = 2;
	Bset(pidState.status,SAFETY_SWITCH);
	Bset(pidState.status,ACTUATOR_POWER);
	pidState.effectBlockIndex = 0;

	switch (control)
	{
	case 0x01:
		//LogTextLf("Disable Actuators");
		Bclr(pidState.status,ACTUATORS_ENABLED);
		break;
	case 0x02:
		//LogTextLf("Enable Actuators");
		Bset(pidState.status,ACTUATORS_ENABLED);
		break;
	case 0x03:
		//LogTextLf("Stop All Effects");		// Disable auto-center spring and stop all effects
		SetAutoCenter(0);
		pidState.effectBlockIndex = 0;
		break;
	case 0x04:
		//LogTextLf("Reset");			// Reset (e.g. FFB-application out of focus)
		//SetAutoCenter(1);		// Enable auto-center spring and stop all effects
//		WaitMs(75);
#ifdef ffbDebug1
		printf("DCTRL_FREE\r\n");
#endif
		FreeAllEffects();
#ifdef ffbDebug1
		printf("--\r\n");
#endif
		break;
	case 0x05:
		//LogTextLf("Pause");
		Bset(pidState.status,DEVICE_PAUSED);
		break;
	case 0x06:
		//LogTextLf("Continue");
		Bclr(pidState.status,DEVICE_PAUSED);
		break;
	default:
		//LogTextP(PSTR("Other "));
		//LogBinaryLf(&data->control,1);
		asm("nop");
	}
}

void FfbEngine::FfbHandle_DeviceGain(USB_FFBReport_DeviceGain_Output_Data_t *data)
{
	//LogTextP(PSTR("Device Gain: "));
	//LogBinaryLf(&data->gain, 1);
	//printf("g %d\r\n", data->gain);
	joystick.gFFBDevice.ffbDevGain = data->gain;
	simucubelog.addEventParam(FFB_SetGain, data->gain, true);
}

void FfbEngine::FfbHandle_SetCustomForce(USB_FFBReport_SetCustomForce_Output_Data_t *data)
{
	//LogTextLf("Set Custom Force");
//	LogBinary(&data, sizeof(USB_FFBReport_SetCustomForce_Output_Data_t));
}


//from FFB_PRO
void FfbEngine::SetAutoCenter(uint8_t enable)
{
}

// --------------------------- effect operations ---------------------------------------------------

void FfbEngine::InitEffect(uint8_t effectId)
{
	volatile cEffectState* effect = &gEffectStates[effectId];
	effect->counter = 0;
	effect->fcounter = 0;
	effect->fade_start_time = effect->duration - effect->fadeTime;
}

// modify operations ---------------------------------------------------------

void FfbEngine::ModifyDuration(uint8_t effectId, uint16_t duration)
{
	volatile cEffectState* effect = &gEffectStates[effectId];
	effect->duration = duration;
	effect->counter = 0;
	effect->fade_start_time = duration - effect->fadeTime;

	//debugPrint(DMid,"ModifyDuration eid=%d %d %d %d",effectId,effect->duration, effect->counter, effect->fade_start_time);
}

void FfbEngine::SetEnvelope (USB_FFBReport_SetEnvelope_Output_Data_t* data, int effectId)
{
	volatile cEffectState* effect=&gEffectStates[effectId];
	effect->attackLevel = data->attackLevel;
	effect->fadeLevel = data->fadeLevel;
	effect->attackTime = data->attackTime;
	effect->fadeTime = data->fadeTime;
	//debugPrint(DMid,"SetEnvelope eid=%d %d %d %d %d",effectId,data->attackLevel,data->fadeLevel,data->attackTime,data->fadeTime);
}

void FfbEngine::SetConditional (USB_FFBReport_SetCondition_Output_Data_t* data, int effectId)
{
#ifdef ffbDebug1
	printf("%d\r\n",effectId);
#endif
	volatile cEffectState* effect=&gEffectStates[effectId];

	effect->magnitude = (s16)data->positiveCoefficient;
	effect->offset = data->cpOffset;
	effect->positiveSaturation = data->positiveSaturation;
	effect->negativeSaturation = data->negativeSaturation;

	//debugPrint(DMid,"SetCondition eid=%d %d %d %d %d",effectId,effect->magnitude, effect->offset, effect->positiveSaturation, effect->negativeSaturation);
}

void FfbEngine::SetPeriodic (USB_FFBReport_SetPeriodic_Output_Data_t* data, int effectId)
{
	volatile cEffectState* effect=&gEffectStates[effectId];

	effect->magnitude = (s16)data->magnitude;
	effect->offset = data->offset;
	effect->phase = (s16)data->phase;
	effect->period = (u16)data->period;
	effect->freq = 0.0f;

	//debugPrint(DMid,"SetPeriodic eid=%d %d %d %d %d %d",effectId,effect->magnitude, effect->offset, effect->phase, effect->period, effect->freq);
}

void FfbEngine::SetConstantForce (USB_FFBReport_SetConstantForce_Output_Data_t* data, int effectId)
{
	volatile cEffectState* effect=&gEffectStates[effectId];

	effect->magnitude = data->magnitude;
}

void FfbEngine::SetRampForce (USB_FFBReport_SetRampForce_Output_Data_t* data, int effectId )
{
	volatile cEffectState* effect=&gEffectStates[effectId];

	effect->start_mag = data->start;
	effect->end_mag= data->end;

	//debugPrint(DMid,"SetRampForce eid=%d %d %d",effectId,effect->start_mag, effect->end_mag);
}

void FfbEngine::SetEffect (USB_FFBReport_SetEffect_Output_Data_t *data, int effectId )
{
	volatile cEffectState* effect=&gEffectStates[effectId];


	uint8_t eid = data->effectBlockIndex;
	effect->type = data->effectType;
	effect->gain = data->gain;
	ModifyDuration(eid,data->duration); // 0..32767 ms

//	bool is_periodic = false;

	char const *type;
#ifdef ffbDebug1
	printf("%d:",effectId);
#endif
	// Fill in the effect type specific data
	switch (data->effectType)
	{
		case USB_EFFECT_SQUARE:
#ifdef ffbDebug1
			printf("SQ_EFF\r\n");
#endif
			type="SQUARE";
		break;
		case USB_EFFECT_SINE:
#ifdef ffbDebug1
			printf("SIN_EFF\r\n");
#endif
			type="SINE";
		break;
		case USB_EFFECT_TRIANGLE:
#ifdef ffbDebug1
			printf("TRI_EFF\r\n");
#endif
			type="TRIANGLE";
		break;

		case USB_EFFECT_SAWTOOTHDOWN:
#ifdef ffbDebug1
			printf("SAW_DN_EFF\r\n");
#endif
			type="SAWTOOTH";
		break;

		case USB_EFFECT_SAWTOOTHUP:
#ifdef ffbDebug1
			printf("SAW_UP_EFF\r\n");
#endif
			type="SAWTOOTHUP";
		break;
//			is_periodic = true;
		case USB_EFFECT_CONSTANT:
#ifdef ffbDebug1
			printf("CONS_EFF\r\n");
#endif
			type="CONSTANT";
		break;

		case USB_EFFECT_RAMP:
#ifdef ffbDebug1
			printf("RAMP_EFF\r\n");
#endif
			type="RAMP";
		break;

		case USB_EFFECT_SPRING:
#ifdef ffbDebug1
			printf("SPR_EFFt\r\n");
#endif
			type="SPRING";
		break;

		case USB_EFFECT_DAMPER:
#ifdef ffbDebug1
			printf("DMP_EFF\r\n");
#endif
			type="DAMPER";
		break;
		case USB_EFFECT_INERTIA:
#ifdef ffbDebug1
			printf("INR_EFF\r\n");
#endif
			type="INERTIA";
		break;

		case USB_EFFECT_FRICTION:
#ifdef ffbDebug1
			printf("FRI_EFF\r\n");
#endif
			type="FRICTION";
		break;

		case USB_EFFECT_CUSTOM:
			effect->period = data->samplePeriod;	// 0..32767 ms
#ifdef ffbDebug1
			printf("CUS_EFF\r\n");
#endif
			type="CUSTOM";

		break;

		default:
			type="UNKNOWN EFFECT";
#ifdef ffbDebug1
			printf("unknown effect\r\n");
#endif
		break;
	}
	//debugPrint(DMid,"SetEffect eid=%d %s gain %d duration %d",effectId, type,data->gain,data->duration);
}

void FfbEngine::CreateNewEffect(USB_FFBReport_CreateNewEffect_Feature_Data_t* inData, int effectId )
{
	volatile cEffectState* effect=&gEffectStates[effectId];

	effect->type = inData->effectType;
	effect->gain = 0xFF;
	effect->attackLevel = 0xFF;
	effect->fadeLevel = 0xFF;

	effect->positiveSaturation = 0;
	effect->negativeSaturation = 0;
	effect->magnitude = 0;
	effect->phase = 0;
	effect->offset = 0;
	effect->period = 100;
	effect->duration = USB_DURATION_INFINITE;
	effect->fadeTime = USB_DURATION_INFINITE;

	//bdebugPrint(DHigh,"CreateNewEffect eid=%d",effectId);
}



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

extern bool debugMode;
extern USBGameController joystick;
extern uint64_t millis;
extern eventLog simucubelog;

#define FFBENGINEDEBUG 0
#if FFBENGINEDEBUG
#define ffbprintf(...) printf(__VA_ARGS__);
#else
#define ffbprintf(...)
#endif


FfbEngine::FfbEngine() {
	FreeAllEffects();
}

FfbEngine::~FfbEngine() {
	// Auto-generated destructor stub
}

volatile cEffectState* FfbEngine::getEffectState(uint8_t id) {
	return &gEffectStates[id];
}

void FfbEngine::SendPidStateForEffect(uint8_t eid, uint8_t effectState)
{
	pidState.effectBlockIndex = effectState;
	pidState.effectBlockIndex = 0;
}


uint8_t FfbEngine::GetNextFreeEffect(void)
{
	simucubelog.addEvent(FFB_FindFreeEffect, true);
	ffbprintf("find free eff\r\n");
	// Find the next free effect ID for next time
	for (uint8_t i = FIRST_EID; i <= MAX_EFFECTS; i++)
	{
		if (gEffectStates[i].state == MEffectState_Free)
		{
			ffbprintf("found free eff %d\r\n", i);
			simucubelog.addEventParam(FFB_FoundFreeEffectSlot, i, true);
			gEffectStates[i].state = MEffectState_Allocated;
			return i;
		}
	}
	return 0;
 }

void FfbEngine::StopAllEffects(void)
{
	simucubelog.addEvent(FFB_StopAllEffects, true);
	ffbprintf("stop all\r\n");
	for (uint8_t id = FIRST_EID; id <= MAX_EFFECTS; id++) {
		StopEffect(id);
	}
}

void FfbEngine::StartEffect(uint8_t id)
{
	simucubelog.addEventParam(FFB_StartEffect, id, true);
	ffbprintf("start eff %d\r\n", id);
	if ((id == 0) || (id > MAX_EFFECTS) || (gEffectStates[id].state == MEffectState_Free)) {
		simucubelog.addEventParam(FFB_InvalidEffect, id, true);
		return;
	}
	InitEffect(id);
	gEffectStates[id].state = MEffectState_Playing;
	gEffectStates[id].starttime = millis;
}

void FfbEngine::StopEffect(uint8_t id)
{
	simucubelog.addEventParam(FFB_StopEffect, id, true);
	ffbprintf("stop eff %d\r\n", id);
	if ((id > MAX_EFFECTS) || (gEffectStates[id].state == 0)) {
		return;
	}
	gEffectStates[id].state = MEffectState_Allocated;
}

void FfbEngine::FreeEffect(uint8_t id)
{
	simucubelog.addEventParam(FFB_FreeEffect, id, true);
	ffbprintf("free eff %d\r\n", id);
	if (id == 0 || id > MAX_EFFECTS) {
		return;
	}
	gEffectStates[id].state = MEffectState_Free;
}

void FfbEngine::FreeAllEffects(void)
{
	simucubelog.addEvent(FFB_FreeAllEffects, true);
	ffbprintf("free all eff\r\n");
	for(int i=FIRST_EID; i <= MAX_EFFECTS; i++) {
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
bool FfbEngine::handleReceivedHIDReport(const uint8_t* data)
{
	uint8_t effectId = data[1]; // effectBlockIndex is always the second byte.

	// if got this far (wasn't a simucube-specific command)
	// check for ffb report and handle them.

	switch (data[0])	// reportID
	{
	case 1:
		ffbprintf("seteff\r\n");
		FfbHandle_SetEffect((USB_FFBReport_SetEffect_Output_Data_t *) data);
		break;
	case 2:
		ffbprintf("setenv\r\n");
		SetEnvelope((USB_FFBReport_SetEnvelope_Output_Data_t*) data, effectId);
		break;
	case 3:
		ffbprintf("setcnd\r\n");
		SetConditional((USB_FFBReport_SetCondition_Output_Data_t*) data, effectId);
		break;
	case 4:
		ffbprintf("setper\r\n");
		SetPeriodic((USB_FFBReport_SetPeriodic_Output_Data_t*) data, effectId);
		break;
	case 5:
		//ffbprintf("setcon\r\n");
		SetConstantForce((USB_FFBReport_SetConstantForce_Output_Data_t*) data, effectId);
		break;
	case 6:
		ffbprintf("setrmp\r\n");
		SetRampForce((USB_FFBReport_SetRampForce_Output_Data_t*)data, effectId);
		break;
	case 7:
		ffbprintf("setcust\r\n");
		FfbHandle_SetCustomForceData((USB_FFBReport_SetCustomForceData_Output_Data_t*) data);
		break;
	case 8:
		ffbprintf("setdl\r\n");
		FfbHandle_SetDownloadForceSample((USB_FFBReport_SetDownloadForceSample_Output_Data_t*) data);
		break;
	case 10:
		ffbprintf("seteffop\r\n");
		FfbHandle_EffectOperation((USB_FFBReport_EffectOperation_Output_Data_t*) data);
		break;
	case 11:
		ffbprintf("setblkfree\r\n");
		FfbHandle_BlockFree((USB_FFBReport_BlockFree_Output_Data_t *) data);
		break;
	case 12:
		ffbprintf("setdevctrl\r\n");
		FfbHandle_DeviceControl((USB_FFBReport_DeviceControl_Output_Data_t*) data);
		break;
	case 13:
		ffbprintf("setgain\r\n");
		FfbHandle_DeviceGain((USB_FFBReport_DeviceGain_Output_Data_t*) data);
		break;
	case 14:
		ffbprintf("setcustom\r\n");
		FfbHandle_SetCustomForce((USB_FFBReport_SetCustomForce_Output_Data_t*) data);
		break;
	case 9: // no idea why 9 is noop
	default:
		break;
	}
	return true;
}

void FfbEngine::FfbOnCreateNewEffect (USB_FFBReport_CreateNewEffect_Feature_Data_t* inData, USB_FFBReport_PIDBlockLoad_Feature_Data_t *outData)
{
	outData->reportId = 6;
	// some operations happen directly from the interrupt handler
	// for some reason, so block interrupts to "lock" an effect index
	__disable_irq();
	outData->effectBlockIndex = GetNextFreeEffect();
	__enable_irq();
	if (outData->effectBlockIndex == 0) {
		outData->loadStatus = 2;	// 1=Success,2=Full,3=Error
		simucubelog.addEvent(FFB_MemFull, true);
	} else {
		outData->loadStatus = 1;	// 1=Success,2=Full,3=Error
		simucubelog.addEventParam(FFB_CreatedNewEffect, inData->effectType, true);
		ffbprintf("new eff: slot %d type %d\r\n", outData->effectBlockIndex, inData->effectType);
		CreateNewEffect(inData, outData->effectBlockIndex);
	}
	outData->ramPoolAvailable = 0x01FF;//0xFFFF;	// =0 or 0xFFFF - don't really know what this is used for?
}

void FfbEngine::FfbHandle_SetEffect(USB_FFBReport_SetEffect_Output_Data_t *data)
{
	SetEffect(data,data->effectBlockIndex);
}

void FfbEngine::FfbOnPIDPool(USB_FFBReport_PIDPool_Feature_Data_t *data)
{
	ffbprintf("ffb: onpidpool\r\n");
	FreeAllEffects();
	data->reportId = 7;
	data->ramPoolSize = 0xFFFF;
	data->maxSimultaneousEffects = MAX_EFFECTS;
	data->memoryManagement = 3;
}

void FfbEngine::FfbHandle_SetCustomForceData(USB_FFBReport_SetCustomForceData_Output_Data_t *data)
{

}

void FfbEngine::FfbHandle_SetDownloadForceSample(USB_FFBReport_SetDownloadForceSample_Output_Data_t *data)
{

}

void FfbEngine::FfbHandle_EffectOperation(USB_FFBReport_EffectOperation_Output_Data_t *data)
{
	uint8_t eid = data->effectBlockIndex;

	if (eid == 0xFF) {
		eid = 0x7F;	// All effects
	}

	switch (data->operation) {
	case 1:
	    // start all or one ... except that the StartEffect does not support such operation
	    StartEffect(eid);
	    break;
	case 2:
	    // start solo
	    StopAllEffects();
	    StartEffect(eid);
	    break;
	case 3:
	    // stop
	    StopEffect(eid);
	    break;
	default:
	    // not handled
	    break;
	}
}


void FfbEngine::FfbHandle_BlockFree (USB_FFBReport_BlockFree_Output_Data_t *data)
{
	uint8_t eid = data->effectBlockIndex;

	if (eid == 0xFF) {
		ffbprintf("freeall\r\n");
		FreeAllEffects();
	} else {
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
	// 1=Enable Actuators, 2=Disable Actuators, 3=Stop All Effects, 4=Reset, 5=Pause, 6=Continue

	// PID State Report:
	//	uint8_t	reportId;	// =2
	//	uint8_t	status;	// Bits: 0=Device Paused,1=Actuators Enabled,2=Safety Switch,3=Actuator Override Switch,4=Actuator Power
	//	uint8_t	effectBlockIndex;	// Bit7=Effect Playing, Bit0..7=EffectId (1..40)

	pidState.reportId = 2;
	Bset(pidState.status, SAFETY_SWITCH);
	Bset(pidState.status, ACTUATOR_POWER);
	pidState.effectBlockIndex = 0;

	switch (data->control)
	{
	case 0x01:
	    // disable actuators
	    Bclr(pidState.status, ACTUATORS_ENABLED);
		break;
	case 0x02:
	    // enable actuators
		Bset(pidState.status, ACTUATORS_ENABLED);
		break;
	case 0x03:
		// Disable auto-center spring and stop all effects
		SetAutoCenter(0);
		break;
	case 0x04:
		// Reset (e.g. FFB-application out of focus)
		ffbprintf("ffb devcntrl free\r\n");
		FreeAllEffects();
		break;
	case 0x05:
		// pause
		Bset(pidState.status, DEVICE_PAUSED);
		break;
	case 0x06:
		// continue
		Bclr(pidState.status, DEVICE_PAUSED);
		break;
	default:
	    return;
	}
}

void FfbEngine::FfbHandle_DeviceGain(USB_FFBReport_DeviceGain_Output_Data_t *data)
{
	joystick.gFFBDevice.ffbDevGain = data->gain;
	simucubelog.addEventParam(FFB_SetGain, data->gain, true);
}

void FfbEngine::FfbHandle_SetCustomForce(USB_FFBReport_SetCustomForce_Output_Data_t *data)
{
}


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
}

void FfbEngine::SetEnvelope (USB_FFBReport_SetEnvelope_Output_Data_t* data, int effectId)
{
	volatile cEffectState* effect = &gEffectStates[effectId];
	effect->attackLevel = data->attackLevel;
	effect->fadeLevel = data->fadeLevel;
	effect->attackTime = data->attackTime;
	effect->fadeTime = data->fadeTime;
	ffbprintf("env %d: %d,%d,%d,%d\r\n", effectId, data->attackLevel, data->fadeLevel, data->attackTime, data->fadeTime);
}

void FfbEngine::SetConditional (USB_FFBReport_SetCondition_Output_Data_t* data, int effectId)
{
	volatile cEffectState* effect = &gEffectStates[effectId];

	effect->magnitude = (s16)data->positiveCoefficient;
	effect->offset = data->cpOffset;
	effect->positiveSaturation = data->positiveSaturation;
	effect->negativeSaturation = data->negativeSaturation;
}

void FfbEngine::SetPeriodic (USB_FFBReport_SetPeriodic_Output_Data_t* data, int effectId)
{
	volatile cEffectState* effect = &gEffectStates[effectId];

	effect->magnitude = (s16)data->magnitude;
	effect->offset = data->offset;
	effect->phase = (s16)data->phase;
	effect->period = (u16)data->period;
	effect->freq = 0.0f;
}

void FfbEngine::SetConstantForce (USB_FFBReport_SetConstantForce_Output_Data_t* data, int effectId)
{
	volatile cEffectState* effect = &gEffectStates[effectId];

	effect->magnitude = data->magnitude;
}

void FfbEngine::SetRampForce (USB_FFBReport_SetRampForce_Output_Data_t* data, int effectId )
{
	volatile cEffectState* effect = &gEffectStates[effectId];

	effect->start_mag = data->start;
	effect->end_mag = data->end;
}

void FfbEngine::SetEffect (USB_FFBReport_SetEffect_Output_Data_t *data, int effectId )
{
	volatile cEffectState* effect = &gEffectStates[effectId];

	uint8_t eid = data->effectBlockIndex;
	effect->type = data->effectType;
	effect->gain = data->gain;
	ModifyDuration(eid, data->duration); // 0..32767 ms
}

void FfbEngine::CreateNewEffect(USB_FFBReport_CreateNewEffect_Feature_Data_t* inData, int effectId )
{
	volatile cEffectState* effect = &gEffectStates[effectId];

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
}

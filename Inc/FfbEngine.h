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
 * FfbEngine.h
 *
 *  Created on: Feb 21, 2017
 *      Author: Mika
 */

#ifndef FFBENGINE_H_
#define FFBENGINE_H_

#include "stdint.h"
#include "types.h"
#include "answer_filetypes.h"
#include "USBHID_Types.h"

typedef struct
{
	volatile uint8_t state;	// see constants MEffectState_*
	uint8_t type;	// see constants USB_EFFECT_*
	uint8_t gain;
	int8_t pad0;
	uint8_t attackLevel,fadeLevel;
	int8_t start_mag,end_mag;
	uint16_t positiveSaturation;	// -128..127
	uint16_t negativeSaturation;	// -128..127
	int16_t magnitude;
	int16_t phase;
	int16_t offset;
	uint16_t counter;						// ms
	uint16_t period;							// ms
	float freq;
	float fcounter;
	uint16_t duration,fadeTime,attackTime;	// ms
	uint16_t fade_start_time;
	uint64_t starttime;
} cEffectState;





/* Type Defines: */
/** Type define for the joystick HID report structure, for creating and sending HID reports to the host PC.
 *  This mirrors the layout described to the host in the HID report descriptor, in Descriptors.c.
 */

// Maximum number of parallel effects in memory
#define MAX_EFFECTS 13
#define FIRST_EID	1

#define CONTROL_PERIOD_MS	1

#define USB_DURATION_INFINITE	0//0x7FFF
#define USB_DURATION_INFINITE2	0xFFFF

#define USB_EFFECT_CONSTANT		0x01
#define USB_EFFECT_RAMP			0x02
#define USB_EFFECT_SQUARE 		0x03
#define USB_EFFECT_SINE 		0x04
#define USB_EFFECT_TRIANGLE		0x05
#define USB_EFFECT_SAWTOOTHUP	0x06
#define USB_EFFECT_SAWTOOTHDOWN	0x07
#define USB_EFFECT_SPRING		0x08
#define USB_EFFECT_DAMPER		0x09
#define USB_EFFECT_INERTIA		0x0A
#define USB_EFFECT_FRICTION		0x0B
#define USB_EFFECT_CUSTOM		0x0C

// ---- Input

typedef struct
	{
	uint8_t	reportId;	// =2
	uint8_t	status;	// Bits: 0=Device Paused,1=Actuators Enabled,2=Safety Switch,3=Actuator Override Switch,4=Actuator Power
	uint8_t	effectBlockIndex;	// Bit7=Effect Playing, Bit0..7=EffectId (1..40)
	} __attribute__((packed)) USB_FFBReport_PIDStatus_Input_Data_t;

// ---- Output

typedef struct
	{ // FFB: Set Effect Output Report
	uint8_t	reportId;	// =1
	uint8_t	effectBlockIndex;	// 1..40
	uint8_t	effectType;	// 1..12 (effect usages: 26,27,30,31,32,33,34,40,41,42,43,28)
	uint16_t	duration; // 0..32767 ms
	uint16_t	triggerRepeatInterval; // 0..32767 ms
	uint16_t	samplePeriod;	// 0..32767 ms
	uint8_t	gain;	// 0..255	 (physical 0..10000)
	uint8_t	triggerButton;	// button ID (0..8)
	uint8_t	enableAxis; // bits: 0=X, 1=Y, 2=DirectionEnable
	uint8_t	directionX;	// angle (0=0 .. 255=360deg)
	uint8_t	directionY;	// angle (0=0 .. 255=360deg)
//	uint16_t	startDelay;	// 0..32767 ms
	} __attribute__((packed)) USB_FFBReport_SetEffect_Output_Data_t;

typedef struct
	{ // FFB: Set Envelope Output Report
	uint8_t	reportId;	// =2
	uint8_t	effectBlockIndex;	// 1..40
	uint8_t attackLevel;
	uint8_t	fadeLevel;
	uint16_t	attackTime;	// ms
	uint16_t	fadeTime;	// ms
	} __attribute__((packed)) USB_FFBReport_SetEnvelope_Output_Data_t;

typedef struct
	{ // FFB: Set Condition Output Report
	uint8_t	reportId;	// =3
	uint8_t	effectBlockIndex;	// 1..40
	uint8_t	parameterBlockOffset;	// bits: 0..3=parameterBlockOffset, 4..5=instance1, 6..7=instance2
	int8_t  cpOffset;	// -128..127
	int8_t	positiveCoefficient;	// -128..127
//	int8_t	negativeCoefficient;	// -128..127
	uint8_t	positiveSaturation;	// -128..127
	uint8_t	negativeSaturation;	// -128..127
//	uint8_t	deadBand;	// 0..255
	} __attribute__((packed)) USB_FFBReport_SetCondition_Output_Data_t;

#if 0
typedef struct
	{ // FFB: Set Periodic Output Report
	uint8_t	reportId;	// =4
	uint8_t	effectBlockIndex;	// 1..40
	uint8_t magnitude;
	int16_t	offset;
	uint8_t	phase;	// 0..255 (=0..359, exp-2)
	uint16_t	period;	// 0..32767 ms
	} __attribute__((packed)) USB_FFBReport_SetPeriodic_Output_Data_t;
#endif
typedef struct
	{ // FFB: Set Periodic Output Report
	uint8_t	reportId;	// =4
	uint8_t	effectBlockIndex;	// 1..40
	uint16_t magnitude;
	int16_t	offset;
	uint16_t	phase;	// 0..255 (=0..359, exp-2)
	uint16_t	period;	// 0..32767 ms
	} __attribute__((packed)) USB_FFBReport_SetPeriodic_Output_Data_t;
typedef struct
	{ // FFB: Set ConstantForce Output Report
	uint8_t	reportId;	// =5
	uint8_t	effectBlockIndex;	// 1..40
	int16_t magnitude;	// -10000..10000 (set by hid descriptor report)
	} __attribute__((packed)) USB_FFBReport_SetConstantForce_Output_Data_t;

typedef struct
	{ // FFB: Set RampForce Output Report
	uint8_t	reportId;	// =6
	uint8_t	effectBlockIndex;	// 1..40
	int8_t start;
	int8_t	end;
	} __attribute__((packed)) USB_FFBReport_SetRampForce_Output_Data_t;

typedef struct
	{ // FFB: Set CustomForceData Output Report
	uint8_t	reportId;	// =7
	uint8_t	effectBlockIndex;	// 1..40
	uint8_t dataOffset;
	int8_t	data[12];
	} __attribute__((packed)) USB_FFBReport_SetCustomForceData_Output_Data_t;

typedef struct
	{ // FFB: Set DownloadForceSample Output Report
	uint8_t	reportId;	// =8
	int8_t	x;
	int8_t	y;
	} __attribute__((packed)) USB_FFBReport_SetDownloadForceSample_Output_Data_t;

typedef struct
	{ // FFB: Set EffectOperation Output Report
	uint8_t	reportId;	// =10
	uint8_t effectBlockIndex;	// 1..40
	uint8_t operation; // 1=Start, 2=StartSolo, 3=Stop
	uint8_t	loopCount;
	} USB_FFBReport_EffectOperation_Output_Data_t;

typedef struct
	{ // FFB: Block Free Output Report
	uint8_t	reportId;	// =11
	uint8_t effectBlockIndex;	// 1..40
	} __attribute__((packed)) USB_FFBReport_BlockFree_Output_Data_t;

typedef struct
	{ // FFB: Device Control Output Report
	uint8_t	reportId;	// =12
	uint8_t control;	// 1=Enable Actuators, 2=Disable Actuators, 4=Stop All Effects, 8=Reset, 16=Pause, 32=Continue
	} __attribute__((packed)) USB_FFBReport_DeviceControl_Output_Data_t;

typedef struct
	{ // FFB: DeviceGain Output Report
	uint8_t	reportId;	// =13
	uint8_t gain;
	} __attribute__((packed)) USB_FFBReport_DeviceGain_Output_Data_t;

typedef struct
	{ // FFB: Set Custom Force Output Report
	uint8_t		reportId;	// =14
	uint8_t effectBlockIndex;	// 1..40
	uint8_t	sampleCount;
	uint16_t	samplePeriod;	// 0..32767 ms
	} __attribute__((packed)) USB_FFBReport_SetCustomForce_Output_Data_t;

// ---- Features

typedef struct
	{ // FFB: Create New Effect Feature Report
	uint8_t		reportId;	// =1
	uint8_t	effectType;	// Enum (1..12): ET 26,27,30,31,32,33,34,40,41,42,43,28
	uint16_t	byteCount;	// 0..511
	} __attribute__((packed)) USB_FFBReport_CreateNewEffect_Feature_Data_t;
#if 0
typedef struct
	{ // FFB: PID Block Load Feature Report
	uint8_t	reportId;	// =2
	uint8_t effectBlockIndex;	// 1..40
	uint8_t	loadStatus;	// 1=Success,2=Full,3=Error
	uint16_t	ramPoolAvailable;	// =0 or 0xFFFF?
	} __attribute__((packed)) USB_FFBReport_PIDBlockLoad_Feature_Data_t;


typedef struct
	{ // FFB: PID Pool Feature Report
	uint8_t	reportId;	// =3
	uint16_t	ramPoolSize;	// ?
	uint8_t		maxSimultaneousEffects;	// ?? 40?
	uint8_t		memoryManagement;	// Bits: 0=DeviceManagedPool, 1=SharedParameterBlocks
	} __attribute__((packed)) USB_FFBReport_PIDPool_Feature_Data_t;
#endif
// Lengths of each report type
extern const uint16_t OutReportSize[];

// Handles Force Feeback data manipulation from USB reports to joystick's MIDI channel

void FfbSetDriver(uint8_t id);

// Handle incoming data from USB
void FfbOnUsbData(uint8_t *data, uint16_t len);

// Handle incoming feature requests
//void FfbOnCreateNewEffect(USB_FFBReport_CreateNewEffect_Feature_Data_t* inData, USB_FFBReport_PIDBlockLoad_Feature_Data_t *outData);
void FfbOnPIDPool(USB_FFBReport_PIDPool_Feature_Data_t *data);

// Utility to wait any amount of milliseconds.
// Resets watchdog for each 1ms wait.
void WaitMs(int ms);

// delay_us has max limits and the wait time must be known at compile time.
// function for making 10us delays that don't have be known at compile time.
// max delay 2560us.
void _delay_us10(uint8_t delay);

// Send raw data to the
void FfbSendData(const uint8_t *data, uint16_t len);
void FfbSendPackets(const uint8_t *data, uint16_t len);
void FfbPulseX1( void );

// Debugging
//	<index> should be pointer to an index variable whose value should be set to 0 to start iterating.
//	Returns 0 when no more effects
uint8_t FfbDebugListEffects(uint8_t *index);

// Effect manipulations

// Bit-masks for effect states
#define MEffectState_Free			107
#define MEffectState_Allocated		108
#define MEffectState_Playing		109


class FfbEngine {
public:
	FfbEngine();
	virtual ~FfbEngine();


	bool handleReceivedHIDReport(HID_REPORT report);
	void FfbOnCreateNewEffect (USB_FFBReport_CreateNewEffect_Feature_Data_t* inData, USB_FFBReport_PIDBlockLoad_Feature_Data_t *outData);
	volatile cEffectState* getEffectState(u8 id);

	void FreeAllEffects(void);

private:
	// Effect management
	volatile USB_FFBReport_PIDStatus_Input_Data_t pidState;	// For holding device status flags

	volatile cEffectState gEffectStates[MAX_EFFECTS+1];	// one for each effect (array index 0 is unused to simplify things)
	//cEffectState gEffectStates[MAX_EFFECTS+1];	// one for each effect (array index 0 is unused to simplify things)
	USB_FFBReport_PIDBlockLoad_Feature_Data_t gNewEffectBlockLoad;



	void SendPidStateForEffect(uint8_t eid, uint8_t effectState);
	uint8_t GetNextFreeEffect(void);
	void StartEffect(uint8_t id);
	void StopEffect(uint8_t id);
	void StopAllEffects(void);
	void FreeEffect(uint8_t id);

	void FfbHandle_EffectOperation(USB_FFBReport_EffectOperation_Output_Data_t *data);
	void FfbHandle_BlockFree(USB_FFBReport_BlockFree_Output_Data_t *data);
	void FfbHandle_DeviceControl(USB_FFBReport_DeviceControl_Output_Data_t *data);
	void FfbHandle_DeviceGain(USB_FFBReport_DeviceGain_Output_Data_t *data);
	void FfbHandle_SetCustomForceData(USB_FFBReport_SetCustomForceData_Output_Data_t* data);
	void FfbHandle_SetDownloadForceSample(USB_FFBReport_SetDownloadForceSample_Output_Data_t* data);
	void FfbHandle_SetCustomForce(USB_FFBReport_SetCustomForce_Output_Data_t* data);
	void FfbHandle_SetEffect(USB_FFBReport_SetEffect_Output_Data_t *data);
	void FfbOnPIDPool(USB_FFBReport_PIDPool_Feature_Data_t *data);

	// Lengths of each report type
	const uint16_t OutReportSize[14] =
	{
		sizeof(USB_FFBReport_SetEffect_Output_Data_t),		// 1
		sizeof(USB_FFBReport_SetEnvelope_Output_Data_t),	// 2
		sizeof(USB_FFBReport_SetCondition_Output_Data_t),	// 3
		sizeof(USB_FFBReport_SetPeriodic_Output_Data_t),	// 4
		sizeof(USB_FFBReport_SetConstantForce_Output_Data_t),	// 5
		sizeof(USB_FFBReport_SetRampForce_Output_Data_t),	// 6
		sizeof(USB_FFBReport_SetCustomForceData_Output_Data_t),	// 7
		sizeof(USB_FFBReport_SetDownloadForceSample_Output_Data_t),	// 8
		0,	// 9
		sizeof(USB_FFBReport_EffectOperation_Output_Data_t),	// 10
		sizeof(USB_FFBReport_BlockFree_Output_Data_t),	// 11
		sizeof(USB_FFBReport_DeviceControl_Output_Data_t),	// 12
		sizeof(USB_FFBReport_DeviceGain_Output_Data_t),	// 13
		sizeof(USB_FFBReport_SetCustomForce_Output_Data_t),	// 14
	};


	// FROM FFB_PRO
	void SetAutoCenter(uint8_t enable);
	void InitEffect(uint8_t id);
	void ModifyDuration(uint8_t effectId, uint16_t duration);
	void SetEnvelope(USB_FFBReport_SetEnvelope_Output_Data_t* data, int effectId );
	void SetConditional(USB_FFBReport_SetCondition_Output_Data_t* data, int effectId );
	void SetPeriodic(USB_FFBReport_SetPeriodic_Output_Data_t* data, int effectId );
	void SetConstantForce(USB_FFBReport_SetConstantForce_Output_Data_t* data, int effectId );
	void SetRampForce(USB_FFBReport_SetRampForce_Output_Data_t* data, int effectId );
	void SetEffect(USB_FFBReport_SetEffect_Output_Data_t *data, int effectId );
	void CreateNewEffect(USB_FFBReport_CreateNewEffect_Feature_Data_t* inData, int effectId );




};

#endif /* FFBENGINE_H_ */

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
 * cFFBDevice.h
 *
 *  Created on: Jan 30, 2017
 *      Author: Mika
 */

#ifndef CFFBDEVICE_H_
#define CFFBDEVICE_H_
//#include "stm32f4xx_hal_def.h"
#include "types.h"
#include "cDeviceConfig.h"
#include "../SimpleMotion/simplemotion.h"
#include "ffbengine.h"
//#include "stm32f407xx.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_gpio.h"
#include "Biquad1storder.h"

// maximum number of seconds for the a effect to stay visible to the configuration tool
#define ffbActiveLowpassTimeSeconds 30

// approximate ffb loop rate
#define ffbActiveLowpassTimeCycles ffbActiveLowpassTimeSeconds*2500

// maximum torque
#define MAX_NORM_TORQUE		0x10000

extern uint8_t IoniDrcFile[16384];

class cFFBDevice {
public:
	cFFBDevice();
	~cFFBDevice();

	// this sets default values to all class variables
	void SetDefault();

	// this sets the ffbengine pointer
	void SetFFB(FfbEngine* handle);

	// This is the main loop when SimuCUBE is operational. It parses FFB effects and
	// also writes it to drive.
	bool CalcTorqueCommand(s32 *readEncoderPos);

	// this initializes the class variables according to current profile.
	void initVariables();

	// this calculates the steering angle from the encoder position to
	// directinput unsigned 16bit. Uses floats internally.
	uint16_t calcSteeringAngle(s32 encoderCounter);

	// update button statuses
	void updateButtons();

	// flash load and save
	bool loadConfigsFromFlash();
	bool saveConfigsToFlash();

	// sets wheel center
	void setWheelCenter();

	//
	void writeChangedIoniParameters(bool forced);
	void readIoniParameters();
	bool readEncoder32bit();

	// limits effect to a range
	s32 ConstrainEffect(s32 val);

	// scales effect for simplemotion torque setpoint range
	s32 scaleEffectForSimplemotion (float inputval);


	// handles commands from configuration tool
	bool handleSimuCUBEAPIcommand(uint8_t* data);

	// handles custom IRFFB reports
	bool handleIRFFBcommand(uint8_t* data);


	// this contains all motor config and profiles
	cDeviceConfig mConfig;

	//smbus communication handle
	smbus mSMBusHandle;

	/* this variable controls whether simplemotion is free for new commmands */
	bool waitLastSimpleMotion;

	/* minimum and maximum steering angle in degrees. E.g, -900.0 - 900.0 */
	float minSteeringAngle;
	float maxSteeringAngle;

	/* these are used for endstop, as user can configure it to different positions */
	float minSteeringAngleForStop;
	float maxSteeringAngleForStop;
	float endstopdampergain;
	float degrees_from_endstop; // degrees from endstop on last loop
	bool endstopDirection;  // true -> turning towards the right endstop, false -> turningwards the left
	float endStopGain;
	float endstopMaxTorque;

	/* axis position and wheel speed variables */
	float wheelSpeed;
	s32 previousEncoderPos;
	volatile s32 axisPos;
	float axisPosFloat;
	float wheelSpeedMs;
	float lastAxisPosFloat;
	float axisSpeedPerMs;
	float diffMs;
	float steeringAngleUnlimited;
	uint16_t diffUs;
	uint16_t prevCalcTimeUs;


	/* temporary center point mode */
	bool temporaryCenterPoint;
	s32 mEncoderTemporaryOffset;

	/* temporary steering angle mode, set by game via our API */
	bool temporarySteeringAngleIsSet;
	uint16_t temporarySteeringLockToLock;

	/* motor control related variables */
	uint8_t indexpointfound;
	smint32 motorFaultRegister;
	bool forcesEnabled;

	/* analog axis mode */
	bool rawAnalogMode;

	/* variable to keep 2500 Hz sync */
	bool FFB2500HzWait;

	uint32_t debugvalue1;

	/* unsaved settings */
	uint8_t unsavedSettings;

	/* these are used to store received Ioni DRC Data and simucube hardware version ID */

	uint16_t IoniDrcDataReceivedBytes;
	uint8_t firmwareuploadingstatuspercentage;
	bool firmwareuploadinprogress;
	uint16_t IoniFWVersion;
	uint8_t scHWVersion;
	uint16_t DeviceTypeID;


	// put stuff needed for filtering here
	s32 movingAvgFilterHistory[50];
	u8 movingAvgFilterLatest;
	uint64_t prevEffectCalcTime;

	// special IRFFB automatic mode
	bool IRFFBModeEnabled;
	uint16_t latestIRFFBForce[6];



	/* buttons */
	uint64_t 		lastButtonUpdateMillis;
	GPIO_TypeDef* 	button_port[16];
	uint16_t 		button_pin[16];
	uint64_t 		button_lastUpdate[16];
	GPIO_PinState	button_currentState[16];
	GPIO_PinState	button_debouncedState[16];
	uint8_t debounce;

	/* profile management */
	uint16_t currentprofileindex;
	uint16_t numberofprofiles;
	uint16_t defaultprofileindex;
	bool rewriteDefault;


	/* Running FFB Device state variables */
	uint8_t ffbDevGain; // 0-255

	/* reset ffb state */
	void resetFFBDeviceState();

	/* flag to see if drive is initialized successfully */
	bool driveInitSuccess;

	/* encoder position at index point */
	s32 indexPointEncPos;

	/* when using the new fastUpdateCycle() mode, wheel phasing does not end at 0 encoder position. It shows this position. */
	s32 phasingEndOffset;

	/* status flag to see if simplemotion watchdog is enabled or not */
	bool watchdogenabled;

	// debug value for endstop debugging
	int16_t overendstop;




	/* Drive status handling and other drive related status variables */


	/* Drive status bits */
	u32 driveStatusBits;

	/* Automatic commutation status flags */
	AutoCommutationMode autoCommutationStatus;

	/* This variable holds the type of the encoder */
	int32_t encoderType;

	/* this is used in initing the drive, to force steering angle between -180 and +180 degrees
	 * at startup when using absolute encoder
	 */
	int32_t absoluteFullRotOffset;

	/* Last Motor Fault Location ID */
	int32_t faultLocationID;




	/* Biquad filters used by the firmware */
	Biquad1StOrder inertiaLPF;
	Biquad1StOrder dampingLPF;
	Biquad1StOrder dampingForceLPF;
	Biquad1StOrder frictionForceLPF;
	Biquad1StOrder axisSpeedMsLPF;
	Biquad1StOrder desktopDampingLPF;
	Biquad1StOrder endStopDampingLPF;


private:
	FfbEngine* ffbhandle;
	uint32_t settingsStartAddress = 0x08060000;//0x8040000;
	static uint32_t GetSector(uint32_t Address);


	// limit variable to certain limitss
	float limit(float input, float min, float max);

	// this holds up information about which effects are in use, to sent to PC in status reports.
	uint32_t ffbEffectUsage;
	// this holds up information about which effects have gotten changes.
	uint32_t ffbEffectActive;

	// these act as a "lowpass" filter for effect usage detection.
	uint32_t ConstantEffectCounters[32];
	//these hold last known value set by game for each effect
	int32_t LastEffectValue[32];

	void updateEffectCounters(s32 newdata, uint32_t bit);
	void updateEffectRegister();



	uint8_t counter;





	bool clippingLedOn;
};

#endif /* CFFBDEVICE_H_ */

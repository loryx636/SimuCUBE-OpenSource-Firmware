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

#include "driveparametertracker.h"
#include "simplemotion.h"
#include "simplemotioncomms.h"
#include "USBGameController.h"

extern USBGameController joystick;
extern bool debugMode;


DriveParameterTracker::DriveParameterTracker() {
	reset();
}

DriveParameterTracker::~DriveParameterTracker() {

}

void DriveParameterTracker::reset() {
	for(int i = 0; i<MAXPARAMETERS; i++) {
		lastParameters[i]=0;
		parameterAddrs[i]=0;
	}
	numberOfParameters=0;
	setOnce=false;
}

void DriveParameterTracker::addTrackedParameter(uint32_t SMAddr, uint32_t profileParameterAddr) {
	if (SMAddr==0) return; // illegal
	if (profileParameterAddr==0) return; //illegal
	if (numberOfParameters>=MAXPARAMETERS) return; //index larger than table
	if (profileParameterAddr>=numberOfProfAddrs) return; // larger than largest address
	parameterAddrs[numberOfParameters] = SMAddr;
	profileAddrs[numberOfParameters] = profileParameterAddr;
	numberOfParameters++;
}

#define PARAMDEBUG 0
#if PARAMDEBUG
bool paramdebug = true;
#else
bool paramdebug = false;
#endif
// The main point about this function is to remove unneeded filter changes on drive.
// If the parameters are related to biquad filters, an unneeded resetting might cause
// momentary torque jump/clitch, which this attempts to reduce.
void DriveParameterTracker::updateParameters(bool forced) {
	int counter=0;
	int32_t fault = 0;
	SM_STATUS result = SM_OK;
	if(paramdebug) {
		smRead1Parameter(joystick.gFFBDevice.mSMBusHandle, 1, SMP_FAULT_LOCATION1, &fault);
		printf("faults before: %ld\r\n", fault);
	}
	for(int i=0;i<MAXPARAMETERS; i++) {
		if(parameterAddrs[i]==0) break; //illegal/not added
		int32_t parameter = joystick.gFFBDevice.mConfig.profileConfigs[joystick.gFFBDevice.currentprofileindex].settings[profileAddrs[i]];
		if(!setOnce || forced || (lastParameters[i] != parameter)) {
			printf("dpt: setting parameter %ld value %d\r\n", parameterAddrs[i], parameter);
			if(paramdebug) {
				result = smRead1Parameter(joystick.gFFBDevice.mSMBusHandle, 1, SMP_FAULT_LOCATION1, &fault);
				printf("faults before: %ld\r\n", fault);
			}
			result = smSetParameter(joystick.gFFBDevice.mSMBusHandle, 1, parameterAddrs[i], (uint32_t)parameter);
			if(result != SM_OK) {
				printf("fault! ");
				result = smRead1Parameter(joystick.gFFBDevice.mSMBusHandle, 1, SMP_FAULT_LOCATION1, &fault);
				printf("%ld\r\n", fault);
			}
			lastParameters[i]=parameter;
			counter++;
			if(paramdebug) {
				result = smRead1Parameter(joystick.gFFBDevice.mSMBusHandle, 1, SMP_FAULT_LOCATION1, &fault);
				printf("faults after: %ld\r\n", fault);
			}
		}
	}
	if(debugMode) printf("dpt: changed %d filter parameters on drive. Forced: %d SetOnce: %d\r\n", counter, forced, setOnce);


	setOnce=true;
	setCurrentProfileMMC();
}



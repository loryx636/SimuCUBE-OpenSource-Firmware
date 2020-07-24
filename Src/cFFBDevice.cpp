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

#include <eventLog.h>
#include <FfbEngine.h>
#include <FfbEffects.h>
#include "cFFBDevice.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"
#include "config_comm_defines.h"

#include "usbd_customhid.h"
#include "USBGameController.h"
#include "types.h"
#include "FfbEffects.h"

#define ARM_MATH_CM4
#include <math.h>
#include "arm_math.h"

#include "Biquad1storder.h"
#include "simplemotioncomms.h"
#include "simplemotion_private.h"

#include "hw_functions.h"

#include "stm32f407xx.h"

extern volatile SystemStatus currentSystemStatus;
extern volatile SystemStatus nextSystemStatus;
extern volatile SystemStatus previousSystemStatus;
extern volatile SystemStatus previousSystemStatus2;
extern USBD_HandleTypeDef hUsbDeviceFS;

extern uint8_t firmwareVersion[3];
extern uint8_t lowestCompatibleFlashSettingsMajorVersion;
extern uint8_t lowestCompatibleFlashSettingsMinorVersion;
extern uint32_t driveStatusBits;

extern bool debugMode;
extern bool debugMode2;
extern void start_timer2_15ms_delay();
extern USBGameController joystick;

extern uint32_t globaldebugvalue2;
extern eventLog simucubelog;

extern uint64_t millis;
extern TIM_HandleTypeDef htim6_1MhzEffectClock;
static FLASH_EraseInitTypeDef EraseInitStruct;

cFFBDevice::cFFBDevice() : latestIRFFBForce{0} {
	// Auto-generated constructor stub
	SetDefault();
	_smReleased = false;
	rawAnalogMode = false;
	forcesEnabled = 1; // only disabled for motorconfigwizard or if returning from IONI config with instant torque
	forcesDisabledForSafety = 0; // only 1 if over/too near endstop when coming back from IONI config mode

	temporaryCenterPoint = false;

	temporarySteeringAngleIsSet = false;
	temporarySteeringLockToLock = 0;

	IoniDrcDataReceivedBytes=0;
	firmwareuploadingstatuspercentage = 0;
	firmwareuploadinprogress = false;

	unsavedSettings = 0;
	_indexpointFound = 0;
	motorFaultRegister = 0;

	IRFFBModeEnabled = false;
	IoniFWVersion = 0;
	scHWVersion = hwunknown;

	FFBLoopWait = false;
	debugvalue1_ = 0;
	currentprofileindex = 0;
	currentanalogconfigindex = 0;
	defaultprofileindex = 0;
	numberofprofiles = 1;
	rewriteDefault = false;

	ffbDevGain = 255;
	ffbEffectUsage = 0;
	driveInitSuccessFlag=0;
	clippingLedOn=false;

	indexPointEncPos=0;
	counter=0;
	faultLocationID=0;
	testModeActive=0;
	driveStatusBits=0;
	pin7shift=0;

	bledetected=0;

	ServoDriveDisabled=false;
	servoStandby = 0;
	_swwIdleDisonnect = 0;
	torqueFailCount = 0;
	factory_encoder_offset = 0;
	ledCyclePointer = 0;
	STOStatus=0; // STO disabled
	wirelessTorqueOffEvent = 0;
	wirelessTorqueOffButton = 0;
	torqueOffSavingDelay = 0;
	torqueOffSavingDelayStart = 0;
	sendDisableHighTorque = 0;

	lastWirelessConnectionState = false;
	lastWirelessVoltage = 0;

}

cFFBDevice::~cFFBDevice() {
	// Auto-generated destructor stub
}

void cFFBDevice::SetFFB(FfbEngine* handle) {
	ffbhandle = handle;
}

void cFFBDevice::setSmReleased(bool newstate) {
	_smReleased = newstate;
}

void cFFBDevice::SetDefault()	{
	mConfig.SetDefault();
	lastTelemetryReceived=millis&0xFFFF;

	currentprofileindex = 0;
	defaultprofileindex = 0;
	numberofprofiles = 1;
	rewriteDefault = false;
	waitLastSimpleMotion = false;

	lastButtonUpdateMillis = 0;
	forceFirsttimeDriveInit=0;

	initButtonInputs();

	BLEConn.setUart(&huart1);
	angle.setDevice(this);
	_swwautodisconnecthandler.init(&angle, &BLEConn, this);
	/* ffb device state variables */
	resetFFBDeviceState();
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

void cFFBDevice::setDebugValue1(uint32_t value) {
	if(debugvalue1_ == 0) {
		debugvalue1_ = value;
	}
}

uint32_t cFFBDevice::getDebugValue1() {
	return debugvalue1_;
}

void cFFBDevice::updateEffectCounters(int32_t newdata, uint32_t bit) {
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

uint32_t torque_last_sent = 0;
uint32_t between_torque_sends = 0;

bool wait(volatile bool &until) {
	uint32_t start = millis;
 	while (until) {
		if(millis - start > 3) {
			return false; // timer stuck?
		}
	}

	return true;
}

bool cFFBDevice::CalcTorqueCommand() {

	bool restore_pc_sampling = (DWT->CTRL & DWT_CTRL_PCSAMPLENA_Msk) != 0;

	if (restore_pc_sampling) {
		// turn off program counter sampling to remove the biggest offender
		// (this and/or the wait method) from the results.
		// did not appear to give better resolution for the sampling in general.
		DWT->CTRL &= !DWT_CTRL_PCSAMPLENA_Msk;
	}

	// await until the timer interrupt sets FFBLoopWait to false
	bool waited_ok = wait(FFBLoopWait);

	if (restore_pc_sampling) {
		DWT->CTRL |= DWT_CTRL_PCSAMPLENA_Msk;
	}

	if (!waited_ok) {
		// we must have missed the interrupt or something has been misconfigured
		return false;
	}

	FFBLoopWait = true;

	float effectTorque = 0.0;
	float effectTorque_for_filtering = 0.0;//torque that will go through reconstruction filter as well

	if(!IRFFBModeEnabled) {
		calculate_effects_torque(effectTorque, effectTorque_for_filtering);
	} else {
		//IRFFB MODE (not implemented at all)
        effectTorque += irFFBEffects();
	}

	endstop_effect.totalOut(this->axisSpeedPerMs, effectTorque_for_filtering, effectTorque);

	/* EFFECT SCALING FOR SIMPLEMOTION */
	// this command needs to be scaled to suitable range for Simplemotion:

	const float deviceGain = this->ffbDevGain / 255.0f;

    effectTorque *= deviceGain;
    effectTorque_for_filtering *= deviceGain;

	const int32_t smcommand = scaleEffectForSimplemotion(effectTorque);
	const int32_t smcommand_for_filtering = scaleEffectForSimplemotion(effectTorque_for_filtering);

	// toggle clipping led/led color
	setClippingLed(smcommand_for_filtering + smcommand);

	torqueCommand(smcommand_for_filtering, smcommand);

	return true;
}

bool cFFBDevice::zeroTorqueCommand() {
	if (!FFBLoopWait) {
		FFBLoopWait = true;
		torqueCommand(0, 0);

		torqueOffSavingDelay = 1;
		torqueOffSavingDelayStart = millis;
		return true;
	}

	return false;
}

void cFFBDevice::torqueCommand(int32_t filtered, int32_t direct) {
	int8_t wirelessTorqueStatus = inputDeviceList.getTorqueButtonState();
	switch(wirelessTorqueStatus) {
	case -1: // Torque button not pressed, torque off
		wirelessTorqueOffButton=1;
		break;
	case 0: // No torque button
	case 1: // Torque button pressed, torque on
		wirelessTorqueOffButton=0;
		break;
	default:
		break;
	}

	if(torqueOffSavingDelay==1) {

		const uint64_t since = millis - torqueOffSavingDelayStart;
		const uint64_t limit = 5000;

		if (since > limit) {
			torqueOffSavingDelay=0;
		} else {
			float c = (float)since / (float)limit;
			filtered = (int32_t)((float)filtered * c);
			direct = (int32_t)((float)direct * c);
		}
	}

	TorqueResponse resp;

	if(forcesEnabled==0 || wirelessTorqueOffButton==1 || wirelessTorqueOffEvent==1) {
		resp = SetTorque(0, 0);
		lastTorqueSummed = 0;
	} else {
		resp = SetTorque(filtered, direct);
		lastTorqueSummed = filtered + direct;
	}

	{
		auto now = DWT->CYCCNT;

		if (torque_last_sent != 0) {
			// if you are interested in jitter, use SWO and DWT to track writes to &between_torque_sends.
			// with Atollic this means opening the SWV data trace view and configuring:
			//
			//  - disabling everything
			//  - enable timestamps
			//  - enable comparator one, track writes to a word (uint32_t) at &between_torque_sends
			//
			// after configuring, press the red recording button and enjoy the Atollic graphical experience.
			// the result here is number of cycles or 1/144e6 seconds
			between_torque_sends = now - torque_last_sent;
		}

		torque_last_sent = now;
	}

	angle.calcSteeringAngle(resp);
}

void cFFBDevice::setSWWIdleDisconnectStatus(uint8_t status) {
	_swwIdleDisonnect = status;
}

void cFFBDevice::calculate_effects_torque(float& torquecommand, float& torquecommand_for_filtering) {
	const uint32_t cpr = angle.getCPR();
	uint8_t numEffects = 0;
	float frictionTorq = 0.0;

	ffbEffectUsage = 0;

	for (uint8_t id = FIRST_EID; id <= MAX_EFFECTS; id++) {
		volatile cEffectState* ef = ffbhandle->getEffectState(id);
		if(ef->state != MEffectState_Playing && ef->state != MEffectState_Allocated)
		{
			continue;
		}

		numEffects++;
		uint8_t effectType = 32; // invalid shift for uint32_t
		int16_t magnitude = ef->magnitude;

		switch (ef->type)
		{
		case USB_EFFECT_CONSTANT:
			effectType = ConstantEffectBit;
			torquecommand_for_filtering += constantForceEffect(ef);
			break;
		case USB_EFFECT_RAMP:
			// not implemented yet
			effectType = RampEffectBit;
			break;
		case USB_EFFECT_SQUARE:
			effectType = SquareEffectBit;
			torquecommand_for_filtering += squareEffect(ef, mConfig.profileConfigs[currentprofileindex].settings[addrSquareGain]);
			break;
		case USB_EFFECT_SINE:
			if(ef->magnitude==0) {
				// implement special case for rFactor2/RaceRoom/other ISI engine games, that do not use USB_EFFECT_CONSTANT
				// but use USB_EFFECT_SINE with magnitude = 0 and offset = force level.
				effectType = PeriodSineConstantEffectBit;
				magnitude = ef->offset;
				torquecommand_for_filtering+=sineEffect(ef, mConfig.profileConfigs[currentprofileindex].settings[addrSineGain]);
			} else {
				effectType = PeriodSineChangingEffectBit;
				magnitude = ef->magnitude;
				torquecommand_for_filtering+=sineEffect(ef, mConfig.profileConfigs[currentprofileindex].settings[addrSineGain]);
			}
			break;
		case USB_EFFECT_TRIANGLE:
			effectType = TriangleEffectBit;
			torquecommand_for_filtering+=triangleEffect(ef, mConfig.profileConfigs[currentprofileindex].settings[addrTriangleGain]);
			break;
		case USB_EFFECT_SAWTOOTHDOWN:
			effectType = SawtoothDownEffectBit;
			torquecommand_for_filtering += sawtoothDownEffect(ef, mConfig.profileConfigs[currentprofileindex].settings[addrSawtoothGain]);
			break;
		case USB_EFFECT_SAWTOOTHUP:
			effectType = SawtoothUpEffectBit;
			torquecommand_for_filtering += sawtoothUpEffect(ef, mConfig.profileConfigs[currentprofileindex].settings[addrSawtoothGain]);
			break;
		case USB_EFFECT_SPRING:
			effectType = SpringEffectBit;
			torquecommand_for_filtering += springEffect(ef, mConfig.profileConfigs[currentprofileindex].settings[addrSpringGain]);
			break;
		case USB_EFFECT_FRICTION:
			effectType = FrictionEffectBit;
			frictionTorq = frictionEffect(angle.getCurrentEncoderValue(), cpr, ef, mConfig.profileConfigs[currentprofileindex].settings[addrFrictionGain]);
			break;
		case USB_EFFECT_DAMPER:
			effectType = DampingEffectBit;
			torquecommand += calcDampingEffect(mConfig.profileConfigs[currentprofileindex].settings[addrDamperGain], axisSpeedPerMs, ef, dampingForceLPF);
			break;
		case USB_EFFECT_INERTIA:
			// not implemented yet
			effectType = InertiaEffectBit;
			break;
		case USB_EFFECT_CUSTOM:
			// not implemented yet.
			effectType = CustomEffectBit;
			break;
		default:
			break;
		}

		if (effectType < 32) {
			ffbEffectUsage |= (1 << effectType);
			updateEffectCounters(magnitude, effectType);
		}
	}

	/* FFB FILTERING STUFF */

	if (mConfig.profileConfigs[currentprofileindex].settings[addrFilteringModes]&(1<<bitTestFilter)) {
		// other filter development can be done here, for example. Define bits like testFilter, and assign new variables in profiles settings if required.
	}

	/* DESKTOP SPRING EFFECT */
	if(numEffects==0) {
		torquecommand_for_filtering += desktopSpringEffect(mConfig.hardwareConfig.hwSettings[addrDesktopSpringGain]);
	}

	/* PROCESS INERTIA, FRICTION AND DAMPING EFFECTS */

	// as friction is unimplmemented, these do nothing
	const float some_coefficient = 256.0f * 256.0f; // experimented to be in correct scale
	const float really_no_idea_what_this_is_and_why = -frictionTorq * some_coefficient;

	torquecommand -= really_no_idea_what_this_is_and_why;
}

int32_t cFFBDevice::scaleEffectForSimplemotion(float inputval) {

	//max possible value for val is +/-
	//10000*255 and it needs to scaled to +/- 16384
	// -> output = inputval/maxvalue*maxoutput
	//           = inputval*maxoutput/maxvalue
	const float maxoutvalue = 16384.0;
	const float maxinputvalue = 255 * 10000.0;

	const float output = inputval * (maxoutvalue / maxinputvalue);

	if (isnan(output)) {
		// fmax and fmin will select the non-NaN component but we do not want to have -maxoutvalue
		// if some calculation failed
		return 0;
	}

	// clamp output between -max and max before casting it to int32_t in order
	// to avoid UB when output does not fit into int32_t or is nan or inf
	return (int32_t)fmin(maxoutvalue, fmax(-maxoutvalue, output));
}

// this function should be called when any hardware or profile settings has been being changed.
void cFFBDevice::initVariables() {

	int32_t cpr = angle.getCPR();
	float LPFfreq = fmax(4.0, fmin(30.0, 4*cpr/1000.0)); //4 is good value for even 1024ppr

	inertiaLPF.setBiquad(Biquad1StOrder::bq_type_lowpass_1st_order, LPFfreq/DRIVEUPDATERATE);
	dampingLPF.setBiquad(Biquad1StOrder::bq_type_lowpass_1st_order, 1250.0/DRIVEUPDATERATE);
	dampingForceLPF.setBiquad(Biquad1StOrder::bq_type_lowpass_1st_order, 200.0/DRIVEUPDATERATE);
	frictionForceLPF.setBiquad(Biquad1StOrder::bq_type_lowpass_1st_order, 200.0/DRIVEUPDATERATE);

	desktopDampingLPF.setBiquad(Biquad1StOrder::bq_type_lowpass_1st_order, 200.0/DRIVEUPDATERATE);

	// FIXME: unknown why this has different fc than the most
	axisSpeedMsLPF.setBiquad(Biquad1StOrder::bq_type_lowpass_1st_order, 100.0/DRIVEUPDATERATE);

	float max_angle = 0.0f;
	if(!temporarySteeringAngleIsSet) {
		max_angle = (float)(mConfig.profileConfigs[currentprofileindex].settings[addrMaxAngle]);
	} else {
		max_angle = (float)(temporarySteeringLockToLock);
	}

	angle.setCenteringMode(mConfig.hardwareConfig.hwSettings[addrIndexingMode]);
	angle.setMinMaxSteeringAngles(max_angle);

	endstop_effect.refresh(true,
	        mConfig.profileConfigs[currentprofileindex].settings[addrEndstopOffset],
	        mConfig.profileConfigs[currentprofileindex].settings[addrBumpstopSetting],
		max_angle,
		200.0f/DRIVEUPDATERATE,
		&angle);

	// middle position momentarily, proper value with possibly new
	// steering angle (lock-to-lock) setting will be calculated
	// in the next effect command.
	angle.setUSBAngle(32768);
	initHwEncoderButtons();
	setBLEAutoConnectMode();
}


void cFFBDevice::setWheelCenter() {
	int32_t offset = 0;
	TorqueResponse resp = SetTorque(0,0);

	SM_STATUS stats = SM_OK;
	int32_t indexingMode = angle.getCenteringMode();
	switch(indexingMode) {
	case noIndexing:
		// not possible/not set, do nothing
		break;
	case startAtCenterPhasedIndexing:
		stats = smSetParameter(mSMBusHandle, 1, SMP_SIMUCUBE_WHEEL_CENTER_OFFSET, -1);
		//readback
		stats = smRead1Parameter(mSMBusHandle, 1,SMP_SIMUCUBE_WHEEL_CENTER_OFFSET, &offset);
		if(debugMode) {
			printf("setWheelCenter: new offset = %ld\r\n", offset);
		}
		simucubelog.addEventParam(96, offset, true);

		if(!temporaryCenterPoint) {
			mConfig.hardwareConfig.hwSettings[addrEncoderOffset] = offset;
			unsavedSettings = 1;
		} else {
			mEncoderTemporaryOffset = offset;
		}
		angle.reset();
		break;
	case indexPointIndexing:
		// for index point indexing, need to get latest value from torqueresponse.
		offset = resp.position;
		if(!temporaryCenterPoint) {
			if(debugMode) {
				printf("indexpoint center, offset %ld\r\n", offset);
			}
			mConfig.hardwareConfig.hwSettings[addrEncoderOffset] = offset;
			angle.setIndexpointOffset(offset);
		} else {
			mEncoderTemporaryOffset = offset;
		}
		break;
	case autoCommutationIndexing:
		// this case matches simucube 2 exactly
		stats = smSetParameter(mSMBusHandle, 1, SMP_SIMUCUBE_WHEEL_CENTER_OFFSET, -1);
		//readback
		stats = smRead1Parameter(mSMBusHandle, 1,SMP_SIMUCUBE_WHEEL_CENTER_OFFSET, &offset);
		if(debugMode) {
			printf("setWheelCenter: new offset = %ld\r\n", offset);
		}
		simucubelog.addEventParam(96, offset, true);

		if(!temporaryCenterPoint) {
			mConfig.hardwareConfig.hwSettings[addrEncoderOffset] = offset;
			unsavedSettings=1;
		} else {
			mEncoderTemporaryOffset = offset;
		}
		angle.reset();
		break;
	}
}

void cFFBDevice::updateButtons() {
	if(lastButtonUpdateMillis == millis) {
		return;
	}
	lastButtonUpdateMillis = millis;
	uint8_t debounce = mConfig.hardwareConfig.hwSettings[addrButtonDebounceMs];
	int numButtons = getNumberOfButtons();
	for(int i=0; i<numButtons; i++) {
		button_currentState[i] = HAL_GPIO_ReadPin(button_port[i], button_pin[i]);
		if((button_currentState[i] != button_debouncedState[i]) && ((button_lastUpdate[i]+debounce) < millis)) {
			button_debouncedState[i] = button_currentState[i];
			button_lastUpdate[i]=lastButtonUpdateMillis;
		}
	}
}


void cFFBDevice::handleWirelessEvents() {

	bool disconnected = false;
	// handle auto idle disconnect
	if(!_swwautodisconnecthandler.run()) {
		if(debugMode) printf("wireless wheel inactivity disconnect");
		simucubelog.addEvent(BLEInactivityDisconnect);
	}

	if (inputDeviceList.getPaddlesPressedOver5s()) {
		if (debugMode) {
			printf("Disconnect after having paddles pressed over 5s\r\n");
		}
		BLEConn.disconnectFromDevice();
		BLEConn.lastButtonState = 0;
		inputDeviceList.process(0);
		disconnected = true;
	} else {
		bool previousConnectionState = lastWirelessConnectionState;
		lastWirelessConnectionState = BLEConn.getConnectionState();
		disconnected = previousConnectionState && !lastWirelessConnectionState;
	}

	if (disconnected) {
        if(inputDeviceList.getTorqueButtonState()){
            wirelessTorqueOffEvent = 1;
        }
		lastWirelessVoltage = 0;
		SMPlaySound(Disconnected);
		inputDeviceList.clearConfiguration();
		return;
	}

	if (!lastWirelessConnectionState) {
		return;
	}

	uint16_t currentVoltage = BLEConn.getBatteryVoltage();
	if(lastWirelessVoltage == 0 && currentVoltage != 0) {

        LowBatteryWarning warning = BLEConn.getLowBatteryWarning();
        switch(warning) {
            case LowBatteryWarning::NoWarning:
                lastWirelessVoltage = currentVoltage; // no further checks
                break;
            case LowBatteryWarning::Warning:
                lastWirelessVoltage = currentVoltage;
                SMPlaySound(LowBattery);
                break;
            default:
                break;
        }
		if(currentVoltage < 2700) {
			// TODO: change this limit for release ... has not been changed
			// play sound anyway if <2700 mV is seen
			SMPlaySound(LowBattery);
			lastWirelessVoltage = currentVoltage; // no further checks
		}
	}
}

// checks if loaded flash data was such that it should be re-initialized
bool cFFBDevice::checkIfPre012Data() {
	if(flashMajorVersion == 0) {
		if(flashMinorVersion < 12) {
			return true;
		}
	}
	return false;
}

bool cFFBDevice::checkIfPre10000Data() {
	if(flashMajorVersion<1) {
		return true;
	}
	return false;
}

// initializes those hardware setting and profile parameters to 0 that <0.12.x series configuration tools could have
// sent to the device as random (uninitialized) parameters
void cFFBDevice::initUninitializedProfileParametersPre012() {
	for(unsigned int i = 0; i < maxnumprofiles_v11; i++) {
		for(unsigned int j = 19; j < numberOfProfAddrs; j++) {
			mConfig.profileConfigs[i].settings[j] = 0;
		}
	}
}

void cFFBDevice::resetParametersPre10000() {
	for(unsigned int i = 0; i < maxnumprofiles_v11; i++) {
		// static force might be initialized as bogus value, set it to 0
		mConfig.profileConfigs[i].settings[addrStaticForceReduction] = 0;
	}
	mConfig.hardwareConfig.hwSettings[addrEncoderOffset] = 0;
	mConfig.hardwareConfig.hwSettings[addrDesktopSpringGain] = 0;
}

void cFFBDevice::adjustAnyWheelConnection() {
	if(flashMajorVersion == 1 && flashMinorVersion == 0 && flashBuildVersion<24) {
		int32_t oldsetting = mConfig.hardwareConfig.hwSettings[addrAutoConnectBLEDevice];
		// old no  		= 2, new = 1
		// old yes 		= 1, new = 0;
		// old yes_any  = 0, new = 0;
		oldsetting--;
		if(oldsetting < 0) {
			oldsetting = 0;
		}
		mConfig.hardwareConfig.hwSettings[addrAutoConnectBLEDevice] = oldsetting;
	}
}

bool cFFBDevice::fillFlashZeroes(uint32_t startAddress, uint32_t length) {
	// flash erase fills with 0xFFFF. Do not do anything here.
	return true;
}

bool cFFBDevice::loadConfigsFromFlash() {
	volatile uint32_t idstring = *(uint32_t*)SETTINGSSTARTADDRESS;
	uint8_t b = (idstring >> 24);
	uint8_t c = (idstring >> 16) & 0xFF;
	uint8_t m = (idstring >> 8)  & 0xFF;
	uint8_t s = (idstring) & 0xFF;
	if(s != 'S' || m != 'M' || c != 'C' || b != 'B') {
		if(debugMode) printf("no settings found in flash at all!\r\n");
		simucubelog.addEvent(flashEmpty);
		return false;
	}

	volatile uint32_t verstring = *(uint32_t*)(SETTINGSSTARTADDRESS+4);

	flashMajorVersion = (verstring) & 0xFF;
	flashMinorVersion = (verstring >> 8)  & 0xFF;
	flashBuildVersion = (verstring >> 16) & 0xFF;

	// too old checks
	if(flashMajorVersion < lowestCompatibleFlashSettingsMajorVersion) {
		if(debugMode) printf("reading flash status: too old major settings on flash!\r\n");
		simucubelog.addEvent(flashTooOldMajor);
		return false;
	}

	// too new checks
	if(flashMajorVersion > firmwareVersion[majorVersionIdx]) {
		if(debugMode) printf("reading flash failed: too new major settings on flash!\r\n");
		simucubelog.addEvent(flashTooNewMajor);
		return false;
	}

	bool sameMajorVersion = (flashMajorVersion == firmwareVersion[majorVersionIdx]);

	if(sameMajorVersion) {
		if((flashMinorVersion > firmwareVersion[minorVersionIdx])
				&& (flashMajorVersion == lowestCompatibleFlashSettingsMajorVersion)) {
			if(flashMajorVersion>firmwareVersion[majorVersionIdx]) {
				if(debugMode) printf("reading flash failed: too new settings on flash!\r\n");
				simucubelog.addEvent(flashTooNewMinor);
				return false;
			}
		}
	} // else majorversion rollover, do not do any checks

	// compatible major and minor versions found.

	if(flashBuildVersion<firmwareVersion[buildVersionIdx]
        ||  flashMinorVersion<firmwareVersion[minorVersionIdx]
		||  !sameMajorVersion) {
		// need to write read-only default profile again.
		rewriteDefault = true;
	}
	bool doCRC = false;
	if((flashMajorVersion>=0)) {
		if(flashMinorVersion>=11) {
			if((flashMinorVersion==11)) {
				if((flashBuildVersion< 7)) {
					doCRC=false;
				} else {
					doCRC=true;
				}
			} else {
				doCRC=true;
			}
		} else {
			doCRC=false;
		}
	}

	if(doCRC) {
		// flash crc is stored at 0.11.6 and up
		bool valid = checkFlashCRC();
		if(!valid) {
			if(debugMode) printf("flash crc32 invalid\r\n");
			simucubelog.addEventParam(flashDataInvalid, 1);
			rewriteDefault = true;
			return false;
		}
		if(debugMode) printf("flash crc32 valid\r\n");
	}

	uint32_t sourceaddr = HWSETTINGSADDRESS;

	// read hardware configs
	for(int i = 0; i< numberOfHwSettingsAddrs; i++) {
		mConfig.hardwareConfig.hwSettings[i] = *(int32_t*)sourceaddr;
		sourceaddr+=4;
	}
	//bytestore
	for(int i = 0; i<256; i++) {
		mConfig.hardwareConfig.bytestore[i] = *(uint8_t*)sourceaddr;
		sourceaddr++;
	}


	// analog axises
	sourceaddr=ANALOGSETTINGSADDRESS;
	for(int analogconf = 0; analogconf<maxnumanalogconfigs; analogconf++) {
		for(int analogaxis = 0; analogaxis<7; analogaxis++) {
			for(int analogsetting = 0; analogsetting<numberOfAnalogSettingsAddrs; analogsetting++) {
				mConfig.analogConfig[analogconf].axises[analogaxis].settings[analogsetting] = *(int32_t*)sourceaddr;
				sourceaddr+=4;
			}
		}
		for(int bytestoreint = 0; bytestoreint<32; bytestoreint++) {
			mConfig.analogConfig[analogconf].bytestore[bytestoreint] = *(uint8_t*)sourceaddr;
			sourceaddr++;
		}
	}

	// buttons
	sourceaddr=BUTTONSSETTINGSADDRESS;
	for(int i = 0; i< numberofbuttonsettings; i++) {
		mConfig.buttonConfig.settings[i] = *(int32_t*)sourceaddr;
		sourceaddr+=4;
	}

	sourceaddr=PROFILESETTINGSADDRESS;
	/* read number of profiles, low and high bytes */
	uint8_t bytelo = *(uint8_t*)sourceaddr; sourceaddr++;
	uint8_t bytehi = *(uint8_t*)sourceaddr; sourceaddr++;
	numberofprofiles = bytelo + (bytehi << 8);

	// read defaultprofile, low and high bytes */
	bytelo = *(uint8_t*)sourceaddr; sourceaddr++;
	bytehi = *(uint8_t*)sourceaddr; sourceaddr++;
	defaultprofileindex = bytelo + (bytehi << 8);
	currentprofileindex=defaultprofileindex;


	// profiles
	for(int profileidx = 0; profileidx<maxnumprofiles_v11; profileidx++) {
		//addrs
		for(int profileaddr=0; profileaddr<numberOfProfAddrs; profileaddr++) {
			mConfig.profileConfigs[profileidx].settings[profileaddr] = *(int32_t*)sourceaddr;
			sourceaddr+=4;
		}
		//names
		for(int namebyte=0; namebyte<32; namebyte++) {
			mConfig.profileConfigs[profileidx].profilename[namebyte] = *(uint8_t*)sourceaddr;
			sourceaddr++;
		}
		//bytestores
		for(int storebyte=0; storebyte<32; storebyte++) {
			mConfig.profileConfigs[profileidx].bytestore[storebyte] = *(uint8_t*) sourceaddr;
			sourceaddr++;
		}
		mConfig.validateProfile(profileidx);
	}

	if(flashMajorVersion<1 && flashMinorVersion<50) {
		mConfig.hardwareConfig.convertDesktopSpring();
		// set new parameters for 1.0.0 version:
	}
	return true;

}




// Reverses (reflects) bits in a 32-bit word.
uint32_t cFFBDevice::reverse(uint32_t x) {
   x = ((x & 0x55555555) <<  1) | ((x >>  1) & 0x55555555);
   x = ((x & 0x33333333) <<  2) | ((x >>  2) & 0x33333333);
   x = ((x & 0x0F0F0F0F) <<  4) | ((x >>  4) & 0x0F0F0F0F);
   x = (x << 24) | ((x & 0xFF00) << 8) |
       ((x >> 8) & 0xFF00) | (x >> 24);
   return x;
}

// ----------------------------- crc32a --------------------------------
uint32_t cFFBDevice::crc32a(const uint8_t *message, uint32_t datalength) {
	uint32_t i, j;
	uint32_t byte, crc;

   i = 0;
   crc = 0xFFFFFFFF;
   while (i < datalength ) {
      byte = message[i];            // Get next byte.
      byte = reverse(byte);         // 32-bit reversal.
      for (j = 0; j <= 7; j++) {    // Do eight times.
         if ((int)(crc ^ byte) < 0)
              crc = (crc << 1) ^ 0x04C11DB7;
         else crc = crc << 1;
         byte = byte << 1;          // Ready next msg bit.
      }
      i = i + 1;
   }
   return reverse(~crc);
}


bool cFFBDevice::checkFlashCRC() {
	uint32_t totalCRC = crc32a((uint8_t*)SETTINGSSTARTADDRESS, SETTINGSENDADDRESS - 4 - SETTINGSSTARTADDRESS);
	if(debugMode) printf("got %lu\r\n", totalCRC);
	uint32_t savedCRCaddr = SETTINGSENDADDRESS -4;
	uint32_t savedCRC = *(uint32_t*)savedCRCaddr;
	if(debugMode) printf("saved was %lu\r\n", savedCRC);
	if(savedCRC != totalCRC) {
		return false;
	}
	return true;
}


bool cFFBDevice::saveConfigsToFlash() {
	// unlock flash
	HAL_FLASH_Unlock();

	// erase flash page
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP|FLASH_FLAG_PGPERR|FLASH_FLAG_WRPERR|FLASH_FLAG_PGSERR);
	EraseInitStruct.TypeErase = TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange = VOLTAGE_RANGE_3;
	uint32_t FirstSector = GetSector(SETTINGSSTARTADDRESS);
	uint32_t SectorError = 0;
	EraseInitStruct.Sector = FirstSector;
	EraseInitStruct.NbSectors = 1;//NbOfSectors;
	//HAL_FLASH_ErasePage(startAddress);
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
	{
		if(debugMode) printf("Flash Sector Erase Error occurred.\r\n");
		volatile uint32_t errorcode = HAL_FLASH_GetError();
		simucubelog.addEvent(flashWriteFailure, errorcode);
		HAL_FLASH_Lock();
		return false;
	}

	// write data.
	uint32_t address = SETTINGSSTARTADDRESS;

	uint8_t s='S';
	uint8_t m='M';
	uint8_t c='C';
	uint8_t b='B';
	// first write magic string
	if (HAL_FLASH_Program(TYPEPROGRAM_BYTE, address, s) != HAL_OK) {
		if(debugMode) printf("Flash Sector Write Error 1 occurred.\r\n");
		HAL_FLASH_Lock();
		simucubelog.addEventParam(flashWriteFailure,1);
		return false;
	}
	address++;
	if (HAL_FLASH_Program(TYPEPROGRAM_BYTE, address, m) != HAL_OK) {
		if(debugMode) printf("Flash Sector Write Error 2 occurred.\r\n");
		HAL_FLASH_Lock();
		simucubelog.addEventParam(flashWriteFailure,2);
		return false;
	}
	address++;
	if (HAL_FLASH_Program(TYPEPROGRAM_BYTE, address, c) != HAL_OK) {
		if(debugMode) printf("Flash Sector Write Error 3 occurred.\r\n");
		HAL_FLASH_Lock();
		simucubelog.addEventParam(flashWriteFailure,3);
		return false;
	}
	address++;
	if (HAL_FLASH_Program(TYPEPROGRAM_BYTE, address, b) != HAL_OK) {
		if(debugMode) printf("Flash Sector Write Error 4 occurred.\r\n");
		HAL_FLASH_Lock();
		simucubelog.addEventParam(flashWriteFailure,4);
		return false;
	}
	address++;


	// write version info
	if (HAL_FLASH_Program(TYPEPROGRAM_BYTE, address, firmwareVersion[majorVersionIdx]) != HAL_OK) {
		if(debugMode) printf("Flash Sector Write Error 5 occurred.\r\n");
		HAL_FLASH_Lock();
		simucubelog.addEventParam(flashWriteFailure,5);
		return false;
	}
	address++;
	if (HAL_FLASH_Program(TYPEPROGRAM_BYTE, address, firmwareVersion[minorVersionIdx]) != HAL_OK) {
		if(debugMode) printf("Flash Sector Write Error 6 occurred.\r\n");
		HAL_FLASH_Lock();
		simucubelog.addEventParam(flashWriteFailure,6);
		return false;
	}
	address++;
	if (HAL_FLASH_Program(TYPEPROGRAM_BYTE, address, firmwareVersion[buildVersionIdx]) != HAL_OK) {
		if(debugMode) printf("Flash Sector Write Error 7 occurred.\r\n");
		HAL_FLASH_Lock();
		simucubelog.addEventParam(flashWriteFailure,7);
		return false;
	}
	address++;

	// for align
	if(!fillFlashZeroes(address, 1)) {
		if(debugMode) printf("Flash Sector Write Error 8 occurred.\r\n");
		HAL_FLASH_Lock();
		simucubelog.addEventParam(flashWriteFailure,8);
		return false;
	}
	address++;
	if(!fillFlashZeroes(address, HWSETTINGSADDRESS-address)) {
		if(debugMode) printf("Flash Sector Write Error 9 occurred.\r\n");
		HAL_FLASH_Lock();
		simucubelog.addEventParam(flashWriteFailure,9);
		return false;
	}


	address = HWSETTINGSADDRESS;
	for(int i = 0; i< numberOfHwSettingsAddrs; i++) {
		int32_t value = mConfig.hardwareConfig.hwSettings[i];
		if(HAL_FLASH_Program(TYPEPROGRAM_WORD, address, value) != HAL_OK) {
			if(debugMode) printf("Flash Sector Write Error 10 occurred.\r\n");
			HAL_FLASH_Lock();
			simucubelog.addEventParam(flashWriteFailure,10);
			return false;
		}
		address+=4;
	}
	//bytestore
	for(int i = 0; i<256; i++) {
		uint8_t value = mConfig.hardwareConfig.bytestore[i];
		if(HAL_FLASH_Program(TYPEPROGRAM_BYTE, address, value) != HAL_OK) {
			HAL_FLASH_Lock();
			if(debugMode) printf("Flash Sector Write Error 11 occurred.\r\n");
			simucubelog.addEventParam(flashWriteFailure,11);
			return false;
		}
		address++;
	}

	if(!fillFlashZeroes(address, ANALOGSETTINGSADDRESS-address)) {
		if(debugMode) printf("Flash Sector Write Error 12 occurred.\r\n");
		HAL_FLASH_Lock();
		simucubelog.addEventParam(flashWriteFailure,12);
		return false;
	}

	address=ANALOGSETTINGSADDRESS;
	// analog configs
	for(int analogconf = 0; analogconf<maxnumanalogconfigs; analogconf++) {
		for(int analogaxis = 0; analogaxis<7; analogaxis++) {
			for(int analogsetting = 0; analogsetting<numberOfAnalogSettingsAddrs; analogsetting++) {
				int32_t value = mConfig.analogConfig[analogconf].axises[analogaxis].settings[analogsetting];
				if(HAL_FLASH_Program(TYPEPROGRAM_WORD, address, value) != HAL_OK) {
					HAL_FLASH_Lock();
					if(debugMode) printf("Flash Sector Write Error 13 occurred.\r\n");
					simucubelog.addEventParam(flashWriteFailure,13);
					return false;
				}
				address+=4;
			}
		}
		for(int bytestoreint = 0; bytestoreint<32; bytestoreint++) {
			uint8_t value = mConfig.analogConfig[analogconf].bytestore[bytestoreint];
			if(HAL_FLASH_Program(TYPEPROGRAM_BYTE, address, value) != HAL_OK) {
				HAL_FLASH_Lock();
				if(debugMode) printf("Flash Sector Write Error 14 occurred.\r\n");
				simucubelog.addEventParam(flashWriteFailure,14);
				return false;
			}
			address++;
		}
	}

	// buttons stuff is not used at all as the configs are stored in the wireless wheel
	if(!fillFlashZeroes(address, BUTTONSSETTINGSADDRESS-address)) {
		if(debugMode) printf("Flash Sector Write Error 15 occurred.\r\n");
		HAL_FLASH_Lock();
		simucubelog.addEventParam(flashWriteFailure,15);
		return false;
	}

	address=BUTTONSSETTINGSADDRESS;
	// buttton configs
	for(int i = 0; i< numberofbuttonsettings; i++) {
		int32_t value = mConfig.buttonConfig.settings[i];
		if(HAL_FLASH_Program(TYPEPROGRAM_WORD, address, value) != HAL_OK) {
			HAL_FLASH_Lock();
			if(debugMode) printf("Flash Sector Write Error 16 occurred.\r\n");
			simucubelog.addEventParam(flashWriteFailure,16);
			return false;
		}
		address+=4;
	}

	if(!fillFlashZeroes(address, PROFILESETTINGSADDRESS-address)) {
		if(debugMode) printf("Flash Sector Write Error 17 occurred.\r\n");
		HAL_FLASH_Lock();
		simucubelog.addEventParam(flashWriteFailure,17);
		return false;
	}

	address = PROFILESETTINGSADDRESS;
	//then write number of profiles number lobyte
	uint8_t byte = numberofprofiles & 0xFF;
	if (HAL_FLASH_Program(TYPEPROGRAM_BYTE, address, byte) != HAL_OK) {
		if(debugMode) printf("Flash Sector Write Error 18 occurred.\r\n");
		HAL_FLASH_Lock();
		simucubelog.addEventParam(flashWriteFailure,18);
		return false;
	}
	address++;
	// hibyte
	byte = (numberofprofiles >> 8) & 0xFF;
	if (HAL_FLASH_Program(TYPEPROGRAM_BYTE, address, byte) != HAL_OK) {
		if(debugMode) printf("Flash Sector Write Error 19 occurred.\r\n");
		HAL_FLASH_Lock();
		simucubelog.addEventParam(flashWriteFailure,19);
		return false;
	}
	address++;
	//then write default profile number lobyte
	byte = defaultprofileindex & 0xFF;
	if (HAL_FLASH_Program(TYPEPROGRAM_BYTE, address, byte) != HAL_OK) {
		if(debugMode) printf("Flash Sector Write Error 20 occurred.\r\n");
		HAL_FLASH_Lock();
		simucubelog.addEventParam(flashWriteFailure,20);
		return false;
	}
	address++;
	// hibyte
	byte = (defaultprofileindex >> 8) & 0xFF;
	if (HAL_FLASH_Program(TYPEPROGRAM_BYTE, address, byte) != HAL_OK) {
		if(debugMode) printf("Flash Sector Write Error 21 occurred.\r\n");
		HAL_FLASH_Lock();
		simucubelog.addEventParam(flashWriteFailure,21);
		return false;
	}
	address++;


	//then write profile configs

	for(int profileidx = 0; profileidx<maxnumprofiles_v11; profileidx++) {
		//addrs
		for(int profileaddr=0; profileaddr<numberOfProfAddrs; profileaddr++) {
			int32_t value = mConfig.profileConfigs[profileidx].settings[profileaddr];
			if(HAL_FLASH_Program(TYPEPROGRAM_WORD, address, value) != HAL_OK) {
				HAL_FLASH_Lock();
				if(debugMode) printf("Flash Sector Write Error 22 occurred.\r\n");
				simucubelog.addEventParam(flashWriteFailure,22);
				return false;
			}
			address+=4;
		}
		//names
		for(int namebyte=0; namebyte<32; namebyte++) {
			uint8_t byte = mConfig.profileConfigs[profileidx].profilename[namebyte];
			if (HAL_FLASH_Program(TYPEPROGRAM_BYTE, address, byte) != HAL_OK) {
				if(debugMode) printf("Flash Sector Write Error 23 occurred.\r\n");
				HAL_FLASH_Lock();
				simucubelog.addEventParam(flashWriteFailure,23);
				return false;
			}
			address++;
		}
		//bytes
		for(int storebyte=0; storebyte<32; storebyte++) {
			uint8_t byte = mConfig.profileConfigs[profileidx].bytestore[storebyte];
			if (HAL_FLASH_Program(TYPEPROGRAM_BYTE, address, byte) != HAL_OK) {
				if(debugMode) printf("Flash Sector Write Error 24 occurred.\r\n");
				HAL_FLASH_Lock();
				simucubelog.addEventParam(flashWriteFailure,24);
				return false;
			}
			address++;
		}
	}

	if(!fillFlashZeroes(address, SETTINGSENDADDRESS -4-address)) {
		if(debugMode) printf("Flash Sector Write Error 25 occurred.\r\n");
		HAL_FLASH_Lock();
		simucubelog.addEventParam(flashWriteFailure,25);
		return false;
	}

	// save flash data crc value
	uint32_t totalCRC = crc32a((uint8_t*)SETTINGSSTARTADDRESS, SETTINGSENDADDRESS - 4 - SETTINGSSTARTADDRESS);

	if(debugMode) printf("saving %u\r\n", totalCRC);

	uint32_t savedCRCaddr = SETTINGSENDADDRESS -4;
	if(HAL_FLASH_Program(TYPEPROGRAM_WORD, savedCRCaddr, totalCRC) != HAL_OK) {
		HAL_FLASH_Lock();
		if(debugMode) printf("Flash Sector Write Error 26 occurred.\r\n");
		simucubelog.addEventParam(flashWriteFailure,26);
		return false;
	}
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
	int returnvalue = USBD_BUSY;
	uint8_t command = data[1];
	switch(command) {
		case nopCommand:
		{
			if(debugMode) printf("nopcommand\r\n");
			break;
		}
		case requestStatus: //statusupdate
		{
			statusreply.reportID = inReport;
			statusreply.command = replyStatus,
			statusreply.majorVersion = firmwareVersion[majorVersionIdx];
			statusreply.minorVersion = firmwareVersion[minorVersionIdx];
			statusreply.buildVersion = firmwareVersion[buildVersionIdx];
			statusreply.SimuCubeStatus = currentSystemStatus;
			statusreply.drcReceivedBytes = IoniDrcDataReceivedBytes;
			statusreply.DriveFWUploadPercentage = firmwareuploadingstatuspercentage;
			statusreply.DriveFWVersion = IoniFWVersion;
			statusreply.hwVersion = scHWVersion;
			statusreply.motorfaults = motorFaultRegister;

			statusreply.activeProfileIndex = currentprofileindex;
			statusreply.defaultProfileIndex = defaultprofileindex;
			statusreply.numberofprofiles = numberofprofiles;
			// these are automatically calculated in torqueCommand generation. If not doing that
			// and thus not updated, need to interpret that in the config software side?
			statusreply.driveStatus = driveStatusBits;

			statusreply.simucubeStatusBits  = 0;
            statusreply.simucubeStatusBits2 = 0;

			statusreply.simucubeStatusBits |= (unsavedSettings<<unsavedSettingsBit);
			statusreply.simucubeStatusBits |= (_indexpointFound<<indexpointFoundBit);
			statusreply.simucubeStatusBits |= (driveInitSuccessFlag<<initSuccessBit);

			statusreply.simucubeStatusBits |= (forcesEnabled<<forcesEnabledBit);
			statusreply.simucubeStatusBits |= (bledetected<<bleModuleFoundBit);
			if(bledetected) {
				bool bleconnstate = BLEConn.getConnectionState();
				if(bleconnstate) {
					statusreply.simucubeStatusBits |= (1<<bleWheelConnectedBit);
				}
				bool blescanstate = BLEConn.getScanStatus();
				if(blescanstate) {
					statusreply.simucubeStatusBits |= (1<<bleScanInProgressBit);
				}
			}
			statusreply.simucubeStatusBits |= (wirelessTorqueOffEvent<<wirelessTorqueOffEventBit);
			statusreply.simucubeStatusBits |= (wirelessTorqueOffButton<<wirelessTorqueOffButtonBit);
			statusreply.simucubeStatusBits |= (STOStatus<<stoStatusBit);
			statusreply.simucubeStatusBits |= (torqueFailed<<smPermanentFaultBit);
			statusreply.simucubeStatusBits |= (torqueOffSavingDelay<<torqueOffSavingBit);
			statusreply.simucubeStatusBits |= (temporarySteeringAngleIsSet<<temporarySteeringAngleBit);
			statusreply.simucubeStatusBits |= (_swwIdleDisonnect<<bleAutoDisconnectedBit);

			if(endstop_effect.unsafe()) {
                statusreply.simucubeStatusBits2 = 1; // bit 0
            }

			statusreply.motorFaultLocationID = faultLocationID;
			statusreply.debugvalue1 = debugvalue1_;
			statusreply.debugvalue2 = endstop_effect.direction();//globaldebugvalue2;
			statusreply.logVerbosityMode = simucubelog.getVerbosity();

			statusreply.ffbEffectsInUse = ffbEffectUsage;
			statusreply.ffbEffectsInActiveUse = ffbEffectActive;
			statusreply.driveTypeID=DeviceTypeID;

			int counter = 0;
			while (returnvalue != USBD_OK) {												   // remember to always send 61, otherwise microsoft hid discards it!
				counter++;
				returnvalue = USBD_CUSTOM_HID_SendReport( &hUsbDeviceFS, (uint8_t*)&statusreply, 61);
				if(returnvalue == USBD_BUSY) {
					if(debugMode2)printf(".");

					if(currentSystemStatus == Operational) {
						//printf("!\r\n");
						if(!ServoDriveDisabled) {
							CalcTorqueCommand(); //reads encoder counter too
						}
					}
				}
			}
			// this is important so that any next USBD_CUSTOM_HID_SendReport command won't get to overwrite the pointer. Also, this must be kept fast enough so that simplemotion won't reset.
			start_timer2_15ms_delay();
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

			eventreply.reportID = inReport;
			eventreply.command = replyEventLog;
			eventreply.latestEvent = simucubelog.lastEventIndex;
			for(int i=0;i<9;i++) {
				if(offset+i > logSize-1) {
					// log is at end.
					break;
				}
				simucubelog.getEvent(offset+i, ev, param);//reply.events[i], reply.parameters[i]);
				eventreply.events[i]=ev;
				eventreply.parameters[i]=param;
			}
			while (returnvalue != USBD_OK) {												   // remember to always send 61, otherwise microsoft hid discards it!
				returnvalue = USBD_CUSTOM_HID_SendReport( &hUsbDeviceFS, (uint8_t*)&eventreply, 61);//sizeof(statusReplyPacket));
				if(returnvalue == USBD_BUSY) {if(debugMode2)printf(".");}

				if(currentSystemStatus == Operational) {
					if(!ServoDriveDisabled) {
						CalcTorqueCommand(); //reads encoder counter too
					}
				}
			}
			start_timer2_15ms_delay();
			return true;
			break;
		}

		case startEventLogging:
		{
			if(debugMode) printf(" startlogging\r\n");
			simucubelog.addEvent(Command_startLogging);
			simucubelog.setEnabled();
			return true;
			break;
		}

		case stopEventLogging:
		{
			if(debugMode) printf(" stoplogging\r\n");
			simucubelog.addEvent(Command_stopLogging);
			simucubelog.setDisabled();
			return true;
			break;
		}

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
		{
			simucubelog.addEvent(Command_startIRFFBmode);
			IRFFBModeEnabled = true;
			return true;
			break;
		}

		case stopIRFFBMode:
		{
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
		}

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
			// else, was in some configuration / idle mode, or fault mode, and that doesn't need to be changed.
			unsavedSettings = 0;
			return true;
			break;
		}

		case unsetSettingsChanged:
		{
			if(debugMode) printf(" unsetsettingschanged\r\n");
			simucubelog.addEvent(Command_unsetSettingChanged);
			unsavedSettings = 0;
			return true;
			break;
		}

		case transferIoniDrcData:
		{
			if(debugMode) printf(" ionidrcdata");

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
			if(debugMode) printf(" applyionidrcdata\r\n");
			simucubelog.addEvent(Command_ApplyIoniConfig);
			if(firmwareuploadinprogress) {
				simucubelog.addEventParam(Command_ApplyIoniConfigFail,0);
				if(debugMode2) printf(" ioni firmware upgrading!! can't do this.\r\n");
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
			return true;
			break;
		}


		case sethwdata:
		{
			if(debugMode) printf(" set hw data\r\n");
			simucubelog.addEvent(Command_setHardware);
			setSettingsDataPacket* packet = (setSettingsDataPacket*)data;
			bool settings = true;
			bool driveSettings1Changed = false;
			for(uint8_t i=0; i<9; i++) {
				if((packet->addrs[i]==0) || (packet->addrs[i]>(numberOfHwSettingsAddrs-1))) {
					if(i==0) {
						settings=false;
					}
					break;
				}
				mConfig.hardwareConfig.hwSettings[packet->addrs[i]]=packet->values[i];

				int addr = packet->addrs[i];
				if(addr == addrVariousSettingsBits1) driveSettings1Changed=true;
			}
			if(!settings) {
				// no settings at all in the packet??
				return true;
				break;
			}
			if(driveSettings1Changed) {
				// set beeps according to mode
				s32 parametervalue;
				smRead1Parameter(mSMBusHandle, 1, SMP_DRIVE_FLAGS, &parametervalue);
				if(mConfig.hardwareConfig.hwSettings[addrVariousSettingsBits1]&(1<<bitAudibleNotificationsEna)) {
					parametervalue = parametervalue | FLAG_ENABLE_MOTOR_SOUND_NOTIFICATIONS;
				} else {
					parametervalue = parametervalue & ~FLAG_ENABLE_MOTOR_SOUND_NOTIFICATIONS;
				}
				if(debugMode) printf("set hardware parameters: audible beeb flag in SMP_DRIVE_FLAGS %lu to drive\r\n", parametervalue);
				smSetParameter(mSMBusHandle, 1, SMP_DRIVE_FLAGS, parametervalue);
			}

			if(currentSystemStatus==Operational) {
				currentSystemStatus = BeforeOperational;
			} else {
				initVariables(); // was not in operational mode, so just init variables separately
			}
			unsavedSettings=1;
			return false;
			break;
		}

		case setanalogdata:
		{
			if(debugMode) printf(" set analog data\r\n");
			setSettingsDataPacket* packet = (setSettingsDataPacket*)data;
			uint8_t configIndex=packet->value1;
			simucubelog.addEventParam(Command_setAnalog, configIndex);
			if(configIndex>maxnumanalogconfigs-1) {
				configIndex=0;
				break;
			}
			uint8_t axisIndex = packet->value2;
			if(axisIndex>7) {
				axisIndex=0;
				break;
			}
			for(uint8_t i = 0; i<9; i++){
				if((packet->addrs[i]==0) || (packet->addrs[i]>(numberOfAnalogSettingsAddrs-1))) {
					break;
				}
				mConfig.analogConfig[configIndex].axises[axisIndex].settings[packet->addrs[i]]=packet->values[i];
			}

			unsavedSettings=1;
			return true;
			break;
		}


		case setprofiledata:
		{
			setSettingsDataPacket* packet = (setSettingsDataPacket*)data;
			uint8_t profileidx = packet->value1;
			if(debugMode) printf(" setprofile %d\r\n", profileidx);
			simucubelog.addEventParam(Command_setProfile, profileidx);
			if(profileidx==0) {
				// do not overwrite the 0 profile
				return false;
				break;
			}
			bool settings = true;
			for(uint8_t i = 0; i<9; i++) {
				if((packet->addrs[i]==0) || (packet->addrs[i]>(numberOfProfAddrs-1))) {
					if(i==0) {
						settings=false;
					}
					break;
				}
				mConfig.profileConfigs[profileidx].settings[packet->addrs[i]]=packet->values[i];
			}
			if(!settings) {
				return true; // no settings at all?
				break;
			}
			unsavedSettings = 1;
			if (profileidx != currentprofileindex) {
				return true;
				break;
			}
			/* was in current profile, must do the beforeOperational stuff
			 * but only if the state is such that the beforeOperational is not run anyway
			 * by state machine progression (drive init->setbaudrate->beforeoperational->operational).
			 * The beforeOperational state is always run before going to operational.
			 */
			if(currentSystemStatus==Operational) {
				currentSystemStatus = BeforeOperational;
				return false; // return false to indicate that state change is upcoming
			} else {
				initVariables(); // was not in operational mode, so just init variables separately
				return true;
			}
			break;
		}
		case setprofilename:
		{
			ProfileNameReplyPacket* packet = (ProfileNameReplyPacket*) data;
			if(debugMode) printf(" set profile %d name\r\n", packet->value1);
			uint16_t profileidx = packet->value1;
			simucubelog.addEventParam(Command_setProfileName, profileidx);
			if(profileidx==0) {
				return true;
				break;
			}
			for(uint8_t i =0; i<32; i++) {
				mConfig.profileConfigs[profileidx].profilename[i]=packet->bytes[i];
			}
			unsavedSettings = 1;
			return true;
			break;
		}
		case setanalogbytedata:
		{
			ProfileNameReplyPacket* packet = (ProfileNameReplyPacket*) data;
			if(debugMode) printf(" set analog bytedata for set %d\r\n", packet->value1);
			uint16_t idx = packet->value1;
			simucubelog.addEventParam(Command_setAnalogByteData, idx);
			for(uint8_t i =0; i<32; i++) {
				mConfig.analogConfig[idx].bytestore[i]=packet->bytes[i];
			}
			unsavedSettings = 1;
			return true;
			break;
		}
		case setprofilebytedata:
		{
			if(debugMode) printf(" set profile bytedata\r\n");
			ProfileNameReplyPacket* packet = (ProfileNameReplyPacket*) data;
			uint16_t profileidx = packet->value1;
			simucubelog.addEventParam(Command_setProfileBytedata, profileidx);
			if(profileidx==0) {
				return true;
				break;
			}
			for(uint8_t i =0; i<32; i++) {
				mConfig.profileConfigs[profileidx].bytestore[i]=packet->bytes[i];
			}
			unsavedSettings = 1;
			return true;
			break;
		}
		case requesthwdata:
		{
			if(debugMode) printf(" request hw data\r\n");
			simucubelog.addEvent(Command_requestHardware);
			requestSettingsDataPacket* packet = (requestSettingsDataPacket*) data;
			settingsdatareply.reportID = inReport;
			settingsdatareply.command = datareplypacketid;
			for(uint8_t i = 0; i<9; i++) {
				if (packet->addrs[i]==0) {
					settingsdatareply.addrs[i]=0;
					continue;
				}
				settingsdatareply.addrs[i] = packet->addrs[i];
				settingsdatareply.values[i]= mConfig.hardwareConfig.hwSettings[packet->addrs[i]];
			}
			while (returnvalue != USBD_OK) {
				returnvalue = USBD_CUSTOM_HID_SendReport( &hUsbDeviceFS, (uint8_t*)&settingsdatareply, 61);
				if(returnvalue == USBD_BUSY) {if(debugMode2)printf(".");}
			}
			return true;
			break;
		}
		case requestanalogdata:
		{
			simucubelog.addEvent(Command_requestAnalog);
			requestSettingsDataPacket* packet = (requestSettingsDataPacket*) data;
			settingsdatareply.reportID = inReport;
			settingsdatareply.command = datareplypacketid;
			for(uint8_t i = 0; i<9; i++) {
				if (packet->addrs[i]==0) {
					settingsdatareply.addrs[i]=0;
					continue;
				}
				settingsdatareply.addrs[i] = packet->addrs[i];
				settingsdatareply.values[i]= mConfig.analogConfig[packet->value1].axises[packet->value2].settings[packet->addrs[i]];
			}
			while (returnvalue != USBD_OK) {
				returnvalue = USBD_CUSTOM_HID_SendReport( &hUsbDeviceFS, (uint8_t*)&settingsdatareply, 61);
				if(returnvalue == USBD_BUSY) {if(debugMode2)printf(".");}
			}
			return true;
			break;

		}
		case requestprofiledata:
		{
			requestSettingsDataPacket* packet = (requestSettingsDataPacket*) data;
			settingsdatareply.reportID = inReport;
			settingsdatareply.command = datareplypacketid;
			if(debugMode) printf(" request profile data %d\r\n", packet->value1);
			simucubelog.addEventParam(Command_requestProfile, packet->value1);
			for(uint8_t i = 0; i<9; i++) {
				if (packet->addrs[i]==0) {
					settingsdatareply.addrs[i]=0;
					continue;
				}
				settingsdatareply.addrs[i] = packet->addrs[i];
				settingsdatareply.values[i]= mConfig.profileConfigs[packet->value1].settings[packet->addrs[i]];
			}
			while (returnvalue != USBD_OK) {
				returnvalue = USBD_CUSTOM_HID_SendReport( &hUsbDeviceFS, (uint8_t*)&settingsdatareply, 61);
				if(returnvalue == USBD_BUSY) {if(debugMode2)printf(".");}
			}
			return true;
			break;
		}
		case requestprofilename:
		{
			commandPacket* packet = (commandPacket*) data;
			namereply.reportID = inReport;
			namereply.command = bytedatareplypacketid;
			if(debugMode) printf(" request profile name %u\r\n", packet->value);
			simucubelog.addEventParam(Command_requestProfileName, packet->value);
			for(uint8_t i = 0; i<32; i++) {
				namereply.bytes[i]=mConfig.profileConfigs[packet->value].profilename[i];
			}

			while (returnvalue != USBD_OK) {
				returnvalue = USBD_CUSTOM_HID_SendReport( &hUsbDeviceFS, (uint8_t*)&namereply, 61);
				if(returnvalue == USBD_BUSY) {if(debugMode2)printf(".");}
			}
			return true;
			break;
		}
		case requestanalogbytedata:
		{
			commandPacket* packet = (commandPacket*) data;
			namereply.reportID = inReport;
			namereply.command = bytedatareplypacketid;
			if(debugMode) printf(" request analog byte data %u\r\n", packet->value);
			simucubelog.addEventParam(Command_requestAnalogByteData, packet->value);
			for(uint8_t i = 0; i<32; i++) {
				namereply.bytes[i]=mConfig.analogConfig[packet->value].bytestore[i];
			}

			while (returnvalue != USBD_OK) {
				returnvalue = USBD_CUSTOM_HID_SendReport( &hUsbDeviceFS, (uint8_t*)&namereply, 61);
				if(returnvalue == USBD_BUSY) {if(debugMode2)printf(".");}
			}
			return true;
			break;
		}
		case requestprofilebytedata:
		{
			commandPacket* packet = (commandPacket*) data;

			bytedatareply.reportID = inReport;
			bytedatareply.command = bytedatareplypacketid;
			if(debugMode) printf(" request profile name %u\r\n", packet->value);
			simucubelog.addEventParam(Command_requestProfileByteData, packet->value);
			for(uint8_t i = 0; i<32; i++) {
				bytedatareply.bytes[i]=mConfig.profileConfigs[packet->value].bytestore[i];
			}

			while (returnvalue != USBD_OK) {
				returnvalue = USBD_CUSTOM_HID_SendReport( &hUsbDeviceFS, (uint8_t*)&bytedatareply, 61);
				if(returnvalue == USBD_BUSY) {if(debugMode2)printf(".");}
			}
			return true;
			break;
		}
		case freshStart:
		{
			if(debugMode) printf(" freshstart\r\n");
			simucubelog.addEvent(Command_goFreshStart);
			currentSystemStatus = goFreshStart;
			return false;
			break;
		}
		case enableSMUSB:
		{
			if(debugMode) printf(" enablesmusb\r\n");
			simucubelog.addEvent(Command_enableSMUSB);
			// set wheel forces to off!
			if(currentSystemStatus==Operational) {
				SetTorque(0, 0);
			}
			nextSystemStatus = currentSystemStatus;
			currentSystemStatus = releaseSMBus;
			return false;
			break;
		}
		case disableSMUSB:
		{
			if(debugMode) printf(" disablesmusb\r\n");
			if(_smReleased) {
				simucubelog.addEventParam(Command_disableSMUSB, 1);
				currentSystemStatus = regainSMBus;
				return false;
			} else {
				if(debugMode) printf(" not already released smbus, so do nothing.\r\n");
				simucubelog.addEventParam(Command_disableSMUSB, 0);
				return true;
			}
			break;
		}
		case requestRawAnalogAxis:
		{
			if(debugMode) printf(" requestrawanalog\r\n");
			simucubelog.addEvent(Command_setRawAnalogMode);
			rawAnalogMode = true;
			return true;
			break;
		}
		case requestCalibratedAnalogAxis:
		{
			if(debugMode) printf(" requestcalibratedanalog\r\n");
			simucubelog.addEvent(Command_setCalibAnalogMode);
			rawAnalogMode = false;
			return true;
			break;
		}
		case startDriveInit:
		{
			if(debugMode) printf(" Going to init drive!\r\n");
			simucubelog.addEvent(Command_startDriveInit);
			driveInitSuccessFlag=0;
			forceFirsttimeDriveInit=1; // commanded init from motor config wizard

			currentSystemStatus = DriveInit;
			return false;
			break;
		}
		case restartDrive:
		{
			if(debugMode) printf(" restart drive command!\r\n");
			simucubelog.addEvent(Command_restartDrive);
			commandPacket* packet = (commandPacket*) data;
			uint16_t staystate = packet->value;
			if(staystate==107) {
				restartSMDrive();
				return true;
				break;
			}
			restartSMDrive();
			currentSystemStatus = SystemNotConfigured;
			return false;
			break;
		}
		case setInitialConfigDone:
		{
			if(debugMode) printf(" setting initial config to done!\r\n");
			simucubelog.addEvent(Command_setInitialConfig);
			mConfig.hardwareConfig.hwSettings[addrVariousSettingsBits1]|=(1<<bitInitialConfigDone);
			if(currentSystemStatus == DriveInitSuccessPause) {
				currentSystemStatus = SetBaudrateForOperational;
				return false;
				break;
			}
			return true;
			break;
		}
		case clearInitialConfigDone:
		{
			if(debugMode) printf(" clearing initial config, going to notconfigured mode!\r\n");
			simucubelog.addEvent(Command_unsetInitialConfig);
			mConfig.hardwareConfig.hwSettings[addrVariousSettingsBits1]&=~(1<<bitInitialConfigDone);
			currentSystemStatus = SystemNotConfigured;
			nextSystemStatus = SystemNotConfigured;
			initSMBusBaudrate(false);
			temporaryCenterPoint = false;
			_indexpointFound = 0;
			return false;
			break;
		}
		case setWheelCenterHere:
		{
			if(debugMode) printf(" setwheelcenterhere\r\n");
			simucubelog.addEvent(Command_setWheelCenter);
			setWheelCenter();
			return true;
			break;
		}

		case setTemporaryCenterMode:
		{
			if(debugMode) printf(" setting temporary centering for this session\r\n");
			simucubelog.addEvent(Command_setTempCenter);
			temporaryCenterPoint = true;
			return true;
			break;
		}
		case unsetTemporaryCenterMode:
		{
			if(debugMode) printf(" unsetting temporary centering for this session\r\n");
			simucubelog.addEvent(Command_unsetTempCenter);
			temporaryCenterPoint = false;
			return true;
			break;
		}
		case activateProfile:
		{
			commandPacket* packet = (commandPacket*) data;
			currentprofileindex = packet->value;
			if(debugMode) printf(" activating profile index %u\r\n", currentprofileindex);
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
			if(debugMode) printf(" setting number of profiles to %u\r\n", numberofprofiles);
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
			if(debugMode) printf(" setting default profile index to %u\r\n", defaultprofileindex);
			simucubelog.addEventParam(Command_setDefProfile,defaultprofileindex);
			unsavedSettings = 1;
			return true;
			break;
		}
		case jmpToBootloader:
		{
			if(debugMode) printf(" jumping to bootloader\r\n");
			simucubelog.addEvent(Command_jmpToBootLoader);
			currentSystemStatus = JumpToBootLoader;
			return false;
			break;
		}
		case setForcesDisabled:
		{
			if(debugMode) printf(" set forces disabled\r\n");
			simucubelog.addEvent(Command_disableForces);
			forcesEnabled = 0;
			return true;
			break;
		}
		case setForcesEnabled:
		{
			if(debugMode) printf(" set forces enabled\r\n");
			simucubelog.addEvent(Command_enableForces);
			forcesEnabled = 1;
			return true;
			break;
		}
		case resetFFBvariables:
		{
			if(debugMode) printf(" force-reset ffb device\r\n");
			simucubelog.addEvent(Command_resetFFBStates);
			resetFFBDeviceState();
			//todo: smooth return with todo drive functionality?
			return true;
			break;
		}
		case reEnableTorqueMode:
		{
			if(debugMode) printf(" re-enable torque generation\r\n");
			simucubelog.addEvent(Command_reEnableTorque);
			wirelessTorqueOffEvent=0;
			return true;
			break;
		}
		case startCommutationAutoSetup:
		{
			if(debugMode) printf(" going to autosetup commucation\r\n");
			simucubelog.addEvent(Command_setupCommutation);
			previousSystemStatus2 = currentSystemStatus;
			currentSystemStatus = AutoDetectCommutationSensors;
			return false;
			break;
		}
		case clearCommutationAutoSetup:
		{
			if(debugMode) printf(" clear autosetup commucation\r\n");
			simucubelog.addEvent(Command_clearCommutation);
			clearCommutationConfig(); //also saves drive cfg
			mConfig.hardwareConfig.hwSettings[addrVariousSettingsBits1]&=~(bitAutoCommutationMode);
			mConfig.hardwareConfig.hwSettings[addrIndexingMode]=startAtCenterPhasedIndexing;
			unsavedSettings=1;
			return false;
			break;
		}
		case setTemporaryVariable:
		{
			if(debugMode) printf(" set temporary variable\r\n");
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
					return true;
					break;
			}
			return true;
			break;
		}



		case startScanning:
		{
			handleStartScanningCmd();
			return true;
			break;
		}

		case stopScanning:
		{
			handleStopDiscoveryCmd();
			return true;
			break;
		}

		case getNumberOfFoundDevices:
		{
			handleGetNumberOfFoundDevicesCmd();
			return true;
			break;
		}

		case getDeviceData:
		{
			handleGetDeviceDataCmd(data);
			return true;
			break;
		}

		case getCurrentDeviceData:
		{
			handleGetCurrentDeviceDataCmd();
			return true;
			break;
		}
		case connectDevice:
		{
			handleConnectDeviceCmd(data);
			return true;
			break;
		}
		case disconnectDevice:
		{
			handleDisconnectDeviceCmd();
			return true;
			break;
		}
		case disconnectForgetDevice:
		{
			handleDisconnectForgetDeviceCmd();
			return true;
			break;
		}
		case setDriveParams:
		{
			if(debugMode) printf(" set Drive parameters");
			simucubelog.addEvent(Command_writeDriveParams, true);
			setSettingsDataPacket* packet = (setSettingsDataPacket*) data;
			if(true) {
				// proprietary simucube 2 command
			} else {
				for(int i=0;i<9;i++) {
					if(packet->addrs[i]==0) {
						break;
					}
					while(waitLastSimpleMotion) {
						asm volatile("nop");
					}
					SM_STATUS status = smSetParameter(mSMBusHandle, 1, packet->addrs[i], packet->values[i]);
					if(status!=SM_OK) {
						break;
					}
				}
			}
			return true;
			break;
		}
		case startReadDriveParams:
		{
			if(debugMode) printf(" read Drive parameters");
			simucubelog.addEvent(Command_startReadDriveParams, true);
			SM_STATUS statuss;
			setSettingsDataPacket* packet = (setSettingsDataPacket*) data;
			for(int i=0;i<9;i++) {
				drivedatareply.addrs[i]=0;drivedatareply.values[i]=0;
			}
			for(int i=0;i<9;i++) {
				if(packet->addrs[i]==0) {
					break;
				}
				while(waitLastSimpleMotion) {
					asm volatile("nop");
				}

				statuss = smRead1Parameter(mSMBusHandle, 1, packet->addrs[i], &drivedatareply.values[i]);
				if(statuss!=SM_OK) {
					if(debugMode) printf(" smbus error\r\n");
					return true;
					break;
				}
				drivedatareply.addrs[i]=packet->addrs[i];
			}
			printf("\r\n");
			return true;
			break;
		}
		case requestReadDriveParams:
		{
			if(debugMode) printf(" request last read drive parameter values\r\n");
			simucubelog.addEvent(Command_readDriveParams, true);
			drivedatareply.reportID=inReport;
			drivedatareply.command=replyReadDriveParams;

			while (returnvalue != USBD_OK) {
				returnvalue = USBD_CUSTOM_HID_SendReport( &hUsbDeviceFS, (uint8_t*)&drivedatareply, 61);
				if(returnvalue == USBD_BUSY) {if(debugMode2)printf(".");}
			}
			return true;
			break;
		}
		default:
			break;
	}
	return true;
}

void cFFBDevice::setBLEAutoConnectMode() {
    // init BLE autoconnect mode
    ConnectAutomaticallyToWirelessWheel mode = static_cast<ConnectAutomaticallyToWirelessWheel>(mConfig.hardwareConfig.hwSettings[addrAutoConnectBLEDevice]);
    BLEConn.setAutomaticConnectionMode(mode);
}

void cFFBDevice::handleStartScanningCmd() {
	BLEConn.clearDeviceList();
	BLEConn.startDiscovery();
}

void cFFBDevice::handleStopDiscoveryCmd() {
	BLEConn.stopDiscovery();
	BLEConn.clearDeviceList();
}

void cFFBDevice::handleGetNumberOfFoundDevicesCmd(){
	int returnvalue = USBD_BUSY;
	replyPacket.reportID = inReport;
	replyPacket.command = replyNumberofFoundDevices;
	replyPacket.value = BLEConn.getAmountOfFoundDevices();
	while (returnvalue != USBD_OK) {
		returnvalue = USBD_CUSTOM_HID_SendReport( &hUsbDeviceFS, (uint8_t*)&replyPacket, 61);
		if(returnvalue == USBD_BUSY) {if(debugMode2)printf(".");}
	}
}

void cFFBDevice::handleGetDeviceDataCmd(uint8_t* data) {
	int returnvalue = USBD_BUSY;
	commandPacket* packet = (commandPacket*) data;
	int number = packet->value;


	blePacket.reportID = inReport;
	blePacket.command = replyDeviceData;
	BTDevice bledevicedata = BLEConn.getDeviceDataByNumber(number);
	uint8_t* pointer = (uint8_t*)&bledevicedata.address;
	for(int i = 0; i<6; i++) {
		blePacket.mac[i] = *pointer;
		pointer++;
	}
	blePacket.mac[6] = 0;
	blePacket.mac[7] = 0;

	for(int i = 0; i<10; i++) {
		volatile uint8_t character = bledevicedata.name[i];
		blePacket.devicename[i] = character;
	}
	blePacket.devicename[10] = 0;
	blePacket.signalstrength = bledevicedata.rssi;
	blePacket.bond = bledevicedata.bonding;
	blePacket.namelength = bledevicedata.nameLength;
	while (returnvalue != USBD_OK) {
		returnvalue = USBD_CUSTOM_HID_SendReport( &hUsbDeviceFS, (uint8_t*)&blePacket, 61);
		if(returnvalue == USBD_BUSY) {if(debugMode2)printf(".");}
	}
}

void cFFBDevice::handleGetCurrentDeviceDataCmd() {
	int returnvalue = USBD_BUSY;
	blePacket.reportID = inReport;
	blePacket.command = replyDeviceData;
	bool connected = BLEConn.getConnectionState();
	BTDevice bledevicedata = BLEConn.getConnectedDeviceInformation();
	uint8_t* pointer = (uint8_t*)&bledevicedata.address;
	for(int i = 0; i<6; i++) {
		if(connected) {
			blePacket.mac[i] = *pointer;
		} else {
			blePacket.mac[i] = 0;
		}
		pointer++;
	}
	blePacket.mac[6] = 0;
	blePacket.mac[7] = 0;

	for(int i = 0; i<10; i++) {
		volatile uint8_t character;
		if(connected) {
			character = bledevicedata.name[i];
		} else {
			character=0;
		}
		blePacket.devicename[i] = character;
	}
	blePacket.devicename[10] = 0;
	blePacket.signalstrength = bledevicedata.rssi;
	blePacket.bond = bledevicedata.bonding;
	blePacket.namelength = bledevicedata.nameLength;
	int level = BLEConn.getBatteryVoltage();
	level = (level - 2000) / 10; // 2.0 V and up in hundreths of volts
	level = constrain(level, 0, 255);
	blePacket.batterylevel = level;
	blePacket.connectionquality = BLEConn.getConnectionQuality();
	blePacket.batteryMeasurementStatus = BLEConn.getLowBatteryWarning();
	while (returnvalue != USBD_OK) {
		returnvalue = USBD_CUSTOM_HID_SendReport( &hUsbDeviceFS, (uint8_t*)&blePacket, 61);
		if(returnvalue == USBD_BUSY) {if(debugMode2)printf(".");}
	}

}

void cFFBDevice::handleConnectDeviceCmd(uint8_t* data) {
	commandPacket* packet = (commandPacket*) data;
	volatile int number = packet->value;
	BLEConn.connectByDeviceNumber(number);
}

void cFFBDevice::handleDisconnectDeviceCmd() {
	BLEConn.disconnectFromDevice();
	BLEConn.clearDeviceList();
}

void cFFBDevice::handleDisconnectForgetDeviceCmd() {
	BLEConn.forgetConnectedDevice();
	handleDisconnectDeviceCmd();
}


uint16_t cFFBDevice::getUnsavedSettings() {
	return (unsavedSettings<<unsavedSettingsBit);
}

void cFFBDevice::setIndexpointFound(bool found) {
	if(found) {
		_indexpointFound = 1;
	} else {
		_indexpointFound = 0;
	}
}
bool cFFBDevice::getIndexpointFound() {
	if(_indexpointFound == 0) {
		return false;
	}
	return true;
}

uint16_t cFFBDevice::getDriveInitSuccessFlag() {
	return (driveInitSuccessFlag<<initSuccessBit);
}

uint16_t cFFBDevice::getForceEnabled() {
	return (forcesEnabled<<forcesEnabledBit);
}

uint16_t cFFBDevice::getBleDetected() {
	return (bledetected<<bleModuleFoundBit);
}

uint16_t cFFBDevice::getSTOStatus() {
	return (STOStatus<<stoStatusBit);
}

uint16_t cFFBDevice::getHighTorqueMode() {
	return (highTorqueMode<<highTorqueModeBit);
}


// USB steering value is according to our descriptor unsigned 0..65536 where center is near 32768
uint16_t cFFBDevice::usb_steering_angle() {
	return (uint16_t) angle.getUSBAngle();
}

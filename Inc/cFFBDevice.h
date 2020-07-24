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

#ifndef CFFBDEVICE_H_
#define CFFBDEVICE_H_
#include "types.h"
#include "cDeviceConfig.h"
#include "FfbEngine.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_gpio.h"

// filters
#include "Biquad1storder.h"

// input device types
#include "input.h"


// drive parameter tracker class
#include "driveparametertracker.h"

// debug print handle
extern UART_HandleTypeDef huart1;

// wireless wheel headers
#include "BLEConnection.h"
#include "input_devices.h"

#include "simplemotioncomms.h"

#include "EndstopEffect.h"

#include "encoderangle.h"
#include "swwautodisconnecthandler.h"


// maximum number of seconds for an effect to stay visible to the configuration tool
#define ffbActiveLowpassTimeSeconds 30

// approximate ffb loop rate
#define ffbActiveLowpassTimeCycles ffbActiveLowpassTimeSeconds*DRIVEUPDATERATE

// maximum torque
#define MAX_NORM_TORQUE		0x10000

// how old flash data can be converted to new settingsformat
#define lowestCompatibleMajorFlashData 0
#define lowestCompatibleMinorFlashData 8

extern uint8_t IoniDrcFile[16384];

enum ledcolor {OFF, RED, GREEN, BLUE, YELLOW, BLUEREDBLINK, BLUEBLANKBLINK};

class cFFBDevice {
public:
	cFFBDevice();
	~cFFBDevice();

	// this sets default values to all class variables
	void SetDefault();

	// initializes button input ports and pins
	void initButtonInputs();

	// this sets the ffbengine pointer
	void SetFFB(FfbEngine* handle);

	// debug value get/set
	void setDebugValue1(uint32_t value);
	uint32_t getDebugValue1();

	// set to store smBus released/not state
	void setSmReleased(bool newstate);

	// This is the main loop when SimuCUBE is operational. It parses FFB effects and
	// also writes it to drive.
	bool CalcTorqueCommand();

	/// Pretty much like CalcTorqueCommand but always sends (0, 0) as torque.
	/// Returns true if steering angle was updated
	bool zeroTorqueCommand();

	int32_t getLastTorque() const { return lastTorqueSummed; }

	// this initializes the class variables according to current profile.
	void initVariables();

	// this initializes the hardware encoder button input support if there are such encoders
	void initHwEncoderButtons();

	// update button statuses
	void updateButtons();

	// get number of buttons in the device according to hardware
	int getNumberOfButtons();

	// flash load and save
	bool checkFlashCRC();
	bool loadConfigsFromFlash();
	bool saveConfigsToFlash();
	bool checkIfPre012Data();
	bool checkIfPre10000Data();
	void initUninitializedProfileParametersPre012();
	void resetParametersPre10000();
	void adjustAnyWheelConnection();

	uint32_t reverse(uint32_t x);
	uint32_t crc32a(const uint8_t *message, uint32_t datalength);

	// sets wheel center
	void setWheelCenter();

	// some IONI comms
	void readHighTorqueModeSetting();

	// BLE setup
	void setBLEAutoConnectMode();


	// limits effect to a range
	int32_t ConstrainEffect(int32_t val);

	// scales effect for simplemotion torque setpoint range
	int32_t scaleEffectForSimplemotion(float inputval);

	// turns a clipping led on or off if hardware has one
	void setClippingLed(s32 inputValue);

	// turn an endstop/bumbstop led on or off if hardware has one
	void setEndstopLed(bool high);

	// toggle endstop led
	void toggleEndstopLed();

	// handles commands from configuration tool
	bool handleSimuCUBEAPIcommand(uint8_t* data);

	// handles custom IRFFB reports
	bool handleIRFFBcommand(uint8_t* data);


	/* parameter getters and setters */
	// gets current profile parameter
	uint32_t getCurrentProfileParameter(uint32_t addr);

	// gets hardware setting parameter
	uint32_t getHardwareParameter(uint32_t addr);
	// gets hardware setting status bits setting on/off status
	bool getHardwareSettingsBit(uint32_t bit);

	// sets current profile parameter
	void setCurrentProfileParameter(uint32_t addr, uint32_t value);

	// sets hardware setting parameter
	void setHardwareParameter(uint32_t addr, uint32_t value);

	void setHardwareSettingsBit(uint32_t bit, bool on);

	// led colours and handling
	void doLeds();
	void startLed(ledcolor setcolor);
	void ledYellow();
	void ledBlue();
	void ledRed();
	void ledGreen();
	void ledHighTorque();
	void ledOff();
	ledcolor ledCycle[8];
	ledcolor lastLedColor;
	int ledCyclePointer;

	// wireless wheel handling calls.
	// These are also used by factorytester, therefore must be public.
	void handleStartScanningCmd();
	void handleStopDiscoveryCmd();
	void handleGetNumberOfFoundDevicesCmd();
	void handleGetDeviceDataCmd(uint8_t* data);
	void handleGetCurrentDeviceDataCmd();
	void handleConnectDeviceCmd(uint8_t* data);
	void handleDisconnectDeviceCmd();
	void handleDisconnectForgetDeviceCmd();


	// this contains all motor config and profiles
	cDeviceConfig mConfig;

	//smbus communication handle
	smbus mSMBusHandle;

	/* this variable controls whether simplemotion is free for new commmands */
	volatile bool waitLastSimpleMotion;


	/* axis position speed variables */
	float axisSpeedPerMs; // filtered usb units per ms



	/* temporary center point mode */
	bool temporaryCenterPoint;
	int32_t mEncoderTemporaryOffset;

	/* temporary steering angle mode, set by game via our API */
	bool temporarySteeringAngleIsSet;
	uint16_t temporarySteeringLockToLock;

	/* motor control related variables */
	bool getIndexpointFound();
	void setIndexpointFound(bool found);


	smint32 motorFaultRegister;

	uint8_t forcesEnabled;
	uint16_t getForceEnabled();
	uint8_t forcesDisabledForSafety;

	/* analog axis mode */
	bool rawAnalogMode;

	/* variable to keep 2500 Hz sync */
	volatile bool FFBLoopWait;

	/* BLE connection class */
	BLEConnection BLEConn;
	InputDevices inputDeviceList;

	/* stores if BLE device is detected or not */
	uint8_t bledetected;
	uint16_t getBleDetected();

	/* a debug value for UI if needed */
	uint32_t debugvalue1_;

	/* unsaved settings */
	uint8_t unsavedSettings;
	uint16_t getUnsavedSettings();
	/* these are used to store received Ioni DRC Data and simucube hardware version ID */

	uint16_t IoniDrcDataReceivedBytes;
	uint8_t firmwareuploadingstatuspercentage;
	bool firmwareuploadinprogress;
	uint16_t IoniFWVersion;
	uint8_t scHWVersion;
	uint16_t DeviceTypeID;

	// special IRFFB automatic mode
	bool IRFFBModeEnabled;
	uint16_t latestIRFFBForce[6];
	uint32_t lastTelemetryReceived;
	simucubeTelemetryPacket lastSimucubeTelemetryPacket;



	/* buttons */
	uint64_t 		lastButtonUpdateMillis;
	GPIO_TypeDef* 	button_port[16];
	uint16_t 		button_pin[16];
	uint64_t 		button_lastUpdate[16];
	GPIO_PinState	button_currentState[16];
	GPIO_PinState	button_debouncedState[16];
	uint8_t debounce;
	uint8_t pin7shift; // 0=off, 1=lower, 2=upper, others=undef

	/* Encoders */
	Input encx12l1;
	Input encx12l2;
	Input encx12l3;
	Input encx12u1;
	Input encx12u2;
	Input encx12u3;
	uint8_t x12LowerEncodersMode;
	uint8_t x12UpperEncodersMode;


	/* profile management */
	uint16_t currentprofileindex;
	uint16_t numberofprofiles;
	uint16_t defaultprofileindex;
	uint16_t currentanalogconfigindex;
	bool rewriteDefault;


	/* Running FFB Device state variables */
	uint8_t ffbDevGain; // 0-255

	/* reset ffb state */
	void resetFFBDeviceState();

	/* flag to see if drive is initialized successfully */
	uint8_t driveInitSuccessFlag;
	uint16_t getDriveInitSuccessFlag();

	// encoder position at index point
	int32_t indexPointEncPos;

	// when using the new fastUpdateCycle() mode, wheel phasing does not end at 0 encoder position. It shows this position.
	// phasing is a test made automatically at start.
	int32_t phasingEndOffset;

	/* status flag to see if simplemotion watchdog is enabled or not */
	bool watchdogenabled;

	/* set this to 1 when totally new drive init is commanded. */
	uint8_t forceFirsttimeDriveInit;

	/* Drive status handling and other drive related status variables */

	/* Drive status bits */
	int32_t driveStatusBits;

	/* Automatic commutation status flags */
	AutoCommutationMode autoCommutationStatus;

	/* Last Motor Fault Location ID */
	int32_t faultLocationID;

	/* factory test mode */
	uint8_t testModeActive;
	uint32_t factory_encoder_offset;

	/* Biquad filters used by the firmware */
	Biquad1StOrder inertiaLPF;
	Biquad1StOrder dampingLPF;
	Biquad1StOrder dampingForceLPF;
	Biquad1StOrder frictionForceLPF;

	Biquad1StOrder desktopDampingLPF;

	/* NO IONI USB MODE */
	bool ServoDriveDisabled;

	/* counts successive SM setTorque() failure counts */
	uint8_t torqueFailCount;
	uint8_t torqueFailed;

	/* high torque mode setting */
	uint8_t highTorqueMode;
	uint16_t getHighTorqueMode();

	//
	uint8_t servoStandby;
	void handleWirelessEvents();



	/* sto output status */
	uint8_t STOStatus; // 1 = STO enabled, motor does not run
	uint16_t getSTOStatus();

	/* torque off due to wireless wheel connection activities */
	uint8_t wirelessTorqueOffEvent; // 1 = torque is off

	/* torque off due to wireless wheel torque off button */
	uint8_t wirelessTorqueOffButton; // 1 = torque is off

	/* torque off due to save to flash */
	uint8_t torqueOffSavingDelay; // 1=torque is off
	uint64_t torqueOffSavingDelayStart;

	/* this gets set to 1 when high torque mode should be disabled */
	uint8_t sendDisableHighTorque;

	/* a class to keep track of parameters sent to drive, to not have to init filters on drive for no reason */
	DriveParameterTracker driveParamTracker;

    uint16_t usb_steering_angle();

	bool haveEffects() const { return ffbEffectUsage != 0; }

	encoderAngle angle;
	EndstopEffect endstop_effect;
	Biquad1StOrder axisSpeedMsLPF;

	void setSWWIdleDisconnectStatus(uint8_t status);

private:
	void initLedCycleFormats();
	FfbEngine* ffbhandle;

	static uint32_t GetSector(uint32_t Address);

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

	ProfileNameReplyPacket namereply __attribute__((aligned(4)));
	settingsByteDataReplyPacket bytedatareply __attribute__((aligned(4)));
	statusReplyPacket statusreply __attribute__((aligned(4)));
	eventLogReplyPacket eventreply __attribute__((aligned(4)));
	settingsDataReplyPacket settingsdatareply __attribute__((aligned(4)));
	setSettingsDataPacket drivedatareply __attribute__((aligned(4)));
	wirelessDataTransferPacket blePacket __attribute__((aligned(4)));
	commandPacket replyPacket __attribute__((aligned(4)));

	uint8_t counter;

	uint8_t flashMajorVersion;
	uint8_t flashMinorVersion;
	uint8_t flashBuildVersion;
	bool fillFlashZeroes(uint32_t startAddress, uint32_t length);

	uint8_t _swwIdleDisonnect;

	bool clippingLedOn;

	bool lastWirelessConnectionState;
	uint16_t lastWirelessVoltage;

	void torqueCommand(int32_t filtered, int32_t direct);

	void calculate_effects_torque(float&, float&);

	bool reset;

	bool _smReleased;
	uint8_t _indexpointFound;

	int32_t lastTorqueSummed;
	SWWAutoDisconnectHandler _swwautodisconnecthandler;
};

#endif /* CFFBDEVICE_H_ */

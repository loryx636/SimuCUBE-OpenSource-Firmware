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

#ifndef __CONFIG_COMM_DEFINES_H__
#define __CONFIG_COMM_DEFINES_H__

#ifdef __cplusplus
#include <cstdint>
#else
#include <stdint.h>
#define nullptr (void*)0
#endif

// hardware settings addresses
#define addrEncoderCPR              	1
#define addrIndexingMode            	2
#define addrDesktopSpringGain       	3
#define addrDesktopDamperGain       	4
#define addrDesktopSpringSaturation     5

// Gain for endstop damping which is based on speed inside the endstop range.
#define addrHwFree_2          			6
#define addrHwFree_3                    7
#define addrHwFree_4                    8
#define addrButtonDebounceMs            9
#define addrMaxMotorCurrent             10
#define addrMotorTorqueConstant         11
#define addrMotorResistance             12
#define addrAutoCommutationStatus       13
#define addrVariousSettingsBits1        14

// bits in hardware bitfields
#define bitDesktopAutoCenterEnabled	0
//#define bitEndstopsEnabled			1 // always on/not checked
#define bitInitialConfigDone		2
#define bitInverSteeringTorque		3
#define bitAudibleNotificationsEna	4
#define bitUSBResetsEnabled			5
#define bitUSBSuspendEnabled		6
#define bitAutoCommutationMode		7
#define bitAbsoluteEncoderDetected	8
#define bitInvertSteeringValue		9
//#define bitAutoConnectBLEDevice	10 // Not in use anymore
#define bitWirelessButtonsFirst		11


#define addrVariousSettingsBits2        15
#define addrEncoderOffset				16
#define addrCurrentAnalogConfig			17
#define addrX12buttonConfigs			18

//modes for button inputs; bits0-7 x12lower, bits 8-15 x12upper
#define input_buttons         0
#define input_encodersmode1   1
#define input_encodersmode2   2

#define addrFactoryEncoderOffset		19
#define addrSimucubeOptions				20
#define addrAutoConnectBLEDevice		21
#define addrHwFree_6					22
#define addrHwFree_7					23
#define addrHwFree_8					24
#define addrHwFree_9					25
#define addrHwFree_10					26
#define addrHwFree_11					27
#define addrHwFree_12					28
#define addrHwFree_13					29
#define addrHwFree_14					30
#define addrHwFree_15					31
#define addrHwFree_16					32
#define addrHwFree_17					33
#define addrHwFree_18					34
#define addrHwFree_19					35
#define addrHwFree_20					36
#define addrHwFree_21					37
#define addrHwFree_22					38
#define addrHwFree_23					39
#define addrHwFree_24					40
#define addrHwFree_25					41
#define addrHwFree_26					42
#define addrHwFree_27					43
#define addrHwFree_28					44
#define addrHwFree_29					45
#define addrHwFree_30					46
#define addrHwFree_31					47
#define addrHwFree_32					48
#define addrHwFree_33					49
#define addrHwFree_34					50
#define addrHwFree_35					51
#define addrHwFree_36					52
#define addrHwFree_37					53
#define addrHwFree_38					54
#define addrHwFree_39					55
#define addrHwFree_40					56
#define addrHwFree_41					57
#define addrHwFree_42					58
#define addrHwFree_43					59
#define addrHwFree_44					60
#define addrHwFree_45					61
#define addrSCSerialNumber				62
#define addrHwFree_47					63
#define numberOfHwSettingsAddrs			64


#define addrAnalogPin 					1
#define addrAnalogEnabled				2
#define addrAnalogMin 					3
#define addrAnalogMax 					4
#define addrAnalogInvert 				5
#define addrAnalogFree1 				6
#define addrAnalogFree2 				7
#define addrAnalogFree3 				8
#define addrAnalogFree4 				9
#define addrAnalogFree5 				10
#define numberOfAnalogSettingsAddrs		11
#define analogDefaultInvert				0
#define analogDefaultMin 				100
#define analogDefaultMax 				65435


#define addrButtonFree_1	1
#define addrButtonFree_2	2
#define addrButtonFree_3	3
#define addrButtonFree_4	4
#define addrButtonFree_5	5
#define addrButtonFree_6	6
#define addrButtonFree_7	7
#define addrButtonFree_8	8
#define addrButtonFree_9	9
#define addrButtonFree_10	10
#define addrButtonFree_11	11
#define addrButtonFree_12	12
#define addrButtonFree_13	13
#define addrButtonFree_14	14
#define addrButtonFree_15	15
#define addrButtonFree_16	16
#define addrButtonFree_17	17
#define addrButtonFree_18	18
#define addrButtonFree_19	19
#define addrButtonFree_20	20
#define addrButtonFree_21	21
#define addrButtonFree_22	22
#define addrButtonFree_23	23
#define addrButtonFree_24	24
#define addrButtonFree_25	25
#define addrButtonFree_26	26
#define addrButtonFree_27	27
#define addrButtonFree_28	28
#define addrButtonFree_29	29
#define addrButtonFree_30	30
#define addrButtonFree_31	31
#define addrButtonFree_32	32
#define addrButtonFree_33	33
#define addrButtonFree_34	34
#define addrButtonFree_35	35
#define addrButtonFree_36	36
#define addrButtonFree_37	37
#define addrButtonFree_38	38
#define addrButtonFree_39	39
#define addrButtonFree_40	40
#define addrButtonFree_41	41
#define addrButtonFree_42	42
#define addrButtonFree_43	43
#define addrButtonFree_44	44
#define addrButtonFree_45	45
#define addrButtonFree_46	46
#define addrButtonFree_47	47
#define addrButtonFree_48	48
#define addrButtonFree_49	49
#define addrButtonFree_50	50
#define addrButtonFree_51	51
#define addrButtonFree_52	52
#define addrButtonFree_53	53
#define addrButtonFree_54	54
#define addrButtonFree_55	55
#define addrButtonFree_56	56
#define addrButtonFree_57	57
#define addrButtonFree_58	58
#define addrButtonFree_59	59
#define addrButtonFree_60	60
#define addrButtonFree_61	61
#define addrButtonFree_62	62
#define addrButtonFree_63	63
#define numberofbuttonsettings 64


#define addrSineGain                1
#define addrSquareGain              2
#define addrMainGain                3
#define addrSpringGain              4
#define addrFrictionGain            5
#define addrDamperGain              6
#define addrInertiaGain             7

/**
 * EndstopOffset is profile setting which is an offset for moving the endstop range around.
 * It allows players to work with iRacing when calibrating the wheel for a new car when they have the same
 * steering range as the car specific steering angle and do not want to fight against endstop effect during
 * calibration.
 *
 * Negative values move bumpstop effect range outside the steering range, positive values make the effect start
 * sooner. Invariant abs(addrEndstopOffset) < addrMaxAngle / 2 needs to hold to know the distance to either end.
 */
#define addrEndstopOffset           8
#define addrIoniLPF                 9
#define addrIoniNotch               10
#define addrIoniDamping             11
#define addrIoniFriction            12
#define addrIoniInertia             13
#define addrIoniFilter1             14
#define addrFilteringModes          15
#define addrSawtoothGain            16
#define addrTriangleGain            17
#define addrMaxAngle                18
#define addrStaticForceReduction    19
#define addrProfileFree1			20
#define addrProfileFree2			21
#define addrProfileFree3			22
#define addrProfileFree4		 	23
#define addrProfileFree5			24
#define addrProfileFree6       		25
#define addrProfileFree7			26
#define addrProfileFree8			27

#define addrProfileSettingsBits1 	28
//bits 0..7: PhysicalPaddlesToSWWPaddles: 0 = no, 1=yes, 2 and up: SC1 things

#define addrBumpstopSetting 		29
#define softBumbstop    0
#define mediumBumbstop  1
#define hardBumbstop    2
#define addrProfileSettingsBits1 28
//bits 0..7: PhysicalPaddlesToSWWPaddles: 0 = no, 1=yes, 2 and up: SC1 things
#define addrProfileFree12   30
#define addrProfileFree13   31
#define addrProfileFree14   32
#define addrProfileFree15   33
#define addrProfileFree16   34
#define addrProfileFree17   35
#define addrProfileFree18   36
#define addrProfileFree19   37
#define addrProfileFree20   38
#define addrProfileFree21   39
#define addrProfileFree22   40
#define addrProfileFree23   41
#define addrProfileFree24   42
#define addrProfileFree25   43
#define addrProfileFree26   44
#define addrProfileFree27   45
#define addrProfileFree28   46
#define addrProfileFree29   47
#define addrProfileFree30   48
#define addrProfileFree31   49

/* UI various 1-bit profile parameters - FW does not care about these */
#define addrReservedUIbits	50

#define numberOfProfAddrs	51


#ifndef WIN32
#include "cHardwareConfig.h"
#include "cProfileConfig.h"
#endif
#include "types.h"

/* Version number indexes */
#define majorVersionIdx 0
#define minorVersionIdx 1
#define buildVersionIdx 2

/* Define HW Version Number */
#define hwunknown   0
#define hwv1        1
#define hwv2        2


/* USB reportID defines */
#define outReport 		0x6B
#define inReport 		0x6C
#define irFFBOutReport 	0x4D //77

/* Analog axis config values */
#define unUsed      0
/* read the rest from main.h */
#ifndef WIN32
#include "main.h"
#endif

#define defaultAnalogDeadzone 100  // default analog deadzone needed to show 0% and 100% at some confidence at default values.


// drive update rate is used to init some filters, among other things.
#define DRIVEUPDATERATE 1250.0f


/* profile management */
#define maxnumprofiles_v8 	500
#define maxnumprofiles_v11 	100
/* analog axis profiles */
#define maxnumanalogconfigs 21



/* this is used to send 55 bytes
 * of profile data in single 64 byte
 * HID reports.
 */
#define profileDataByteSplitIndex 55



/* Wheel indexing modes */
#define noIndexing 0
#define startAtCenterPhasedIndexing 1
#define indexPointIndexing 2
#define autoCommutationIndexing 3


/* filtering modes
 * NOTE: add additional modes here and
 * add them to cFFBDevice::CalcTorqueCommand() when implementing.
 */
#define bitTestFilter 	0
#define bitMovingAvg 	1
#define bitLinearRamp 	2



/* USB command ID:s */
#define requestStatus 				0
#define replyStatus 				1

#define requestIoniConfig 			2
#define replyIoniConfig 			3

#define saveToFlash 				4
#define reloadFromFlash		 		5
#define freshStart 					6

#define firmwareUpdateDataPacket	7
#define flashNewFirmwareNow 		8

#define enableSMUSB 				9
#define disableSMUSB 				10

#define requestRawAnalogAxis		19
#define requestCalibratedAnalogAxis	20 // default operating mode

#define setWheelCenterHere			21
#define replyWheelCenter			22  // maybe unneeded?
#define loadFactoryCenterpoint		23


#define transferIoniDrcData			25
#define applyIoniDrcData			26

#define unsetSettingsChanged        27

#define startDriveInit				28
#define setInitialConfigDone		29
#define restartDrive                30
#define clearInitialConfigDone		31

#define setTemporaryCenterMode		32

#define connectDriveCommand			33

#define unsetTemporaryCenterMode    34
#define readIoniFiltersToProfile	35

#define setForcesDisabled           36
#define setForcesEnabled            37


#define activateProfile             38
#define setNumProfiles              39
#define setDefaultProfile			40

#define resetFFBvariables			41

#define startCommutationAutoSetup	42
#define clearCommutationAutoSetup   43

#define reEnableTorqueMode          44

#define startIRFFBMode   			77
#define stopIRFFBMode				78

#define requestEventLog				79
#define replyEventLog				80
#define stopEventLogging			81
#define startEventLogging			82
#define setEventLogVerbosity		83
#define jmpToBootloader			    99

#define setTemporaryVariable		100

#define gotoTestMode				107

#define requesthwdata				110
#define requestanalogdata			111
#define requestbuttonsdata			112
#define requestprofiledata			113
#define requestprofilename			114
#define requestprofilebytedata		115
#define requestanalogbytedata		116

#define datareplypacketid			120
#define bytedatareplypacketid 		121

#define sethwdata					122
#define setanalogdata				123
#define setbuttonsdata				124
#define setprofiledata				125
#define setprofilename				126
#define setprofilebytedata			127
#define setanalogbytedata			128


#define startScanning				129
#define stopScanning				130

#define getNumberOfFoundDevices		131
#define replyNumberofFoundDevices	132
#define getDeviceData				133
#define replyDeviceData				134
#define connectDevice				135
#define disconnectDevice			136
#define getCurrentDeviceData        137
#define disconnectForgetDevice      138

#define setDriveParams				150
#define startReadDriveParams		151
#define requestReadDriveParams		152
#define replyReadDriveParams		153

#define TelemetryRequestOrReply		154

#define nopCommand					200
#define simucubeTelemetryData		255


#define temporarySteeringAngle		1
#define unsetTemporarySteeringAngle 2


/* DirectInput Effect Bit Defines */
#define ConstantEffectBit 0
#define PeriodSineConstantEffectBit 1
#define PeriodSineChangingEffectBit 2
#define DampingEffectBit 3
#define InertiaEffectBit 4
#define FrictionEffectBit 5
#define RampEffectBit 6
#define SquareEffectBit 7
#define TriangleEffectBit 8
#define SawtoothDownEffectBit 9
#define SawtoothUpEffectBit 10
#define SpringEffectBit 11
#define CustomEffectBit 12

/* Automatic commutation statuses */
enum AutoCommutationMode {Started, Busy, FailedUnsupportedEncoder,
                          FailedNotInitialized, FailedUninitialized, FailedLowCurrent1,
                          FailedLowCurrent2, FailedNoisyBadHall,FailedInvalidHallSequence,
                          Timeout, Success, UnknownResult};

/* EventLog Size */
#define logSize 1000


#ifndef WIN32
/* Typedef for one analog axel */
#ifdef WIN32
#pragma pack(push, 1)
#endif
typedef struct
  {
	uint8_t pin;
	bool invert;
	uint16_t minvalue;
	uint16_t maxvalue;
  } __attribute__((packed)) AnalogAxel;
#ifdef WIN32
#pragma pack(pop)
#endif
#endif

/* Typedef for one analog axel */
#ifdef WIN32
#pragma pack(push, 1)
#endif
typedef struct
  {
	uint32_t settings[numberOfAnalogSettingsAddrs];
  } __attribute__((packed)) AnalogAxelNew;
#ifdef WIN32
#pragma pack(pop)
#endif



/* Report types from PC to SimuCUBE */


#ifdef WIN32
#pragma pack(push, 1)
#endif
typedef struct
	{ // this is used to send a simple command. Most commands are like this.
    uint8_t reportID;
	uint8_t command;
	uint16_t value;   // optional, not used in every packet
	uint16_t value2;  // optional, not used in every packet
	} __attribute__((packed)) commandPacket;
#ifdef WIN32
#pragma pack(pop)
#endif

#ifdef WIN32
#pragma pack(push, 1)
#endif
	typedef struct
		{ // this is used to set Ioni DRC Data to SimuCUBE before uploading it to SimuCUBE.
		uint8_t reportID = outReport;
		uint8_t command = transferIoniDrcData; // = setHardwareConfig or setProfileConfig or setAnalogConfig
        uint8_t firstpacket = 0;
        uint8_t lastByte = 55;
        uint8_t data[55];
		} __attribute__((packed)) transferIoniDrcDataPacket;
#ifdef WIN32
#pragma pack(pop)
#endif

#ifndef WIN32
#include "simplemotion.h"
#endif


#ifdef WIN32
#pragma pack(push, 1)
#endif
typedef struct
	{ // this is used to set force directly from IRFFB
	uint8_t reportID = irFFBOutReport;
	uint16_t IRForce[6]; // 0 = latest
	} __attribute__((packed)) setIRFFBForcePacket;
#ifdef WIN32
#pragma pack(pop)
#endif






/* Report types from SimuCUBE to PC */
#define unsavedSettingsBit          0
#define indexpointFoundBit          1
#define initSuccessBit              2
#define temporarySteeringAngleBit   3
#define forcesEnabledBit			4
#define bleModuleFoundBit           5
#define bleWheelConnectedBit		6
#define bleScanInProgressBit		7

#define wirelessTorqueOffEventBit	8
#define stoStatusBit				9
#define highTorqueModeBit			10
#define torqueOffSavingBit			11
#define smPermanentFaultBit			12
#define wirelessTorqueOffButtonBit	13
#define servoStandbyBit				14
#define bleAutoDisconnectedBit		15

#define endstopUnsafeBit            0


#ifdef WIN32
#pragma pack(push, 1)
#endif
typedef struct
	{ // status reply
	uint8_t reportID;					//1
	uint8_t command;					//2
	uint8_t majorVersion;				//3
	uint8_t minorVersion;				//4
	uint8_t buildVersion;				//5
	uint8_t pad0;						//6
	uint16_t driveStatus;				//8  //drivestatusbits full field
	SystemStatus SimuCubeStatus;		//12
	uint16_t simucubeStatusBits2;       //14
	uint16_t drcReceivedBytes;			//16
	uint16_t DriveFWVersion;			//18
	uint16_t activeProfileIndex;   	 	//20
	uint16_t defaultProfileIndex;   	//22
	uint16_t numberofprofiles;		    //24
	uint8_t hwVersion;					//25
	uint8_t DriveFWUploadPercentage;	//26
	uint16_t driveTypeID;				//28
	uint32_t debugvalue1;				//32
	uint32_t debugvalue2;   	        //36
	uint32_t ffbEffectsInUse;			//40
	uint32_t ffbEffectsInActiveUse; 	//44
	uint32_t motorfaults	;			//48  //motor faults register full field
	uint32_t motorFaultLocationID;		//52
	uint8_t logVerbosityMode;			//53
    uint16_t simucubeStatusBits;	    //54
	uint8_t pad2;					    //55
	uint8_t pad3;					    //56
	} __attribute__((packed)) statusReplyPacket;
#ifdef WIN32
#pragma pack(pop)
#endif

#ifdef WIN32
#pragma pack(push, 1)
#endif
typedef struct
	{ // status reply				//bytes
	uint8_t reportID;					//1 outReport
	uint8_t command;					//2 simucubeTelemetryData
	uint16_t gameID;					//4 can be anything, but if we implement more games requiring different FW implementation....
	uint16_t speed;						//6 in km/h*100, 100km/h = 10000
	uint16_t engRpm;					//8 in x/minute
	int16_t angles[3];					//14 in degrees*100, 100 = 1.0 degrees (todo: check what units iracing outputs)
	int16_t corner_loads[4];			//22 normalized to 1, scaled with 1000, 1000 = 1.0
	// rest of packet reserved for future things
	} __attribute__((packed)) simucubeTelemetryPacket;
#ifdef WIN32
#pragma pack(pop)
#endif


#ifdef WIN32
#pragma pack(push, 1)
#endif
typedef struct
   { // contains parsed wheel and button status
    uint16_t X;
    uint16_t Y;
    uint16_t Z;
    uint16_t Brake;
    uint16_t Throttle;
    uint16_t Clutch;
    uint16_t Rudder;
    uint16_t Hat;
    uint32_t Buttons[4];
} __attribute__((packed)) HIDReport;
#ifdef WIN32
#pragma pack(pop)
#endif

#ifdef WIN32
#pragma pack(push, 1)
#endif
typedef struct
    { // debugLogReply
    uint8_t reportID;
    uint8_t command;
    uint16_t latestEvent;
    uint16_t events[9];
    int32_t parameters[9];
    } __attribute__((packed)) eventLogReplyPacket;
#ifdef WIN32
#pragma pack(pop)
#endif



#ifdef WIN32
#pragma pack(push, 1)
#endif
typedef struct
    { //
    uint8_t reportID;
    uint8_t command;
    uint8_t value1;
    uint8_t value2;
    uint16_t addrs[9];
    } __attribute__((packed)) requestSettingsDataPacket;
#ifdef WIN32
#pragma pack(pop)
#endif


#ifdef WIN32
#pragma pack(push, 1)
#endif
typedef struct
    { // debugLogReply
    uint8_t reportID;
    uint8_t command;
    uint8_t value1;
    uint8_t value2;
    uint16_t addrs[9];
    int32_t values[9];
    } __attribute__((packed)) setSettingsDataPacket;
#ifdef WIN32
#pragma pack(pop)
#endif

#ifdef WIN32
#pragma pack(push, 1)
#endif
typedef struct
    { // debugLogReply
    uint8_t reportID;
    uint8_t command;
    uint16_t addrs[9];
    int32_t values[9];
    } __attribute__((packed)) settingsDataReplyPacket;
#ifdef WIN32
#pragma pack(pop)
#endif

#ifdef WIN32
#pragma pack(push, 1)
#endif
typedef struct
    { // debugLogReply
    uint8_t reportID;
    uint8_t command;
    uint16_t value1;
    uint16_t value2;
    uint8_t bytes[32];
    } __attribute__((packed)) settingsByteDataReplyPacket;
#ifdef WIN32
#pragma pack(pop)
#endif

#ifdef WIN32
#pragma pack(push, 1)
#endif
typedef struct
    {
    uint8_t reportID;
    uint8_t command;
    uint16_t value1;
    uint16_t value2;
    uint8_t mac[8];
    uint8_t devicename[32];
    int8_t signalstrength;
    uint8_t bond;
    uint8_t namelength;
    uint8_t batterylevel;
    uint8_t connectionquality;
    int8_t batteryMeasurementStatus;
    } __attribute__((packed)) wirelessDataTransferPacket;
#ifdef WIN32
#pragma pack(pop)
#endif

#ifdef WIN32
#pragma pack(push, 1)
#endif
typedef struct
    { // debugLogReply
    uint8_t reportID;
    uint8_t command;
    uint16_t value1;
    uint16_t value2;
    uint8_t bytes[32];
    } __attribute__((packed)) ProfileNameReplyPacket;
#ifdef WIN32
#pragma pack(pop)
#endif


/* Event Locations for debug log events */

#define eventTimeStamp					1
#define WatchDogDisableError    		10
#define WatchDogDisableSuccess  		11
#define WatchDogEnableError     		20
#define WathcDogEnableSuccess  	 		21

#define baudRateInitFailure				30
#define baudrateInitSuccess				31
#define baudrateDefaultInitFailure	 	32
#define baudrateDefaultInitSuccess 		33

#define firstTimeDriveInit				40
#define DriveInitError1					41
#define DriveInitError2					42
#define DriveInitError3					43
#define initDriveCPR					44
#define DriveInitParamReadError			45
#define initDriveType					46

#define IoniFWUpdate					50
#define IoniFWUpdateError				51
#define IoniFWUpdateSuccess				52

#define dptParamErrorParam				53
#define dptParamErrorParamValue			54

#define waitFaultsClear					61
#define waitServoReady					62

#define readMMCFromIONI					70
#define readRESFromIONI					71
#define ApplyDriveDRCError				72
#define ApplyDriveDRCSuccess			73

#define DriveInit_initialPos1           90
#define DriveInit_initialPos2           91
#define DriveInit_initialPos3           92
#define DriveInit_initialPos4           93
#define DriveInit_initialPos5           94
#define DriveInit_initialPos6           95
#define DriveInit_initialPos7           96

#define SimuCUBEBoot					100
#define loadedFlash						101
#define flashDataInvalid				102
#define rewriteDefaultProfile			103

#define EventInitialConfigNotDone 		104
#define EventInitialConfigDone 			105
#define EventConvertFlashV8V11          106

#define ReadSc2SerialNr					107

#define flashEmpty						110
#define flashTooOldMajor				111
#define flashTooOldMinor				112
#define flashTooNewMajor				113
#define flashTooNewMinor				114
#define flashWriteFailure				115
#define flashRewritePre01200			116
#define flashRewritePre10000			117

#define USB_ResetStack					130
#define USB_Suspend						131
#define USB_Resume						132


#define StateSystemNotConfigured		150
#define StateDriveInitSuccess			151
#define StateWaitingIndex				152
#define StateDriveInit					153
#define StateBeforeOperational			154
#define StateOperational				155
#define StateSavetoFlash				156
#define StateFreshStart					157
#define StateFlashFault					158
#define StateReleaseSMBus				159
#define StateSMBusReleased				160
#define StateRegainSMBus				161
#define StateRegainSMFailed				162
#define StateDriveConnectonError		163
#define StateWaitClearFaults			164
#define StateUploadDriveFW				165
#define StateUploadDriveFWError			166
#define StateApplyDRCData				167
#define StateDriveWaitReady				168
#define StateJumpBootloader				169
#define StateError						170
#define StateRegainSMFailedTimeOut		171
#define StateDriveInitSuccessPause		172
#define StateAutoSetupCommucation		173
#define StateTestMode					174
#define StateSetBaudrate				266

#define BLEChangeState					175
#define BLEChangeTask					176
#define BLEScanResponse					177
#define BLEConnectionParameters			178
#define BLEProcedureCompleted			179
#define BLELowRSSI						180
#define BLEUptimeCounter				181
#define BLEBatteryVoltage				182
#define BLEConfigurationData			183
#define BLEConfigurationIncorrect		184
#define BLETaskTimeout					185

#define BLEInactivityDisconnect			188
#define BLENoResponse					189

#define AutoCommutationUnstarted  		190
#define AutoCommutationFault      		191
#define AutoCommutationSuccess   		192
#define AutoCommutationBusy      	   	193

#define Command_eventLogRequest			200
#define Command_startLogging			201
#define Command_stopLogging				202
#define Command_startIRFFBmode			203
#define Command_stopIRFFBmode			204
#define Command_reloadFlash				205
#define Command_unsetSettingChanged		206
#define Command_requestProfile			207
#define Command_requestHardware			208
#define Command_requestAnalog			209
#define Command_requestIoniData			210
#define Command_setHardware				211
#define Command_setProfile				212
#define Command_setAnalog				213
#define Command_setIoniConfig			214
#define Command_tranferIoniDRCData		215
#define Command_ApplyIoniConfig			216
#define Command_ApplyIoniConfigFail		217
#define Command_goFreshStart			218
#define Command_enableSMUSB				219
#define Command_disableSMUSB			220
#define Command_setRawAnalogMode		221
#define Command_setCalibAnalogMode		222
#define Command_startDriveInit			223
#define Command_restartDrive			224
#define Command_setInitialConfig		225
#define Command_unsetInitialConfig		226
#define Command_setWheelCenter			227
#define Command_setTempCenter			228
#define Command_unsetTempCenter			229
#define Command_actProfile				230
#define Command_setNumProfiles			231
#define Command_setDefProfile			232
#define Command_readIoniToProf			233
#define Command_jmpToBootLoader			234
#define Command_disableForces			235
#define Command_enableForces			236
#define Command_resetFFBStates			237
#define Command_setLogVerbosity			238
#define Command_SetProfilePage			239
#define Command_requestProfileName 		240
#define Command_setupCommutation 		250
#define Command_clearCommutation		251
#define Command_setButtonsData			252
#define Command_setProfileName			253
#define Command_setProfileBytedata		254
#define Command_requestButtonsData		255
#define Command_requestProfileByteData	256
#define Command_setAnalogByteData   	257
#define Command_requestAnalogByteData 	258
#define Command_gotoTestMode			259
#define Command_startReadDriveParams	260
#define Command_readDriveParams			261
#define Command_writeDriveParams		262
#define Command_loadFactoryCenterpoint  263
#define Command_enableHighTorque		264
#define Command_disableHighTorque		265
#define Command_reEnableTorque          266

#define FFB_FindFreeEffect				300
#define FFB_FoundFreeEffectSlot			301
#define FFB_StopAllEffects				302
#define FFB_StartEffect					303
#define FFB_StopEffect					304
#define FFB_FreeEffect					305
#define FFB_FreeAllEffects				306
#define FFB_CreatedNewEffect			307
#define FFB_SetGain						308

#define FFB_InvalidEffect				349
#define FFB_MemFull						348



#define logVerbosityNormal 	0
#define logVerbosityFFBEvents	1
#define logVerbosityFFBEffects 2

/* Enum for Low battery state warning based on voltage difference between different connection intervals */
enum LowBatteryWarning {NotReady, NoWarning, Warning};

enum ConnectAutomaticallyToWirelessWheel {ToPairedDevice, Never};

#endif


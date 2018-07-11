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
 *
 *
 *
 * ---------------------------------------------------------------------------
*/

#ifndef __CONFIG_COMM_DEFINES_H__
#define __CONFIG_COMM_DEFINES_H__

#ifndef WIN32
#include "cHardwareConfig.h"
#include "cProfileConfig.h"
#endif
#include "types.h"

#ifdef WIN32
///////////////////////////////////////////////////////////////////////////////////////
//TYPES & VALUES //////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
//declare SM lib types
typedef long smbus;
typedef unsigned long smuint32;
typedef unsigned short smuint16;
typedef unsigned char smuint8;
typedef long smint32;
typedef short smint16;
typedef char smint8;
typedef char smbool;
#define smtrue 1
#define smfalse 0
typedef int SM_STATUS;
typedef smuint8 smaddr;
#endif


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

#define X11_upper_1 1  //adc1_chan9 (brake)
#define X11_upper_2 2  //adc1_chan8 (gas)
#define X11_upper_3 3  //adc1_chan15
#define X11_upper_5 4  //adc1_chan14 (clutch)
#define X11_upper_6 5  //adc1_chan7

#define X11_lower_2 6  //adc1_chan13
#define X11_lower_3 7  //adc1_chan3
#define X11_lower_5 8  //adc1_chan4
#define X11_lower_6 9  //adc1_chan5
#define X11_lower_7 10  //adc1_chan6

#define ex_pot_1    11  //adc1_chan12
#define ex_pot_2    12  //adc1_chan11
#define ex_pot_3    13  //adc1_chan10

#define defaultAnalogDeadzone 100  // default analog deadzone needed to show 0% and 100% at some confidence at default values.



/* profile management */
#define maxnumprofiles 500
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

/* Initial config defines */
#define InitialConfigNotDone 0
#define InitialConfigDone 107


/* filtering modes
 * NOTE: add additional modes here and
 * add them to cFFBDevice::CalcTorqueCommand() when implementing.
 */
#define testFilter 0
#define movingAvg 1
#define linearRamp 2



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

#define requestHardwareConfig 		11
#define replyHardwareConfig 		12

#define requestProfileConfig 		13
#define replyProfileConfig 			14

#define setHardwareConfig			15
#define setProfileConfig			16
#define setAnalogConfig				17
#define setIoniConfig				18

#define requestRawAnalogAxis		19
#define requestCalibratedAnalogAxis	20 // default operating mode

#define setWheelCenterHere			21
#define replyWheelCenter			22  // maybe unneeded?

#define requestAnalogConfig			23
#define replyAnalogConfig			24

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

#define startIRFFBMode   			77
#define stopIRFFBMode				78

#define requestEventLog				79
#define replyEventLog				80
#define stopEventLogging			81
#define startEventLogging			82
#define setEventLogVerbosity		83
#define jmpToBootloader			    99

#define setTemporaryVariable		100

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

#define defaultInvert false;
#define defaultMin 100;
#define defaultMax 65435;



/* Report types from PC to SimuCUBE */


#ifdef WIN32
#pragma pack(push, 1)
#endif
typedef struct
	{ // this is used to send a simple command. Most commands are like this.
	uint8_t reportID = outReport;
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
	{ // this is used to request specific Ioni config values
	uint8_t reportID = outReport;
	uint8_t command = requestIoniConfig;
	uint8_t numberOfValues;
	smint16 parameters[10];
	} __attribute__((packed)) ioniConfigRequestPacket;
#ifdef WIN32
#pragma pack(pop)
#endif



#ifdef WIN32
#pragma pack(push, 1)
#endif
typedef struct
	{ // this is used to set config parameters
	uint8_t reportID = outReport;
	uint8_t command; // = setHardwareConfig or setProfileConfig or setAnalogConfig
    uint16_t opt_number=0; // profile number
    uint8_t opt_page=0; // profile data page (55 bytes max/page)
    uint8_t data[55];
	} __attribute__((packed)) setConfigPacket;
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


#ifdef WIN32
#pragma pack(push, 1)
#endif
typedef struct
	{ // this is used to set Ioni parameters
	uint8_t reportID = outReport;
	uint8_t command = setIoniConfig;
	uint8_t numberOfValues;
	int16_t parameters[10];
	int32_t values[10];
	} __attribute__((packed)) setIoniConfigPacket;
#ifdef WIN32
#pragma pack(pop)
#endif


/* defines for bits that are used in variousSettingsBits */
#define USBSuspendBit 0
#define USBResetBit   1

#ifdef WIN32
#pragma pack(push, 1)
#endif
typedef struct
	{ // this is used to hold all hardware configuration parameters.
#if 0
	smint32 mEncoderOffset;
	smint32 mEncoderCPR;
    u8 mIndexingMode;
    s8 mDesktopSpringGain;
	s8 mDesktopDamperGain;
	b8 mDesktopAutoCenter;
	s8 mStopsSpringGain;
	s8 mStopsFrictionGain;
	u8 mInitialConfigDone;
	u8 invertSteeringValue;
	u8 audibleNotificationsEnabled;
#endif
	smint32 mEncoderOffset;				//4
	smint32 mEncoderCPR;				//8
    u8 mIndexingMode;					//9
    u8 mDesktopSpringGain;				//10
	u8 mDesktopDamperGain;				//11
	b8 mDesktopAutoCenter;				//12

	u8 mStopsEnabled;					//13
	u8 mStopsDamperGain;				//14
	u8 mStopsRangeDegrees;				//15
	u8 mStopsMaxForce;					//16

	u8 mInitialConfigDone;				//17
	u8 invertSteeringValue;				//18
	u8 audibleNotificationsEnabled;		//19
	u8 buttonDebounceMillis;			//20
    u16 maxMotorCurrent;				//22
    u16 motorTorqueConstant;			//24
    u16 motorResistance;				//26
    u8 mDesktopSpringSaturation;		//27
    u8 usbResetEnabled;					//28
    s32 autoCommutationStatus;			//32
    u8 autoCommutationMode;				//33
    u8 mAbsoluteEncoder;				//34
    u32 variousSettingsBits;			//38
    u8 pad[19];							//57
	} __attribute__((packed)) hwData;
#ifdef WIN32
#pragma pack(pop)
#endif

#ifdef WIN32
#pragma pack(push, 1)
#endif
typedef struct
    { // this is used to hold all profile configuration parameters.
    u16 mMaxAngle;                      //2
    u8 mSineGain;                       //3
    u8 mSquareGain;                     //4
    // in percentages!
    s8 mMainGain;                       //5
    u8 mSpringGain;                     //6
    u8 mFrictionGain;                   //7
    u8 mDamperGain;                     //8
    u8 mInertiaGain;                    //9
    s16 endstopOffsetAngleDegrees;      //11
    u8 ioni_lpf;                        //12
    u32 ioni_notch;                     //16
    u16 ioni_damping;                   //18
    u16 ioni_friction;                  //20
    u16 ioni_inertia;                   //22
    u8 ioni_filter1;                    //23
    u16 filteringModes;                 //25
    unsigned char profilename[25];      //50
    u8 mSawtoothGain;					//51
    u8 mTriangleGain;					//52
    u8 pad[3];							//55
    } __attribute__((packed)) profData;
#ifdef WIN32
#pragma pack(pop)
#endif


#ifdef WIN32
#pragma pack(push, 1)
#endif
typedef struct
	{ // this is used to hold all analog configuration parameters.
	//AnalogAxel X;
	AnalogAxel Y;
	AnalogAxel Z;
	AnalogAxel Brake;
	AnalogAxel Throttle;
	AnalogAxel Clutch;
	AnalogAxel Rudder;
	AnalogAxel T;
	} __attribute__((packed)) analogData;
#ifdef WIN32
#pragma pack(pop)
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
#define unsavedSettingsBit	0
#define indexpointFoundBit 	1
#define initSuccessBit		2
#ifdef WIN32
#pragma pack(push, 1)
#endif
typedef struct
	{ // status reply
	uint8_t reportID = inReport;		//1
	uint8_t command = replyStatus;		//2
	uint8_t majorVersion;				//3
	uint8_t minorVersion;				//4
	uint8_t buildVersion;				//5
	SystemStatus SimuCubeStatus;		//9
	uint8_t previousCommandSuccess;		//10
	uint16_t driveStatus=0;				//12  //drivestatusbits full field
	uint8_t simucubeStatusBits=0;		//13
	uint16_t drcReceivedBytes=0;		//15
	uint8_t DriveFWUploadPercentage=0;	//16
	uint16_t DriveFWVersion = 0;		//18
	uint16_t activeProfileIndex = 0;    //20
	uint16_t defaultProfileIndex = 0;   //22
	uint16_t numberofprofiles = 1;      //24
	uint8_t hwVersion;					//25
	uint32_t debugvalue1 = 0;			//29
	uint32_t debugvalue2 = 0;           //33
	uint32_t ffbEffectsInUse = 0;		//37
	uint32_t ffbEffectsInActiveUse = 0; //41
	uint32_t motorfaults = 0;			//45  //motor faults register full field
	uint16_t lasteventindex = 0;		//47
	uint8_t logVerbosityMode=0;			//48
	uint16_t driveTypeID=0;				//50
	uint32_t motorFaultLocationID=0;	//54
	uint8_t padding[2];				    //56
	} __attribute__((packed)) statusReplyPacket;
#ifdef WIN32
#pragma pack(pop)
#endif


#ifdef WIN32
#pragma pack(push, 1)
#endif
typedef struct
	{ // hardware config data reply
	uint8_t reportID = inReport;
	uint8_t command = replyHardwareConfig;
	uint8_t data[sizeof(hwData)];
	} __attribute__((packed)) hardwareConfigReplyPacket;
#ifdef WIN32
#pragma pack(pop)
#endif


#ifdef WIN32
#pragma pack(push, 1)
#endif
typedef struct
	{ // analog config data reply
	uint8_t reportID = inReport;
	uint8_t command = replyAnalogConfig;
	uint8_t data[sizeof(analogData)];
	} __attribute__((packed)) analogConfigReplyPacket;
#ifdef WIN32
#pragma pack(pop)
#endif


#ifdef WIN32
#pragma pack(push, 1)
#endif
typedef struct
	{ // profile config data reply
	uint8_t reportID = inReport;
	uint8_t command = replyProfileConfig;
	int8_t data[sizeof(profData)];
	} __attribute__((packed)) profileConfigReplyPacket;
#ifdef WIN32
#pragma pack(pop)
#endif


#ifdef WIN32
#pragma pack(push, 1)
#endif
typedef struct
	{ // ioni config data reply
	uint8_t reportID = inReport;
	uint8_t command = replyIoniConfig;
	uint8_t numberOfValues;
	int16_t parameters[10];
	int32_t values[10];
	} __attribute__((packed)) ioniConfigReplyPacket;
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
    uint32_t Buttons;
} __attribute__((packed)) HIDReport;
#ifdef WIN32
#pragma pack(pop)
#endif

#ifdef WIN32
#pragma pack(push, 1)
#endif
typedef struct
    { // debugLogReply
    uint8_t reportID = inReport;
    uint8_t command = replyEventLog;
    uint16_t latestEvent = 0;
    uint16_t events[9];
    int32_t parameters[9];
    } __attribute__((packed)) eventLogReplyPacket;
#ifdef WIN32
#pragma pack(pop)
#endif



/* Event Locations for debug log events */
#define WatchDogDisableError    	10
#define WatchDogDisableSuccess  	11
#define WatchDogEnableError     	20
#define WathcDogEnableSuccess   	21

#define baudRateInitFailure			30
#define baudrateInitSuccess			31
#define baudrateDefaultInitFailure 	32
#define baudrateDefaultInitSuccess 	33

#define firstTimeDriveInit			40
#define DriveInitError1				41
#define DriveInitError2				42
#define DriveInitError3				43
#define initDriveCPR				44
#define DriveInitParamReadError		45
#define initDriveType				46

#define IoniFWUpdate				50
#define IoniFWUpdateError			51
#define IoniFWUpdateSuccess			52

#define waitFaultsClear				61
#define waitServoReady				62

#define readMMCFromIONI				70
#define readRESFromIONI				71
#define ApplyDriveDRCError			72
#define ApplyDriveDRCSuccess		73


#define SimuCUBEBoot				100
#define loadedFlash					101
#define flashDataOld				102
#define rewriteDefaultProfile		103

#define EventInitialConfigNotDone 	104
#define EventInitialConfigDone 		105

#define flashEmpty					110
#define flashTooOldMajor			111
#define flashTooOldMinor			112
#define flashTooNewMajor			113
#define flashTooNewMinor			114
#define flashWriteFailure			115

#define USB_ResetStack				130
#define USB_Suspend					131
#define USB_Resume					132


#define StateSystemNotConfigured	150
#define StateDriveInitSuccess		151
#define StateWaitingIndex			152
#define StateDriveInit				153
#define StateBeforeOperational		154
#define StateOperational			155
#define StateSavetoFlash			156
#define StateFreshStart				157
#define StateFlashFault				158
#define StateReleaseSMBus			159
#define StateSMBusReleased			160
#define StateRegainSMBus			161
#define StateRegainSMFailed			162
#define StateDriveConnectonError	163
#define StateWaitClearFaults		164
#define StateUploadDriveFW			165
#define StateUploadDriveFWError		166
#define StateApplyDRCData			167
#define StateDriveWaitReady			168
#define StateJumpBootloader			169
#define StateError					170
#define StateRegainSMFailedTimeOut	171
#define StateDriveInitSuccessPause	172
#define StateAutoSetupCommucation	173

#define AutoCommutationUnstarted    190
#define AutoCommutationFault        191
#define AutoCommutationSuccess      192
#define AutoCommutationBusy         193

#define Command_eventLogRequest		200
#define Command_startLogging		201
#define Command_stopLogging			202
#define Command_startIRFFBmode		203
#define Command_stopIRFFBmode		204
#define Command_reloadFlash			205
#define Command_unsetSettingChanged	206
#define Command_requestProfile		207
#define Command_requestHardware		208
#define Command_requestAnalog		209
#define Command_requestIoniData		210
#define Command_setHardware			211
#define Command_setProfile			212
#define Command_setAnalog			213
#define Command_setIoniConfig		214
#define Command_tranferIoniDRCData	215
#define Command_ApplyIoniConfig		216
#define Command_ApplyIoniConfigFail	217
#define Command_goFreshStart		218
#define Command_enableSMUSB			219
#define Command_disableSMUSB		220
#define Command_setRawAnalogMode	221
#define Command_setCalibAnalogMode	222
#define Command_startDriveInit		223
#define Command_restartDrive		224
#define Command_setInitialConfig	225
#define Command_unsetInitialConfig	226
#define Command_setWheelCenter		227
#define Command_setTempCenter		228
#define Command_unsetTempCenter		229
#define Command_actProfile			230
#define Command_setNumProfiles		231
#define Command_setDefProfile		232
#define Command_readIoniToProf		233
#define Command_jmpToBootLoader		234
#define Command_disableForces		235
#define Command_enableForces		236
#define Command_resetFFBStates		237
#define Command_setLogVerbosity		238
#define Command_SetProfilePage		239
#define Command_requestProfilepage  240
#define Command_setupCommutation 	250
#define Command_clearCommutation	251

#define FFB_FindFreeEffect			300
#define FFB_FoundFreeEffectSlot		301
#define FFB_StopAllEffects			302
#define FFB_StartEffect				303
#define FFB_StopEffect				304
#define FFB_FreeEffect				305
#define FFB_FreeAllEffects			306
#define FFB_CreatedNewEffect		307
#define FFB_SetGain					308

#define FFB_InvalidEffect			349
#define FFB_MemFull					348




#if 0
#define Command_actProfile			438
#define Command_actProfile			439
#define Command_actProfile			440
#endif


#define logVerbosityNormal 	0
#define logVerbosityFFBEvents	1
#define logVerbosityFFBEffects 2

#endif


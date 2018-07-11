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


#ifndef TYPES_H
#define TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

///////////////////////////////////////////////////////////////////////////////////////
//TYPES ///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

enum SystemStatus {	BeforeInit, Driveconnect, DriveInit, DriveInitSuccess,
					WaitFindingIndexPoint, DriveConnectionError, DriveWaitClearfaults,
					DriveWaitReady, UploadingDriveFirmware, SystemNotConfigured,
					BeforeOperational, Operational, saveSettingsToFlash, goFreshStart,
					FlashFault, releaseSMBus, SMBusReleased,
					regainSMBus, ApplyIoniDrcData, IoniDrcError,
					UIDisconnected, IoniFWUpgradeError, JumpToBootLoader,
					DriveInitSuccessPause, AutoDetectCommutationSensors,
					Internal_ForceMyEnumIntSize = 2147483647L };


#ifndef u32
typedef unsigned long u32;
#endif
#ifndef u16
typedef unsigned short u16;
#endif
#ifndef u8
typedef unsigned char u8;
#endif

#ifndef s32
typedef signed long s32;
#endif
#ifndef s16
typedef signed short s16;
#endif
#ifndef s8
typedef signed char s8;
#endif
#ifndef b8
typedef u8 b8;
#endif
#ifndef b16
typedef u16 b16;
#endif
#ifndef b32
typedef u32 b32;
#endif
#ifndef f32
typedef float				f32;
#endif
#ifndef f64
typedef double				f64;
#endif
#ifndef s64
typedef signed long long	s64;
#endif
#ifndef u64
typedef unsigned long long	u64;
#endif
#ifndef FALSE
#define FALSE	false
#endif
#ifndef TRUE
#define TRUE	true
#endif

#ifndef NULL
#define NULL 0
#endif
#ifndef WIN32
	/*Defines for Bset etc directives....*/
#define Bset(data,val) data|=(val)
#define Bclr(data,val) data&=~(val)
#define Btest(data,val) ((data&(val))==(val))
#define Bchg(data,val) if (Btest(data,val)) Bclr(data,val); else Bset(data,val)
#define Bmov(data,data1,val) if (Btest(data,val)) Bset(data1,val); else Bclr(data1,val)

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define EULER 2.718281828459045235360287471352

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
//#define abs(x) ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x) ((x)*(x))

//for simple motion param ranges
#define S32MAX 2147483647L
#define S32MIN -2147483648L
#define S30MAX 536870911L
#define S30MIN -536870912L
#define S22MAX 2097151L
#define S22MIN -2097152L
#define S16MAX 32767
#define S16MIN -32768
#define S14MAX 8191L
#define S14MIN -8192L

#define U32MAX 4294967295UL
#define U32MIN 0UL
#define U16MAX 65535U
#define U16MIN 0



//C++ like type castings. ie U32(variable) = (u32)variable
#define U8(v) ((u8)(v))
#define U16(v) ((u16)(v))
#define U32(v) ((u32)(v))
#define S8(v) ((s8)(v))
#define S16(v) ((s16)(v))
#define S32(v) ((s32)(v))


#endif

#ifdef __cplusplus
}
#endif

#endif

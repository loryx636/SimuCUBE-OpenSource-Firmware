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

#ifndef __SIMPLEMOTIONCOMMS_H
#define __SIMPLEMOTIONCOMMS_H

#include "simplemotion.h"
#include "types.h"

#include "stm32f4xx_hal.h"
void SMPortSetMaster(bool me);







#ifdef __cplusplus
extern "C" {
#endif

int SMPortWrite(const char *data, int len);
void MX_TIM3_Init(void);
void MX_USART3_UART_Init(void);

int SMPortReadByte( char *byte );
#ifdef __cplusplus
}
#endif



#define RXBUFSIZE 256
extern unsigned char rxBuffer[RXBUFSIZE];
extern int rxBufPos;




void broadcastSystemStatus(SystemStatus status);

#define smbustimeout 100


// enable/disable SM watchdog
bool disableSMWatchdog();
bool enableSMWatchdog();

// search index pulse. Encoder is at 0 on startup.
bool WaitForIndexPulse( int &indexPos );


// wait for watchdog to time out
void startWatchDogWaitingDelay();

//init SM bus baudrate. can call again after loss of communication to re-init. return -1 if fail, 0 if success
int initSMBusBaudrate();

//init SM bus baudrate. can call again after loss of communication to re-init. return -1 if fail, 0 if success
int initDefaultSMBusBaudrate(bool notest=false);


//Drive initialization
bool InitializeDrive(bool OnlyConnectAndUploadFWIfRequired = false);

//Transmit complete callbacks for UART
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

// update IONI firmware
uint8_t updateIoniFirmware();

// reset UART3 parameters, e.g. when resetting baudrates
void resetUSART3vars();

// read some important drive parameters into the right variables in the gFFBDevice
bool readDriveParams();

// resets known maximum MMC to drive
void resetMaxMMC();

// sets current MMC to drive
void setCurrentProfileMMC();

// save drive configuration to drive flash
void saveDriveCfg();

// empties commutation configuration variables in the drive
void clearCommutationConfig();

// polls auto-commutation setup process state. Returns state from drive.
smint32 updateAutoCommutationSetup();

// Initializes torque command, a "pre-init" of sorts. Currently not used.
void InitializeTorqueCommand ();

//called after drive is fully initialized to set 32 bit position from drive. needed because 16 bit incremental pos counter can roll over and it is absolute only after this is called
//Update: this is not required anymore.
void resetPositionCountAt(s32 newpos);

//Writes torque to drive, returns encoder counter value from drive. Also parses the drive status and reads error codes.
s32 SetTorque (s32 normalized_torque);


// reads all filter parameters from the drive
void readIoniParameters();

// writes all filter parameters to the drive
void writeChangedIoniParameters(bool forced);



#endif

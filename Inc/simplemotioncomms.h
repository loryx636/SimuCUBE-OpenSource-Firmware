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


enum fwUpdateStatus {FWUpdateFailed, FwNotUpdated, FWUpdated};

void broadcastSystemStatus(SystemStatus status);

#define smbustimeout 50 //in milliseconds


// enable/disable SM watchdog
bool disableSMWatchdog();
bool enableSMWatchdog();

// search index pulse. Encoder is at 0 on startup.
bool WaitForIndexPulse( int &indexPos );


// wait for watchdog to time out
void startWatchDogWaitingDelay();

//init SM bus baudrate. can call again after loss of communication to re-init. return -1 if fail, 0 if success.
//param HighBPSMode=true sets high pbs mode and enables watchdog, false sets default and disables watchdog
int initSMBusBaudrate( bool HighBPSMode );


//Drive initialization
// mega function
bool InitializeDrive(bool OnlyConnectAndUploadFWIfRequired = false);

// split to functional parts
void preDriveInit();
fwUpdateStatus tryDfuUploadDriveFw(bool OnlyConnectAndUploadFWIfRequired);
bool updateDriveFw(bool OnlyConnectAndUploadFWIfRequired, smint fwversion);
smint32 readDriveFWVersion();
bool clearDriveErrors();
void readInitialDriveParams(smint32 &driveStatus);
void switchLedsOff();
bool checkConnectionErrors();
void setBeepsAndOtherDriveParameters();
bool waitDriveFaultsClear(smint32 &driveStatus);
void readInitialPos(smint32 &driveStatus);
bool waitForServoReady(smint32 &driveStatus);

//Transmit complete callbacks for UART
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

// update Drive firmware
uint8_t updateDriveFirmware();

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

// restarts drive
void restartSMDrive();

//will set correct fast uddate cycle mode for SetTorque
void initFastUpdateCycleMode();

// empties commutation configuration variables in the drive
void clearCommutationConfig();

// polls auto-commutation setup process state. Returns state from drive.
smint32 updateAutoCommutationSetup();

struct TorqueResponse {
	// microseconds since last SetTorque command
	uint16_t microseconds_since_last;
	// timestamp when the encoder was last sampled, difference between two is 400 us
	uint8_t encoder_sampled_at;
	// 24-bit of the latest encoder position
	int32_t position;
};

// Writes torque to drive, returns encoder counter value from drive. Also parses the drive status and reads error codes.
// parameters full scale = signed 14 bit
// normalized_torque_for_filtering will go through reconstruction filter etc
// normalized_torque_direct will bypass drive filters
TorqueResponse SetTorque(int32_t normalized_torque_for_filtering, int32_t normalized_torque_direct);

// plays sound if hw supports it
void SMPlaySound(int soundNumber);

// applies config data from buffer
bool SMApplyDRCDataFromBuffer(uint8_t* buffer, uint16_t length);


#endif

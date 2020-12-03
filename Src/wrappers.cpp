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

/*
 * wrappers.cpp
 *
 * This file includes all the callbacks to SimuCUBE c++ functions/classes from the
 * STM HAL C code.
 */

#include "wrappers.h"
#include "USBGameController.h"
#include "usbd_core.h"
#include "eventLog.h"
#include "hw_functions.h"
extern USBD_HandleTypeDef hUsbDeviceFS;
extern eventLog simucubelog;

extern USBGameController joystick;

/* some callback wrappers from C USB HAL code. */
extern "C" {

uint8_t gamecontroller_callback_wrapper(uint8_t *report) {
	return joystick.EPINT_OUT_callback(report);
}

void FfbOnCreateNewEffect_wrapper() {
	joystick.FfbOnCreateNewEffect();
}

void send_mSetReportAnswer_wrapper() {
	USBD_HandleTypeDef *pdev  = &hUsbDeviceFS;
	USBD_CtlSendData(pdev, (uint8_t *)joystick.get_mSetReportAnswer(), sizeof(USB_FFBReport_PIDBlockLoad_Feature_Data_t));
}

void send_mGetReportAnswer_wrapper(uint8_t report_id) {
	joystick.set_mGetReportAnswer(report_id);
	USBD_HandleTypeDef *pdev  = &hUsbDeviceFS;
	USBD_CtlSendData(pdev, (uint8_t *)joystick.get_mGetReportAnswer(), sizeof(USB_FFBReport_PIDPool_Feature_Data_t));
}

void simucubelog_addevent_wrapper(uint16_t event, bool forced) {
	simucubelog.addEvent(event, forced);
}

void simucubelog_addeventParam_wrapper(uint16_t event, int32_t parameter, bool forced) {
	simucubelog.addEventParam(event, parameter, forced);
}

uint32_t getVariousSettingsBits() {
	return joystick.gFFBDevice.mConfig.hardwareConfig.hwSettings[addrVariousSettingsBits1];
}

void sendDisableHighTorque() {
	joystick.gFFBDevice.sendDisableHighTorque=1;
}

}
/* end callback wrappers from C USB HAL code. */



/* callback to update button states every 1ms from systick c code */
/* (currently not used) */
extern "C" {
void updateButtons_wrapper() {
	joystick.gFFBDevice.updateButtons();
}
}




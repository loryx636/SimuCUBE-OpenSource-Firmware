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


/* Copyright (c) 2010-2011 mbed.org, MIT License
 * Modified Mouse code for Joystick - WH 2012
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */


/* Contributors:
 * Tero Kontkanen
 * Etienne Saint-Paul
 */

#include <FfbEngine.h>
#include "stdint.h"
#include "types.h"
#include "USBGameController.h"
#include "usb_device.h"
#include "usbd_customhid.h"
//#include "USBDevice_Types.h"
#include "answer_filetypes.h"
#include "simplemotioncomms.h"
#include "eventLog.h"
#include <cstring>

extern USBD_HandleTypeDef hUsbDeviceFS;
extern uint8_t usbsuspended;
extern bool debugMode;
extern eventLog simucubelog;
extern uint64_t millis;

USBGameController::USBGameController()
{
	gFFBDevice.SetFFB(&FFB);
	USBReportsEnabled = true;
	Buttons[0]=0;
	Buttons[1]=0;
	Buttons[2]=0;
	Buttons[3]=0;

	for (size_t i = 0; i < sizeof(reports) / sizeof(reports[0]); ++i) {
		reports[i] = { 0 };
	}

	currentreport=&reports[1];
	oldreport=&reports[0];
	lastsend=millis;
}

USBGameController::~USBGameController() {
	// FIXME: why destructor if this one could be generated automatically?
}

bool USBGameController::update_unchanged_steering()
{
	return update((uint16_t)X);
}

bool USBGameController::update(uint16_t steering)
{
	// if blocked by ongoing transmission, return true right away.
	if(!USBReportsEnabled) {
		printf(",");
		return true;
	}

	// unsigned 16-bit for these, as joystick API needs it.
	// calculated internally with more accuracy when reading/scaling.
	uint16_t throttle = 0;
	uint16_t rudder = 0;
	uint16_t clutch = 0;
	uint16_t brake = 0;
	uint16_t y = 0;
	uint16_t z = 0;
	uint16_t t = 0;

	//read and scale values
	// FIXME: storage for these is signed but we above use unsigned????
	gFFBDevice.mConfig.analogConfig[gFFBDevice.currentanalogconfigindex].updateAnalogAxises(
		gFFBDevice.rawAnalogMode,
		y,
		z,
		brake,
		throttle,
		clutch,
		rudder,
		t);

	gFFBDevice.updateButtons();

	Throttle = throttle;
	Brake = brake;
	Clutch = clutch;
	Rudder = rudder;
	X = steering;
	Y = y;
	Z = z;
	T = t;

	return update();
}

bool USBGameController::update()
{
#define AddAxisValue(m_val)		{currentreport->data[i++] = m_val & 0xff; currentreport->data[i++] = ((m_val & 0xff00) >> 8);}

	static uint16_t busycounter = 0;

	int i=0;
	currentreport->data[i++] = REPORT_ID_JOYSTICK;
	AddAxisValue(X);
	AddAxisValue(Y);
	AddAxisValue(Z);
	AddAxisValue(Brake);
	AddAxisValue(Throttle);
	AddAxisValue(Clutch);
	AddAxisValue(Rudder);
	AddAxisValue(T);

	if(gFFBDevice.mConfig.hardwareConfig.hwSettings[addrVariousSettingsBits1]&(1<<bitWirelessButtonsFirst)) {
		// put wireless buttons first for Raceroom etc compatibility (sees only first 32 buttons)
		addBleButtonsToReport(i);
		addHwButtonsToReport(i);
	} else {
		addHwButtonsToReport(i);
		addBleButtonsToReport(i);
	}

	// FIXME: why do SC1 and SC2 send possibly different sized reports?
	// also, we could just use a constant for length and zero fill with memset
	// in the beginning of this method either way; no need to do that in fillRestOfReport
	fillRestOfReport(i);

	currentreport->length = i;

	auto shouldSend = isReportSendingNeeded();

	if(!shouldSend) {
		return USBD_OK; // FIXME: what is uint8_t in bool???
	}

	lastsend = millis;

	if(currentreport==&reports[0]) {
		currentreport=&reports[1];
		oldreport=&reports[0];
	} else {
		currentreport=&reports[0];
		oldreport=&reports[1];
	}

	uint8_t value = USBD_OK;

	if (usbsuspended == 0) {
		value = USBD_CUSTOM_HID_SendReport( &hUsbDeviceFS, (uint8_t*)&oldreport->data, oldreport->length );
	}

	if (value == USBD_OK) {
		busycounter = 0;
		return value;
	}

	if(value == USBD_BUSY) {
		busycounter++;

		if (busycounter < 1500) {
			// wait for more busy until attempting to reset
			return value;
		}

		busycounter=0;

		if(gFFBDevice.mConfig.hardwareConfig.hwSettings[addrVariousSettingsBits1]&(1<<bitUSBResetsEnabled)) {
			if (debugMode) {
				printf("debug:force-setmode -allowed\r\n");
			}

			auto hhid = (USBD_CUSTOM_HID_HandleTypeDef*)(hUsbDeviceFS.pClassData);

			if(hhid != nullptr) {
				simucubelog.addEventParam(USB_ResetStack, 2, true);
				hhid->state=CUSTOM_HID_IDLE;
			} else {
				simucubelog.addEventParam(USB_ResetStack, 3, true);
			}
		} else {
			if (debugMode) {
				printf("debug:force-setmode -disallowed\r\n");
			}
			simucubelog.addEventParam(USB_ResetStack, -1, true);
		}
	} else if (value == USBD_FAIL) {
		// FIXME: what then?
	}
	return value;
}

void USBGameController::addBleButtonsToReport(int &i) {
	uint32_t s = gFFBDevice.BLEConn.lastButtonState;
	gFFBDevice.inputDeviceList.process(s);

	uint64_t outputValue = gFFBDevice.inputDeviceList.getButtonStates();

	// wireless button inputs
	currentreport->data[i++] = (outputValue&0xff) >> 0;
	currentreport->data[i++] = (outputValue&0xff00) >> 8;
	currentreport->data[i++] = (outputValue&0xff0000) >> 16;
	currentreport->data[i++] = (outputValue&0xff000000) >> 24; // 64
	currentreport->data[i++] = (outputValue&0xff00000000) >> 32;
	currentreport->data[i++] = (outputValue&0xff0000000000) >> 40;
	currentreport->data[i++] = (outputValue&0xff000000000000) >> 48;
	currentreport->data[i++] = (outputValue&0xff00000000000000) >> 56; // 96
}

void USBGameController::FfbOnCreateNewEffect() {
	USBD_HandleTypeDef *pdev  = &hUsbDeviceFS;
	USBD_CUSTOM_HID_HandleTypeDef *hhid = (USBD_CUSTOM_HID_HandleTypeDef*)pdev->pClassData;
	FFB.FfbOnCreateNewEffect((USB_FFBReport_CreateNewEffect_Feature_Data_t *)hhid->Report_buf, &mSetReportAnswer);
}


uint8_t USBGameController::EPINT_OUT_callback(uint8_t* report) {
	if (report == nullptr) {
		return 0;
	}

	// all these types of reports always mean, that there is some sensible USB comms going on.
	// therefore set usb reports to enabled.
	// FIXME: there seems to be more than one variable related to USB suspending
	usbsuspended = 0;

	// first check if the report is something that needs to
	// be handled in the interrupt
	// add more cases if required!
	if ((report[0] != outReport) && (report[0] != irFFBOutReport)) {
		FFB.handleReceivedHIDReport(report);
	} else {
		// put report in the buffer, to be handled by the main task
		memcpy(receivedReports[receivedReportBufferHead].data, report, USBD_CUSTOMHID_OUTREPORT_BUF_SIZE);
		receivedReportBufferHead=(receivedReportBufferHead+1)&(RX_REPORT_BUFFER_COUNT-1);
	}

	return 1;
}

bool USBGameController::tryTakeReceivedReport(HID_REPORT* report) {
	if (report == nullptr || getPendingReceivedReportCount() == 0) {
		return false;
	}

	auto taken = takeReceivedReport();

	memcpy(report, &taken, sizeof(HID_REPORT));

	return true;
}

bool USBGameController::tryConsumeReceivedReport() {
	if (!USBReportsEnabled || getPendingReceivedReportCount() == 0) {
		// FIXME: when USBReportsEnabled is false, our reports (responses) are being throttled
		// so we just queue these old reports up?
		return true;
	}

	auto report = takeReceivedReport();

	if ((report.data[0]==outReport) && (report.data[1]==simucubeTelemetryData)) {
		memcpy(&gFFBDevice.lastSimucubeTelemetryPacket, &report.data[0], sizeof(simucubeTelemetryPacket));
		gFFBDevice.lastTelemetryReceived = millis&0xFFFF;
	} else if(report.data[0] == outReport) {
		return gFFBDevice.handleSimuCUBEAPIcommand(report.data);
	} else if (report.data[0] == irFFBOutReport) {
		return gFFBDevice.handleIRFFBcommand(report.data);
	} else {
		return FFB.handleReceivedHIDReport(report.data);
	}
	return true;
}

unsigned int USBGameController::getPendingReceivedReportCount() {
	return (receivedReportBufferHead - receivedReportBufferTail) & (RX_REPORT_BUFFER_COUNT-1);
}

HID_REPORT& USBGameController::takeReceivedReport() {
	unsigned int takeFrom = receivedReportBufferTail;
	receivedReportBufferTail = (receivedReportBufferTail + 1) & (RX_REPORT_BUFFER_COUNT-1);
	return receivedReports[takeFrom];
}

USB_FFBReport_PIDPool_Feature_Data_t* USBGameController::get_mGetReportAnswer() {
	return &mGetReportAnswer;
}

USB_FFBReport_PIDBlockLoad_Feature_Data_t* USBGameController::get_mSetReportAnswer() {
	return &mSetReportAnswer;
}

void USBGameController::set_mGetReportAnswer(uint8_t report_id) {
	mGetReportAnswer.reportId = report_id;
	mGetReportAnswer.ramPoolSize = 0xffff;
	mGetReportAnswer.maxSimultaneousEffects = MAX_EFFECTS;
	mGetReportAnswer.memoryManagement = 3;
}

bool USBGameController::isReportSendingNeeded() {
	// why only compare bytes [1..34]? data is 64 bytes
	// answer: the joystick report is 0...34, where the byte 0 is always the same (report id)
	// and the size of that report is so that the byte 34 is the last one.
	// true is that maximum possible HID report size is 64 bytes.

	auto ret = memcmp(currentreport->data + 1, oldreport->data + 1, 33);
	return ret != 0 || millis > lastsend + USB_REPORT_SEND_MIN_PERIOD_MS;
}


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

#include <ffbengine.h>
#include "stdint.h"
#include "types.h"
#include "USBGameController.h"
#include "usb_device.h"
#include "usbd_customhid.h"
//#include "USBDevice_Types.h"
#include "answer_filetypes.h"
#include "simplemotioncomms.h"
#include "eventlog.h"

extern USBD_HandleTypeDef hUsbDeviceFS;
extern uint8_t usbsuspended;
extern bool debugMode;
extern eventLog simucubelog;

// remember to define constructor and destructor for your classes!
USBGameController::USBGameController()
{
	counter=0;
	maxcounter=0;
	gFFBDevice.SetFFB(&FFB);
	USBReportsEnabled = true;
}
USBGameController::~USBGameController() {

}

//bool USBGameController::update(uint16_t brake, uint16_t clutch, uint16_t throttle, uint16_t rudder, uint16_t x, uint16_t y, uint32_t button, uint8_t hat)
bool USBGameController::update(uint16_t steering)
{
	// if blocked by ongoing transmission, return true right away.
	if(!USBReportsEnabled) {
		printf(",");
		return true;
	}

	counter++;
	if(counter>maxcounter) maxcounter=counter;
	// unsigned 16-bit for these, as joystick API needs it.
	// calculated internally with more accuracy when reading/scaling.
	uint16_t throttle = 0;
	uint16_t rudder = 0;
	uint16_t clutch = 0;
	uint16_t brake = 0;
	//int8_t hat = 0;
	uint16_t y = 0;
	uint16_t z = 0;
	uint16_t t = 0;
	uint32_t buttons = 0;

	//read and scale values
	gFFBDevice.mConfig.analogConfig.updateAnalogAxises(gFFBDevice.rawAnalogMode, y, z, brake, throttle, clutch, rudder, t);
	gFFBDevice.updateButtons();
	//gFFBDevice.updateButtons(&buttons);

	Throttle = throttle;
	Brake = brake;
	Clutch = clutch;
	Rudder = rudder;
	X = steering;
	Y = y;
	Z = z;
	T = t;
	Buttons = buttons;
	//Hat = hat;

	return(update());
}



#define AddAxisValue(m_val)		{report.data[i++] = m_val & 0xff; report.data[i++] = ((m_val & 0xff00) >> 8);}

bool USBGameController::update() 
{
	static uint16_t busycounter = 0;
	HID_REPORT report;

	int i=0;

	report.data[i++] = REPORT_ID_JOYSTICK;
	AddAxisValue(X);
	AddAxisValue(Y);
	AddAxisValue(Z);
	AddAxisValue(Brake);
	AddAxisValue(Throttle);
	AddAxisValue(Clutch);
	AddAxisValue(Rudder);
	AddAxisValue(T);

	//report.data[8] = ((Buttons & 0x0f) << 4) | (Hat & 0x0f);

	Buttons = 0x0;
	for(int i=0; i<14; i++) {
		if(gFFBDevice.button_debouncedState[i]==GPIO_PIN_RESET) {
			Buttons |= (0x01<<i);
		}
	}
	//gFFBDevice.button_debouncedState[i]

	report.data[i++] = ((Buttons & 0xff) >>0);
	report.data[i++] = ((Buttons & 0xff00) >>8);
	report.data[i++] = ((Buttons & 0xff0000) >>16);
	report.data[i++] = ((Buttons & 0xff000000) >>24);
	report.length = i;

	// interrupts off
//	counter++;
	//	__disable_irq();

	uint8_t value=USBD_OK;
	if(usbsuspended==0) {
		value = USBD_CUSTOM_HID_SendReport( &hUsbDeviceFS, (uint8_t*)&report.data, report.length );
	} else {
		//printf("s.");
	}
	if(value == USBD_BUSY) {
		busycounter++;
		//printf("b %d\r\n", busycounter);
		if(busycounter>1500) {
		#if 0
		if(busycounter>1500 && (gFFBDevice.mConfig.hardwareConfig.usbResetsEnabled == 1)) {


			if(debugMode) printf("debug:deinit+1s+reinit\r\n");
			simucubelog.addEventParam(USB_resetStack,1, true);
			//USBD_CUSTOM_HID_DeInit(&hUsbDeviceFS, 0);
			//hUsbDeviceFS.pClass->DeInit(&hUsbDeviceFS, 0);
			disableSMWatchdog();
			MX_USB_DEVICE_DeInit_own();
			HAL_Delay(1000);
			//USBD_CUSTOM_HID_Init(&UsbDeviceFS, 0);
			//hUsbDeviceFS.pClass->Init(&hUsbDeviceFS, 0);
			MX_USB_DEVICE_Init();
			busycounter=0;
			enableSMWatchdog();
			gFFBDevice.readEncoder32bit();
#endif

			busycounter=0;
			if(gFFBDevice.mConfig.hardwareConfig.variousSettingsBits&(1<<USBResetBit)) {
				if(debugMode) printf("debug:force-setmode -allowed\r\n");
				USBD_CUSTOM_HID_HandleTypeDef *hhid =0;
				hhid = (USBD_CUSTOM_HID_HandleTypeDef*)(hUsbDeviceFS.pClassData);
				if(hhid!=NULL) {
					simucubelog.addEventParam(USB_ResetStack,2, true);
					hhid->state=CUSTOM_HID_IDLE;
				} else {
					simucubelog.addEventParam(USB_ResetStack,3, true);
				}
			} else {
				if(debugMode) printf("debug:force-setmode -disallowed\r\n");
				simucubelog.addEventParam(USB_ResetStack, -1, true);
			}

		}
	} else {
		busycounter=0;
	}

	//bool value = true;
	// interrupts on
	//	__enable_irq();
	return value;
}

bool USBGameController::throttle(int16_t t) {
	Throttle = t;
	return update();
}

bool USBGameController::rudder(int16_t r) {
	Brake = r;
	return update();
}

bool USBGameController::move(int16_t x, int16_t y) {
	X = x;
	Y = y;
	return update();
}

bool USBGameController::button(uint32_t button) {
	Buttons = button;
	return update();
}

bool USBGameController::hat(uint8_t hat) {
	Hat = hat;
	return update();
}


void USBGameController::_init() 
{
	Throttle = -127;
	Brake = -127;
	Clutch=0;
	Rudder=0;
	X = 0;                       
	Y = 0;
	Buttons = 0x00;
	Hat = 0x00;
}

void USBGameController::FfbOnCreateNewEffect() {
	USBD_HandleTypeDef *pdev  = &hUsbDeviceFS;
	USBD_CUSTOM_HID_HandleTypeDef *hhid = (USBD_CUSTOM_HID_HandleTypeDef*)pdev->pClassData;
	//FFB.FfbOnCreateNewEffect();
	FFB.FfbOnCreateNewEffect((USB_FFBReport_CreateNewEffect_Feature_Data_t *)hhid->Report_buf, &mSetReportAnswer);
}


uint8_t USBGameController::EPINT_OUT_callback( uint8_t *report)
{
	/*
    all events go here.

	 */



	if(report)
	{
		// all these types of reports always mean, that there is some sensible USB comms going on.
		// therefore set usb reports to enabled.
		usbsuspended=0;
		// first check if the report is something that needs to
		// be handled in the interrupt
		// add more cases if required!
		if((report[0] != outReport) && (report[0]!= irFFBOutReport)) {
			HID_REPORT interruptReport;
			memcpy(interruptReport.data, report,USBD_CUSTOMHID_OUTREPORT_BUF_SIZE);
			FFB.handleReceivedHIDReport(interruptReport);
		} else {
			// put report in the buffer
			//USBD_CUSTOM_HID_HandleTypeDef *hhid = (USBD_CUSTOM_HID_HandleTypeDef*)pdev->pClassData;
			memcpy(receivedReports[receivedReportBufferHead].data, report, USBD_CUSTOMHID_OUTREPORT_BUF_SIZE); //hhid->Report_buf, USBD_CUSTOMHID_OUTREPORT_BUF_SIZE);
			receivedReportBufferHead=(receivedReportBufferHead+1)&(RX_REPORT_BUFFER_COUNT-1);
		}

		asm("nop");
		bool returnval=1;
		return returnval;
	} else {
		return 0;
	}
}

unsigned int USBGameController::getPendingReceivedReportCount()
{
	//calculate how many reports are pending to read
	int reportCount = (receivedReportBufferHead-receivedReportBufferTail)&(RX_REPORT_BUFFER_COUNT-1);
	//if(reportCount) printf("have %d pending reports\r\n", reportCount);
	//return (receivedReportBufferHead-receivedReportBufferTail)&(RX_REPORT_BUFFER_COUNT-1);
	return reportCount;
}

HID_REPORT USBGameController::getReceivedReport()
{
	int takeFrom=receivedReportBufferTail;
	receivedReportBufferTail=(receivedReportBufferTail+1)&(RX_REPORT_BUFFER_COUNT-1);
	return receivedReports[takeFrom];
}


USB_FFBReport_PIDPool_Feature_Data_t* USBGameController::get_mGetReportAnswer() {
	return &mGetReportAnswer;
}
USB_FFBReport_PIDBlockLoad_Feature_Data_t* USBGameController::get_mSetReportAnswer() {
	return &mSetReportAnswer;
}

void  USBGameController::set_mSetReportAnswerId(uint8_t reportid) {
	mSetReportAnswer.effectBlockIndex= reportid;
}

void USBGameController::set_mGetReportAnswer(uint8_t report_id) {
	mGetReportAnswer.reportId = report_id;
	mGetReportAnswer.ramPoolSize = 0xffff;//0x01FF;//ffff;
	mGetReportAnswer.maxSimultaneousEffects = MAX_EFFECTS;
	mGetReportAnswer.memoryManagement = 3;
}


bool USBGameController::handleReceivedHIDReport(HID_REPORT report) {
	if(report.data[0] == outReport) {
		return gFFBDevice.handleSimuCUBEAPIcommand(report.data);
	} else if (report.data[0] == irFFBOutReport) {
		return gFFBDevice.handleIRFFBcommand(report.data);
	}
	else {
		return FFB.handleReceivedHIDReport(report);
	}
}


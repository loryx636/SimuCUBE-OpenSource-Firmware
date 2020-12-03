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

/* USBJoystick.h */
/* USB device example: Joystick*/
/* Copyright (c) 2011 ARM Limited. All rights reserved. */
/* Modified Mouse code for Joystick - WH 2012 */

#ifndef USBGAMECONTROLLER_H
#define USBGAMECONTROLLER_H

//#include "USBHID.h"
#include "FfbEngine.h"
#include "USBHID_Types.h"
#include "answer_filetypes.h"
#include "cFFBDevice.h"
#include "../SimpleMotion/simplemotion.h"

#define REPORT_ID_JOYSTICK  4
#define USB_REPORT_SEND_MIN_PERIOD_MS 50

// bmRequestType
#define REQUEST_HOSTTODEVICE	0x00
#define REQUEST_DEVICETOHOST	0x80
#define REQUEST_DIRECTION		0x80

#define REQUEST_STANDARD		0x00
#define REQUEST_CLASS			0x20
#define REQUEST_VENDOR			0x40
#define REQUEST_TYPE			0x60

#define REQUEST_DEVICE			0x00
#define REQUEST_INTERFACE		0x01
#define REQUEST_ENDPOINT		0x02
#define REQUEST_OTHER			0x03
#define REQUEST_RECIPIENT		0x03

#define REQUEST_DEVICETOHOST_CLASS_INTERFACE  (REQUEST_DEVICETOHOST + REQUEST_CLASS + REQUEST_INTERFACE)
#define REQUEST_HOSTTODEVICE_CLASS_INTERFACE  (REQUEST_HOSTTODEVICE + REQUEST_CLASS + REQUEST_INTERFACE)

//	Class requests

#define CDC_SET_LINE_CODING			0x20
#define CDC_GET_LINE_CODING			0x21
#define CDC_SET_CONTROL_LINE_STATE	0x22

#define MSC_RESET					0xFF
#define MSC_GET_MAX_LUN				0xFE

#define HID_GET_REPORT				0x01
#define HID_GET_IDLE				0x02
#define HID_GET_PROTOCOL			0x03
#define HID_SET_REPORT				0x09
#define HID_SET_IDLE				0x0A
#define HID_SET_PROTOCOL			0x0B

#define NB_AXIS			8
#define NB_FF_AXIS		1

#define X_AXIS_NB_BITS	16
#define Y_AXIS_NB_BITS	16
#define Z_AXIS_NB_BITS	16
#define RX_AXIS_NB_BITS	16
#define RY_AXIS_NB_BITS	16
#define RZ_AXIS_NB_BITS	16
#define SX_AXIS_NB_BITS	16
#define SY_AXIS_NB_BITS	16
#define NB_BUTTONS		32

#define X_AXIS_LOG_MAX		((1L<<(X_AXIS_NB_BITS))-1)
#define X_AXIS_LOG_MIN		0//(-X_AXIS_LOG_MAX)
#define X_AXIS_PHYS_MAX		((1L<<X_AXIS_NB_BITS)-1)

#define Y_AXIS_LOG_MAX		((1L<<(Y_AXIS_NB_BITS))-1)
#define Y_AXIS_LOG_MIN		0//(-Y_AXIS_LOG_MAX)
#define Y_AXIS_PHYS_MAX		((1L<<Y_AXIS_NB_BITS)-1)

#define Z_AXIS_LOG_MAX		((1L<<(Z_AXIS_NB_BITS))-1)
#define Z_AXIS_LOG_MIN		0//(-Z_AXIS_LOG_MAX)
#define Z_AXIS_PHYS_MAX		((1L<<Z_AXIS_NB_BITS)-1)

#define RX_AXIS_LOG_MAX		((1L<<(RX_AXIS_NB_BITS))-1)
#define RX_AXIS_LOG_MIN		0//(-RX_AXIS_LOG_MAX)
#define RX_AXIS_PHYS_MAX	((1L<<RX_AXIS_NB_BITS)-1)

#define RY_AXIS_LOG_MAX		((1L<<(RY_AXIS_NB_BITS))-1)
#define RY_AXIS_LOG_MIN		0//(-RY_AXIS_LOG_MAX)
#define RY_AXIS_PHYS_MAX	((1L<<RY_AXIS_NB_BITS)-1)

#define RZ_AXIS_LOG_MAX		((1L<<(RZ_AXIS_NB_BITS))-1)
#define RZ_AXIS_LOG_MIN		0//(-RZ_AXIS_LOG_MAX)
#define RZ_AXIS_PHYS_MAX	((1L<<RZ_AXIS_NB_BITS)-1)

#define SX_AXIS_LOG_MAX		((1L<<(SX_AXIS_NB_BITS))-1)
#define SX_AXIS_LOG_MIN		0//(-SX_AXIS_LOG_MAX)
#define SX_AXIS_PHYS_MAX	((1L<<SX_AXIS_NB_BITS)-1)

#define SY_AXIS_LOG_MAX		((1L<<(SY_AXIS_NB_BITS))-1)
#define SY_AXIS_LOG_MIN		0//(-SY_AXIS_LOG_MAX)
#define SY_AXIS_PHYS_MAX	((1L<<SY_AXIS_NB_BITS)-1)

#define TRANSFER_PGM		0x80
#define TRANSFER_RELEASE	0x40
#define TRANSFER_ZERO		0x20

/* Common usage */
enum JOY_BUTTON {
     JOY_B0 = 1,
     JOY_B1 = 2,
     JOY_B2 = 4,
     JOY_B3 = 8,
};

#if(0)
enum JOY_HAT {
     JOY_HAT_UP      = 0,
     JOY_HAT_RIGHT   = 1,
     JOY_HAT_DOWN    = 2,
     JOY_HAT_LEFT    = 3,
     JOY_HAT_NEUTRAL = 4,
};
#else
enum JOY_HAT {
     JOY_HAT_UP         = 0,
     JOY_HAT_UP_RIGHT   = 1,
     JOY_HAT_RIGHT      = 2,
     JOY_HAT_RIGHT_DOWN = 3,
     JOY_HAT_DOWN       = 4,
     JOY_HAT_DOWN_LEFT  = 5,
     JOY_HAT_LEFT       = 6,
     JOY_HAT_LEFT_UP    = 7,
     JOY_HAT_NEUTRAL    = 8,
};
#endif

/* X, Y and T limits */
/* These values do not directly map to screen pixels */
/* Zero may be interpreted as meaning 'no movement' */
#define JX_MIN_ABS    (-127)     /*!< The maximum value that we can move to the left on the x-axis */
#define JY_MIN_ABS    (-127)     /*!< The maximum value that we can move up on the y-axis */
#define JT_MIN_ABS    (-127)     /*!< The minimum value for the throttle */
#define JX_MAX_ABS    (127)      /*!< The maximum value that we can move to the right on the x-axis */
#define JY_MAX_ABS    (127)      /*!< The maximum value that we can move down on the y-axis */
#define JT_MAX_ABS    (127)      /*!< The maximum value for the throttle */

#define RX_REPORT_BUFFER_COUNT 32//this must be power of 2

class USBGameController {
   public:
		USBGameController();
		~USBGameController();

		/**
		 * Report state to USB host.
		 * @returns true if ok, false if some internal error
		 */
		bool update(uint16_t steering);

		/**
		 * Reports the state of USB host when steering has not changed.
		 * @return true if ok, false if some internal error
		 */
		bool update_unchanged_steering();

		/*
		 * Called when a data is received on the USB OUT endpoint.
		 * @returns if handled 1, otherwise 0
		 */
		uint8_t EPINT_OUT_callback(uint8_t *report);

		void FfbOnCreateNewEffect();

		// Processes a single unprocessed (not necessarily latest) HID report when USBReportsEnabled
		bool tryConsumeReceivedReport();

		// Called by factorytester to consume a single received report
		bool tryTakeReceivedReport(HID_REPORT* report);

		/// ???
		USB_FFBReport_PIDPool_Feature_Data_t* get_mGetReportAnswer();

		/// ???
		USB_FFBReport_PIDBlockLoad_Feature_Data_t* get_mSetReportAnswer();

		// Initializes the report received with get_mGetReportAnswer with this report kind as the response
		// might have been changed by FfbEngine after changing effects.
		void set_mGetReportAnswer(uint8_t reportid);

		cFFBDevice gFFBDevice;

		// This is used to throttle report processing immediatedly after connecting the USB device for some reason.
		bool USBReportsEnabled;

		FfbEngine FFB;


	private:
		/**
		 * Send HID report to USB host. Report contains controller positions.
		 * @return true if ok, false if something did not work out
		 */
		bool update();

		// FIXME: used from hw_functions.cpp
		uint32_t Buttons[4];
		// FIXME: this method is implemented in hw_functions.cpp
		void addHwButtonsToReport(int &i);
		// FIXME: also in hw_functions.cpp
		void fillRestOfReport(int &i);

		// Shifts the Bluetooth button state into currentreport and increments i by the used bytes
		void addBleButtonsToReport(int &i);

		int16_t Throttle;
		int16_t Brake;
		int16_t Clutch;
		int16_t Rudder;
		int16_t X;
		int16_t Y;
		int16_t Z;
		int16_t T;
		uint8_t Hat;

		// @returns the number of unprocessed reports in the FIFO
        unsigned int getPendingReceivedReportCount();

		/**
		 * Note: this method will happily move the tail of the FIFO after the head, so only
		 * call this after making sure there is something in the FIFO with getPendingReceivedReportCount.
		 * @returns reference to the oldest unconsumed report
		 */
        HID_REPORT& takeReceivedReport();

		// Distributes given HID report to interested parties
		bool handleReceivedHIDReport(HID_REPORT& report);

		unsigned int receivedReportBufferHead; // the index for latest arrived report
		unsigned int receivedReportBufferTail; // the index for last unhandled report

		HID_REPORT receivedReports[RX_REPORT_BUFFER_COUNT];

		USB_FFBReport_PIDPool_Feature_Data_t mGetReportAnswer;
		USB_FFBReport_PIDBlockLoad_Feature_Data_t mSetReportAnswer;

		HID_REPORT reports[2] __attribute__((aligned(4)));
		HID_REPORT* currentreport;
		HID_REPORT* oldreport;

		// timestamp of when the last report sending started
		uint64_t lastsend;

		// @returns true if enough time has elapsed from last report or if the report has changed
		bool isReportSendingNeeded();

};
#endif

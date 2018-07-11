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

/* USBJoystick.h */
/* USB device example: Joystick*/
/* Copyright (c) 2011 ARM Limited. All rights reserved. */
/* Modified Mouse code for Joystick - WH 2012 */

#ifndef USBGAMECONTROLLER_H
#define USBGAMECONTROLLER_H

#ifdef __cplusplus
 extern "C" {
#endif

//#include "USBHID.h"
#include "FfbEngine.h"
#include "USBHID_Types.h"
#include "answer_filetypes.h"
#include "cffbdevice.h"

#define REPORT_ID_JOYSTICK  4

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

        /**
         *   Constructor
         *
         * @param vendor_id Your vendor_id (default: 0x1234)
         * @param product_id Your product_id (default: 0x0002)
         * @param product_release Your product_release (default: 0x0001)
         */
         USBGameController();//uint16_t vendor_id = 0x7c5a, uint16_t product_id = 0xb101, uint16_t product_release = 0x0001);
             /*USBHID(0, 0, vendor_id, product_id, product_release, false),receivedReportBufferHead(0),FFBEnabled(false),receivedReportBufferTail(0)
             {
                 _init();
                 connect();
             };*/

         ~USBGameController();

         /**
         * Write a state of the game controller
         *
         * @param 4 pedals first
         * @param x x-axis position
         * @param y y-axis position
         * @param buttons buttons state
         * @param hat hat state 0 (up), 1 (right, 2 (down), 3 (left) or 4 (neutral)
         * @returns true if there is no error, false otherwise
         */
         //bool update(uint16_t brake, uint16_t clutch, uint16_t throttle, uint16_t rudder, uint16_t x, uint16_t y, uint32_t button, uint8_t hat);
         bool update(uint16_t steering);

         /**
         * Write a state of the mouse
         *
         * @returns true if there is no error, false otherwise
         */
         bool update();

         /**
         * Move the throttle position
         *
         * @param t throttle position
         * @returns true if there is no error, false otherwise
         */
         bool throttle(int16_t t);

         /**
         * Move the rudder position
         *
         * @param r rudder position
         * @returns true if there is no error, false otherwise
         */
         bool rudder(int16_t r);

         /**
         * Move the cursor to (x, y)
         *
         * @param x-axis position
         * @param y-axis position
         * @returns true if there is no error, false otherwise
         */
         bool move(int16_t x, int16_t y);

         /**
         * Press one or several buttons
         *
         * @param button button state
         * @returns true if there is no error, false otherwise
         */
         bool button(uint32_t button);

         /**
         * Press hat
         *
         * @param hat hat state
         * @returns true if there is no error, false otherwise
         */
         bool hat(uint8_t hat);



         /*
         * Called when a data is received on the OUT endpoint.
         *
         * @returns if handle by subclass, return true
         */
         uint8_t EPINT_OUT_callback(uint8_t *report);





	
        void FfbOnCreateNewEffect();
	    bool handleReceivedHIDReport(HID_REPORT report);

        unsigned int getPendingReceivedReportCount();

        HID_REPORT getReceivedReport();

        void initSteeringAngles();



        USB_FFBReport_PIDPool_Feature_Data_t* get_mGetReportAnswer();
		USB_FFBReport_PIDBlockLoad_Feature_Data_t* get_mSetReportAnswer();
		void set_mSetReportAnswerId(uint8_t reportid);
		void set_mGetReportAnswer(uint8_t reportid);

		cFFBDevice gFFBDevice;

		bool USBReportsEnabled;
		FfbEngine FFB;
		uint32_t counter;
		uint32_t maxcounter;

     private:
         bool FFBEnabled;
         int16_t Throttle;
         int16_t Brake;
         int16_t Clutch;
         int16_t Rudder;
         int16_t X;
         int16_t Y;
         int16_t Z;
         int16_t T;
         uint32_t Buttons;
         uint8_t Hat;

         //storage for SET_REPORTs from host
         unsigned int receivedReportBufferHead, receivedReportBufferTail;//head is index where new arrived report is stored, tail is the index where is last unhandled report
		 HID_REPORT receivedReports[RX_REPORT_BUFFER_COUNT];

		 USB_FFBReport_PIDPool_Feature_Data_t mGetReportAnswer;
		 USB_FFBReport_PIDBlockLoad_Feature_Data_t mSetReportAnswer;




         void _init();
};

#ifdef __cplusplus
}
#endif

#endif

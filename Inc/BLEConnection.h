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


#ifndef BLECONNECTION_H_
#define BLECONNECTION_H_

#ifdef __cplusplus
 extern "C" {
#endif

 /* standard library headers */
 #include <stdint.h>
 #include <string.h>
 #include <stdio.h>
 #include <stdbool.h>
 #include <unistd.h>

 /* BG stack headers */
 #include "bg_types.h"
 #include "gecko_bglib.h"

 /* Own header */
#include "stm32f4xx_hal.h"

#include "config_comm_defines.h"

#define DEVICE_LIST_LENGTH 128
#define BLERXBUFSIZE 512

 // Persistent storage keys for button plate
#define BP_MAIN_VERSION_PS_KEY					0
#define BP_MINOR_VERSION_PS_KEY					1
#define BP_PROTOCOL_VERSION_MIN_PS_KEY			2
#define BP_PROTOCOL_VERSION_MAX_PS_KEY			3
#define BP_WHEEL_ANGLE_PS_KEY					4
#define SC_BLE_DEVICE_TYPE_PS_KEY				5


#define INPUT_DEVICE_0_PS_KEY 	10
#define INPUT_DEVICE_1_PS_KEY 	11

// Timer handles
#define READ_RSSI_TIMER_HANDLE 					0
#define RESET_TASK_TIMER_HANDLE					1
#define START_DISCOVERY_TIMER_HANDLE			2

// Connection configuration
#define CONN_INTERVAL_MIN 						6 // 6-3200, x * 1.25 ms
#define CONN_INTERVAL_MAX 						6 // 6-3200, x * 1.25 ms
#define CONN_FAST_LATENCY 						0 // How many connection intervals a slave can skip
#define CONN_SLOW_LATENCY 						5 // How many connection intervals a slave can skip
#define CONN_FAST_TIMEOUT 						50 // 0-500, x * 10 ms must be larger than (1+latency)*conn_interval_max*2
#define CONN_SLOW_TIMEOUT 						200 // 0-500, x * 10 ms must be larger than (1+latency)*conn_interval_max*2

// Pins 0 - 13: 2 paddles, 5 1:1 encoders and 2 buttons (device type, device number, extra data)
static const uint8_t DEFAULT_INPUT_DEVICE_CONFIGURATION_0[42] = {
	0x01, 0x00, 0x00,
	0x01, 0x01, 0x00,
	0x0a, 0x02, 0x00,
	0x00, 0x03, 0x00,
	0x0a, 0x04, 0x00,
	0x00, 0x05, 0x00,
	0x0a, 0x06, 0x00,
	0x00, 0x07, 0x00,
	0x0a, 0x08, 0x00,
	0x00, 0x09, 0x00,
	0x0a, 0x0a, 0x00,
	0x00, 0x0b, 0x00,
	0x02, 0x0c, 0x00,
	0x02, 0x0d, 0x00
};

// Pins 14-27: 14 buttons (device type, device number, extra data)
static const uint8_t DEFAULT_INPUT_DEVICE_CONFIGURATION_1[42] = {
	0x02, 0x0e, 0x00,
	0x02, 0x0f, 0x00,
	0x02, 0x10, 0x00,
	0x02, 0x11, 0x00,
	0x02, 0x12, 0x00,
	0x02, 0x13, 0x00,
	0x02, 0x14, 0x00,
	0x02, 0x15, 0x00,
	0x02, 0x16, 0x00,
	0x02, 0x17, 0x00,
	0x02, 0x18, 0x00,
	0x02, 0x19, 0x00,
	0x02, 0x1a, 0x00,
	0x02, 0x1b, 0x00
};

typedef struct BTDevice {
	bd_addr address;
	int8_t rssi;
	// 255 means unpaired, any other value means a valid "bonding" index
	uint8_t bonding;
	uint8_t nameLength;
	char name[10];
} BTDevice;

class BLEConnection {

private:

	// Device names must start with "SC2-"
    static const uint8 M_NAME_START_LENGTH = 4;
    const uint8 M_NAME_START[4] = { 0x53,0x43,0x32, 0x2d }; // "SC2-"

    //
	uint8_t conn_handle = 255;
	uint8_t bond_handle = 255;
	uint16_t loopCounter = 0;

	typedef enum {
		state_reset,
		state_disconnected,
		state_start_discovery,
		state_discovery,
		state_connecting,
		state_connected,
		state_last
	} states;

	states state = state_disconnected;

	const char *state_names[state_last] = {
		"reset",
		"disconnected",
		"start discovery",
		"discovery",
		"connecting",
		"connected",
	};

	typedef enum {
		task_none,
		task_boot,
		task_set_fast_connection,
		task_read_configurations,
		task_indicate_gpio_values,
		task_indicate_battery_voltage,
		task_indicate_uptime,
		task_check_security,
		task_clear_bonding,
		task_set_slow_connection,
		task_disconnect,
		task_last
	} tasks;

	tasks task = task_none;

	const char *task_names[task_last] = {
		"none",
		"boot",
		"set_fast_connection",
		"read_configurations",
		"indicate_gpio_values",
		"indicate_battery_voltage",
		"indicate_uptime",
		"check_security",
		"clear_bonding",
		"set_slow_connection",
		"set_disconnect"
	};

	typedef enum {
		procedure_write,
		procedure_read
	} procedure_types;

	procedure_types procedure_type = procedure_write;

	static const uint8 CONFIGURATION_AMOUNT_TO_READ = 4;
	const uint8 configurationsToRead[CONFIGURATION_AMOUNT_TO_READ] = {
		BP_MAIN_VERSION_PS_KEY,
		BP_MINOR_VERSION_PS_KEY,
		//BP_PROTOCOL_VERSION_MIN_PS_KEY,
		//BP_PROTOCOL_VERSION_MAX_PS_KEY,
		//BP_WHEEL_ANGLE_PS_KEY,
		//SC_BLE_DEVICE_TYPE_PS_KEY,
		INPUT_DEVICE_0_PS_KEY,
		INPUT_DEVICE_1_PS_KEY
	};

	uint8_t configurationPointerIndex = 0; 		// Points an item in configurationsToRead[]
	uint8_t configurationPointer = 0;				// PS_KEY being read

	uint8_t inputConfigurationDataSet = 0;		// true if there is a new input device configuration data
	uint8_t inputConfigurationDataAmount = 0;	// byte count of the new input device configuration data

	uint8 inputConfiguration[84];				// input device configuration data

	BTDevice deviceList[DEVICE_LIST_LENGTH];	// List of devices found in discovery
	uint8_t deviceCount;							// Count of devices found in discovery
	BTDevice connectedDevice;

	struct gecko_cmd_packet* evt;

	bd_addr addrToConnect;
	uint64_t taskTimer = 0;

	uint8_t initAttempts;

    ConnectAutomaticallyToWirelessWheel automaticConnectionMode = Never;

    bool speedChangedToSlow;

    void appHandleEvents(struct gecko_cmd_packet *evt);
	void change_task(tasks new_task);
	void change_state(states new_state);

	// Adds (advertised) device to the device list, the value is copied to the array. Array will be cleared if it becomes full.
	// Second argument will be set to true if the device was added to the list (it was new).
	uint8_t addDeviceToList(const BTDevice&, bool*);
	void printDevAddress(bd_addr addr);
	void printDeviceList();

	void checkSecurity();
	void readConfigurationData();

public:

	bool appBooted = false;

	UART_HandleTypeDef *huart = nullptr;

	volatile uint16_t rxBufPos=0;
	volatile uint8_t rxBuffer[BLERXBUFSIZE];

	uint32_t lastButtonState = 0;

	static constexpr uint8_t voltageMeasurementsOnEachConnectionInterval = 3;
    static constexpr float maxVoltageDropInPercents = 2.0;
	uint16_t batteryVoltage = 0;
    uint16_t batteryVoltages[voltageMeasurementsOnEachConnectionInterval*2] = { 0 };
    uint8_t batteryVoltageIndex = 0;
    LowBatteryWarning lowBatteryWarning = LowBatteryWarning::NotReady;

	uint32_t uptime = 0;
	int8_t connectionRSSI = -100;

	uint16_t NCPMajorVersion = 0;
	uint16_t NCPMinorVersion = 0;
	uint16_t NCPPatchVersion = 0;
	uint16_t NCPBuildVersion = 0;
	uint16_t BPMajorVersion = 0;
	uint16_t BPMinorVersion = 0;

	// PUBLIC METHODS

	BLEConnection();
	~BLEConnection();

	void setUart(UART_HandleTypeDef *_huart);
	void init();
	void loop();

	bool newInputConfiguration();
	uint8_t *getInputConfiguration();

	void startDiscovery();
	void stopDiscovery();

	void connectByDeviceAddress(bd_addr);
	void connectByDeviceNumber(uint8_t number);
	void disconnectFromDevice();

	uint8_t getAmountOfFoundDevices();
	BTDevice getDeviceDataByNumber(uint8_t num);
	void clearDeviceList();

	uint8_t RSSIToConnectionQuality(int8_t rssi); // RSSI to 0-100 value
	uint8_t batteryVoltageToBatteryLevel(uint16_t voltage); // voltage to 0-100 value

	uint16_t getBatteryVoltage(); // millivolts
	uint8_t getBatteryLevel(); // 0-100
	int8_t getConnectionRSSI(); // always negative value, dBm
	uint8_t getConnectionQuality(); // 0-100
    LowBatteryWarning getLowBatteryWarning();

	bool getConnectionState();
	BTDevice getConnectedDeviceInformation();
	bool getScanStatus();

	void forgetConnectedDevice();

    void setAutomaticConnectionMode(ConnectAutomaticallyToWirelessWheel mode);

    /**
     * @brief Forgets all buttonplate bondings from BLE module.
     */
    void forgetAllDevices();
};

#ifdef __cplusplus
};
#endif

#endif /* BLECONNECTION_H_ */


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

#include <eventLog.h>
#include <main.h>
#include "stm32f4xx_hal.h"
#include "types.h"
#include "malloc.h"
#include "../SimpleMotion/simplemotion_private.h"
#include "../SimpleMotion/simplemotion_defs.h"
#include "simplemotioncomms.h"
#include "cAnalogConfig.h"
#include "config_comm_defines.h"
#include "cHardwareConfig.h"
#include "USBGameController.h"
#include "usb_device.h"
#include "usbd_customhid.h"
#include "FfbEngine.h"
#include "../SimpleMotion/devicedeployment.h"
#include "stm32f4xx_hal_tim.h"
#include "hw_functions.h"

#include "stm32f407xx.h"

/* Toggle general debug mode stuff here! */
bool debugMode = true;
bool debugMode2 = false;
// uncomment to go to a mode that goes Operational that does not require servo drive.
//#define NO_IONI_HID_DEVICE_MODE 1

/* VERSION INFORMATION
 	 change these from fw_version.h for releases! */

uint8_t majorVersion= FW_MAJOR_VERSION;
uint8_t minorVersion= FW_MINOR_VERSION;
uint8_t buildVersion= FW_BUILD_VERSION;

/* Lowest compatible settings version this firmware is compatible with.
 * Write a converter function if the compatibility changes!
 */
uint8_t lowestCompatibleFlashSettingsMajorVersion = 0;
uint8_t lowestCompatibleFlashSettingsMinorVersion = 11;

/* Ioni Firmware version requirement */
uint16_t ioniFirmwareMinimumVersion = IONI_FW_VER;


/* Init version data table.
 * The values in table are used in replying a version request via USB.
 * DO NOT CHANGE THESE!!
 * */
uint8_t firmwareVersion[3];// = {majorVersion, minorVersion, buildVersion};

uint8_t IoniDrcFile[16384];



/* Private variables ---------------------------------------------------------*/

// HAL (init) variables
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef *debugHuart = nullptr;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_rx;
TIM_HandleTypeDef htim2_usbwaitdelay;
TIM_HandleTypeDef htim3_watchdogwaitdelay;
TIM_HandleTypeDef htim4_effecttimer;		//2500 Hz effect operation
TIM_HandleTypeDef htim5_usbsuspenddelay;		// used to suspend usb comms on SUSPEND
TIM_HandleTypeDef htim6_1MhzEffectClock;
TIM_HandleTypeDef htim7_250msledclock;

USBGameController joystick;
uint64_t millis=0;
uint32_t globaldebugvalue2=0;

uint64_t timervalue=0;

// Buffer to store all ADC input values via DMA
uint16_t ADC1_Buffer[ADC1_BUFFER_LENGTH];

// SimuCUBE Event Log system
eventLog simucubelog;


/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

// blocking 1us delay, if required.
void delay_us(uint32_t delay) {
	volatile uint32_t counter = 0;
	counter = (delay * (SystemCoreClock / 1000000U));
	while(counter != 0U)
	{
		counter--;
	}
}

#ifdef __cplusplus
extern "C" {
#endif
// blocking 1us delay, if required.
void delay_us_c(uint32_t delay) {
	delay_us(delay);
}
#ifdef __cplusplus
}
#endif




uint8_t usbsuspended=1;
/* USER CODE END PV */


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
// HAL Initialization function definitions. The functioan are at the bottom of the main.cpp.
void SystemClock_Config(void);
#ifdef __cplusplus
extern "C" {
#endif
void Error_Handler(void);
extern void MX_USART3_UART_Init(void);
#ifdef __cplusplus
}
#endif
static void MX_DMA_Init(void);

static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);

/* System status variables.
 * CurrentSystemStatus is the general state of the system, used directly in the state machine.
 * If a short excursion from normal has to be made, then
 * CurrentSystemStatus is first stored into nextSystemStatus,
 * and restored from there afterwards.
 */

volatile SystemStatus currentSystemStatus=BeforeInit;
volatile SystemStatus nextSystemStatus=BeforeInit;
// for cleanup of debug printing:
volatile SystemStatus previousSystemStatus = Internal_ForceMyEnumIntSize;
volatile SystemStatus previousSystemStatus2 = Internal_ForceMyEnumIntSize;

// wrappers to C+ code from STM HAL C code
#include "wrappers.h"


uint32_t bleCycles = 0;      // number of cycles last used to update bluetooth, undefined if there is no debugger
uint32_t joystickCycles = 0; // number of cycles last used to udpate "joystick" (the usb hid), undefined without debugger

/* HID report handling function
 * return false, if such a report was received
 * that changed currentSystemStatus.
 * The return value should be checked for not
 * to overwrite currentSystemStatus in the main loop
 * state machine.
 */
bool handleHidReports() {
	auto start = DWT->CYCCNT;

	if(joystick.gFFBDevice.bledetected==1) {

		// BLE Connection loop, must be run often. The connected button plate sends data every 7.5ms.
		// Internally the loop might advance by receiving a new packet and processing it.
		joystick.gFFBDevice.BLEConn.loop();

		// Create input device configuration if the BLE device has new configuration data
		if (joystick.gFFBDevice.BLEConn.newInputConfiguration()) {
			joystick.gFFBDevice.inputDeviceList.setConfiguration(joystick.gFFBDevice.BLEConn.getInputConfiguration()); //
			// new input configuration always means that a wireless wheel was connected.
			SMPlaySound(Connected);
			joystick.gFFBDevice.wirelessTorqueOffEvent=0;
			joystick.gFFBDevice.setSWWIdleDisconnectStatus(0);
			// if previous wheel had torque off button and it was off, need to reset this
			// too, and it will be read from device at first torque update cycle.
			joystick.gFFBDevice.wirelessTorqueOffButton=0;
		}

		// handle disconnected or low battery events
		joystick.gFFBDevice.handleWirelessEvents();
	}

	bleCycles = DWT->CYCCNT - start;

	start = DWT->CYCCNT;

	// read the oldest report and process it (for example configure effects)
	bool returnvalue = joystick.tryConsumeReceivedReport();

	joystickCycles = DWT->CYCCNT - start;

	return returnvalue;
}
/* End HID report handling function */



/* simplemotion comms functions to IONI drive        */
#include "simplemotioncomms.h"



/* BOOTLOADER JUMP FUNCTION */
#include "usbd_core.h"
extern USBD_HandleTypeDef hUsbDeviceFS;
#define SYSMEM_RESET_VECTOR            0x1FFF0000//0x1FFF77DE
#define RESET_TO_BOOTLOADER_MAGIC_CODE 0xDEADBEEF
#define BOOTLOADER_STACK_POINTER       0x1FFF0000

void simucube_bootloader(uint32_t magic_code) {
	volatile uint32_t code = magic_code;
	if(code==RESET_TO_BOOTLOADER_MAGIC_CODE) {
		*((unsigned long *)0x2001FFF0) = RESET_TO_BOOTLOADER_MAGIC_CODE;
	} else {
		*((unsigned long *)0x2001FFF0) = 0xb007b007;
	}
	//dfu_reset_to_bootloader_magic = RESET_TO_BOOTLOADER_MAGIC_CODE; // Write a scratch location at end of RAM (or wherever)
	*((unsigned long *)0x2001FFF0) = code;  // RESET_TO_BOOTLOADER_MAGIC_CODE = bootloader, others = just reboot
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_UART_DMAStop(&huart3);
	HAL_UART_DMAStop(&huart1);
	HAL_UART_DMAStop(&huart2);
	HAL_TIM_Base_Stop_IT(&htim2_usbwaitdelay);
	HAL_TIM_Base_Stop_IT(&htim3_watchdogwaitdelay);
	HAL_TIM_Base_Stop_IT(&htim4_effecttimer);
	HAL_TIM_Base_Stop_IT(&htim5_usbsuspenddelay);
	HAL_TIM_Base_Stop(&htim6_1MhzEffectClock);
	HAL_TIM_Base_Stop_IT(&htim7_250msledclock);
	USBD_Stop(&hUsbDeviceFS);
	USBD_DeInit(&hUsbDeviceFS);
	HAL_RCC_DeInit();
	HAL_DeInit();
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
    __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
	NVIC_SystemReset();
}
/* END BOOTLOADER JUMP FUNCTION */




/* Reading of Hardware Version ID */
void readHWVersion();
/* End Reading of Hardware Version ID */


/* The Main function */


int main(void)
{
	joystick.gFFBDevice.waitLastSimpleMotion = false;
	// HAL initializations
	// Reset of all peripherals, Initializes the Flash interface and the Systick.
	HAL_Init();

	// Configure the system clock
	SystemClock_Config();
	// Initialize all configured peripherals
	MX_GPIO_Init();
	SMPortSetMaster(false);
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_USART3_UART_Init();

	// sets start time for debug log time stamps
	simucubelog.resetLogStartTime();

	// test and init ble and debug uarts
	testAndInitBLE();

	// various hardware tests
	int hw_result = init_test();

	MX_TIM4_Init();
	MX_TIM5_Init();
	MX_TIM6_Init();
	MX_TIM7_Init();
	HAL_TIM_Base_Start_IT(&htim4_effecttimer);
	HAL_TIM_Base_Start(&htim6_1MhzEffectClock);
	HAL_TIM_Base_Start_IT(&htim7_250msledclock);

	// Initialize PWM&DIR outputs to LOW, so that "default" Ioni configs on SC1, that take
	// those kind of signals, don't try to interpret anything....
	setPWMDIRPinslow();

	// start all simucube ADC DMA traffics
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1_Buffer, ADC1_BUFFER_LENGTH);

	// start simplemotion SMBUS reception
	HAL_UART_Receive_DMA(&huart3,rxBuffer, RXBUFSIZE);

	readHwVersion();

	if(debugMode) printf("------- SimuCUBE %d.%d.%d BOOT -------\r\n",majorVersion,minorVersion,buildVersion);
	if(debugMode) printf("HAL initializations complete.\r\n");
	if(hw_result==0) {
		printf("invalid hardware for firmware.");
		simucube_bootloader(RESET_TO_BOOTLOADER_MAGIC_CODE);
		while(1) {
			handleHidReports();
		}
	}



#if 0
	// SIMPLEMOTION DEBUG TOGGLE
	if(debugMode) smSetDebugOutput(SMDebugMid,stdout); //prints lots of SM debugging, probably causes printf latency issues
#endif


	simucubelog.addEventParam(SimuCUBEBoot, majorVersion, true);
	simucubelog.addEventParam(SimuCUBEBoot, minorVersion, true);
	simucubelog.addEventParam(SimuCUBEBoot, buildVersion, true);

	// init firmware version table
	firmwareVersion[majorVersionIdx] = majorVersion;
	firmwareVersion[minorVersionIdx] = minorVersion;
	firmwareVersion[buildVersionIdx] = buildVersion;



	// state machine startup initializations

	if(debugMode) printf("Trying to load config from flash..\r\n");
	bool loadflash = joystick.gFFBDevice.loadConfigsFromFlash();
	if(debugMode) printf("Done trying to load config from flash\r\n");
	simucubelog.addEvent(loadedFlash, true);

	// initializes serial port and simplemotion
	preDriveInit();

	// Initializes the USB.
	MX_USB_DEVICE_Init();


	joystick.gFFBDevice.setEndstopLed(true);
	if(!loadflash) {
		if(debugMode) printf("flash data was for old version or invalid CRC32. Saving default configs to flash!\r\n");
		simucubelog.addEventParam(flashDataInvalid, 0);
		joystick.gFFBDevice.SetDefault(); // this also sets initialconfigdone = false

		bool saveflash = joystick.gFFBDevice.saveConfigsToFlash();
		if(!saveflash) currentSystemStatus = FlashFault;
		if(debugMode) printf("going to initialconfignotdone.\r\n");
		currentSystemStatus = SystemNotConfigured;
		// connect drive also, so that it will automatically update firmware on it.
		if(!InitializeDrive(true)) {
			// go to IoniFWUpgradeError
			currentSystemStatus = IoniFWUpgradeError;
		}
		else {
			currentSystemStatus = SystemNotConfigured;
		}

	} else {
		// flash loading was successfull
		// if default setting were possibly changed due to fw version update
		if(joystick.gFFBDevice.rewriteDefault == true) {
			if(debugMode) printf("rewriting profile 0\r\n");
			simucubelog.addEvent(rewriteDefaultProfile, true);
			// need to rewrite default profile
			joystick.gFFBDevice.mConfig.setReadOnlyDefaultProfileValues();
			if(!joystick.gFFBDevice.saveConfigsToFlash()) {
				currentSystemStatus = FlashFault;
				if(debugMode)printf("rewriting profile 0 failed\r\n");
			}
			joystick.gFFBDevice.unsavedSettings = 0;
		}

		// check if there is possibility that PC tool has sent bogus data for unused profile parameters
		if(joystick.gFFBDevice.checkIfPre012Data()) {
			joystick.gFFBDevice.initUninitializedProfileParametersPre012();
		}

		// check if flash had <1.0.0 flash data
		if(joystick.gFFBDevice.checkIfPre10000Data()) {
			joystick.gFFBDevice.resetParametersPre10000();
		}

		if(joystick.gFFBDevice.mConfig.hardwareConfig.hwSettings[addrVariousSettingsBits1]&(1<<bitInitialConfigDone))  {
			if(debugMode) printf("Initial configuration has been performed. Initializing drive.\r\n");
			simucubelog.addEvent(EventInitialConfigDone, true);
			currentSystemStatus = DriveInit;
		} else {

			currentSystemStatus = SystemNotConfigured;
			if(debugMode) printf("Initial configuration has not been performed. Drive not operational.\r\n");
			simucubelog.addEvent(EventInitialConfigNotDone, true);
			// connect drive also, so that it will automatically update firmware on it.
			if(!InitializeDrive(true)) {
				currentSystemStatus = IoniFWUpgradeError;
			} else {
				SetTorque(0, 0);
				currentSystemStatus = SystemNotConfigured;
			}

		}
	}

	joystick.gFFBDevice.setBLEAutoConnectMode();

	joystick.gFFBDevice.IRFFBModeEnabled = false;

	int reconnectCounter = 0;

	// end state machine startup initializations


	// for cleanup of debug printing:
	bool statusChanged = true;

	timervalue=millis;

	/* Infinite Main Loop */
	while (1) {
		if (currentSystemStatus!= previousSystemStatus) {
			statusChanged=true;
			previousSystemStatus = currentSystemStatus;
		} else {
			statusChanged=false;
		}

		switch(currentSystemStatus) {
			case SystemNotConfigured:
				if(debugMode && statusChanged) printf("state: SystemNotConfigured \r\n");
				if(statusChanged) simucubelog.addEvent(StateSystemNotConfigured);
				if(statusChanged) {
					disableSMWatchdog();
				}
				configureSystem();
				handleHidReports();
				break;

			case Driveconnect:
				InitializeDrive(true);
				currentSystemStatus = nextSystemStatus;
				break;

			case DriveInit:
				if(debugMode) printf("state: DriveInit \r\n");
				if(statusChanged) simucubelog.addEvent(StateDriveInit);
				if(!handleHidReports()) {
					break;
				}

				// only if e-stop is up?
				InitializeDrive(); //changes also currentSystemStatus, initializes watchdog, etc.
				break;

			case DriveInitSuccess:
			{
				if(debugMode && statusChanged) printf("state: DriveInitSuccess \r\n");
				if(statusChanged) simucubelog.addEvent(StateDriveInitSuccess);
				bool hidcommand = handleHidReports();
				if(!hidcommand) break; // got a hid report already telling to do something else

				if(joystick.gFFBDevice.mConfig.hardwareConfig.hwSettings[addrIndexingMode] == indexPointIndexing) {
					currentSystemStatus = WaitFindingIndexPoint;
				} else {
					if(!(joystick.gFFBDevice.mConfig.hardwareConfig.hwSettings[addrVariousSettingsBits1]&(1<<bitInitialConfigDone))) {
						currentSystemStatus = DriveInitSuccessPause;
					} else {
						currentSystemStatus = SetBaudrateForOperational;
					}
				}

				break;
			}

			case WaitFindingIndexPoint:
			{
				joystick.gFFBDevice.setIndexpointFound(false);
				if(debugMode && statusChanged) printf("state: Waiting for index point\r\n");
				if(statusChanged) simucubelog.addEvent(StateWaitingIndex);
				int indexpoint = 0;
				bool status = WaitForIndexPulse(indexpoint);
				if(!status) {
					// failure,or was told to go elsewhere
					if(debugMode) printf("cancelled finding index point\r\n");
					break;
				}

				SetTorque(0, 0);

				joystick.gFFBDevice.setIndexpointFound(true);
				SM_STATUS indexstatus = smSetParameter(joystick.gFFBDevice.mSMBusHandle, 1, SMP_SIMUCUBE_WHEEL_CENTER_OFFSET, -1);
				joystick.gFFBDevice.angle.reset();

				// early init of angle mode here
				joystick.gFFBDevice.angle.setIndexpointOffset(joystick.gFFBDevice.mConfig.hardwareConfig.hwSettings[addrEncoderOffset]);
				joystick.gFFBDevice.angle.setCenteringMode(joystick.gFFBDevice.mConfig.hardwareConfig.hwSettings[addrIndexingMode]);

				joystick.gFFBDevice.CalcTorqueCommand(); // forces reset of angle calculations
				if(joystick.gFFBDevice.mConfig.hardwareConfig.hwSettings[addrVariousSettingsBits1]&(1<<bitInitialConfigDone)) {
					currentSystemStatus = SetBaudrateForOperational;
				} else {
					currentSystemStatus = DriveInitSuccessPause;
				}
				break;
			}
			case DriveInitSuccessPause:
			{
				if(debugMode && statusChanged) printf("state: DriveInitSuccessPause \r\n");
				if(statusChanged) {
					simucubelog.addEvent(StateDriveInitSuccessPause);
				}
				handleHidReports();
				int32_t positionFB = 0;
				// do something to keep possible watchdog happy
				smRead1Parameter(joystick.gFFBDevice.mSMBusHandle, 1, SMP_ACTUAL_POSITION_FB_NEVER_RESETTING, &positionFB);//read 32 bit position // to keep watchdog happy
				HAL_Delay(1); // to not overflow IONI simplemotion task.
				break;
			}

			case SetBaudrateForOperational:
			{
				if(debugMode && statusChanged) printf("state: setBaudrateForOperational \r\n");
				if(statusChanged) simucubelog.addEvent(StateSetBaudrate);

				initSMBusBaudrate(true);
				joystick.gFFBDevice.driveParamTracker.updateParameters(true);
				initFastUpdateCycleMode();
				currentSystemStatus=BeforeOperational;
				break;
			}

			case BeforeOperational:
			{
				if(debugMode && statusChanged) printf("state: BeforeOperational \r\n");
				if(statusChanged) simucubelog.addEvent(StateBeforeOperational);
				joystick.gFFBDevice.initVariables(); // this needs to be done to get from encoder resolution to real angle

				// set only drive parameters according to profile, and only if they are changed
				joystick.gFFBDevice.driveParamTracker.updateParameters(false);

				currentSystemStatus = Operational;
				break;
			}

			/* main operating state! */
			case Operational:
			{
				if(debugMode && statusChanged){
					printf("state: Operational \r\n");
				}
				if(statusChanged) {
					simucubelog.addEvent(StateOperational);
				}

				if(!(joystick.gFFBDevice.mConfig.hardwareConfig.hwSettings[addrVariousSettingsBits1]&(1<<bitInitialConfigDone))) {
					currentSystemStatus = SystemNotConfigured;
					break;
				}

				// CalcTorqueCommand currently waits the time we need to sleep so while this gets called
				// very often it will in practice return

				bool newCalculation = joystick.gFFBDevice.CalcTorqueCommand(); //reads encoder counter too
				if (newCalculation) {
					joystick.update(joystick.gFFBDevice.usb_steering_angle()); // updates also everything else (buttons)
				} else {
					joystick.update_unchanged_steering(); // not sure if we need these for too long
				}

				handleHidReports();

				break;
			}
			/* end main operating state! */

			case testMode:
			{
				if(debugMode && statusChanged){
					printf("state: testMode \r\n");
				}
				// testmode does nothing on simucube 1
				break;
			}







			case saveSettingsToFlash:
				if(debugMode && statusChanged) printf("state: saveSettingsToFlash \r\n");
				if(statusChanged) simucubelog.addEvent(StateSavetoFlash);
				// todo: handle ioni comms
				// todo: how to reply this success/failure to PC?
				// maybe via statusReplyPacket using .previousCommandSuccess field!
				// this needs its own state, because the flashing is slow process (1-2s).
				//deinit drive comms maybe?
				disableSMWatchdog();
				if(!joystick.gFFBDevice.saveConfigsToFlash()) {
					// failure -> currentSystemStatus = FlashFault;
					//enableSMWatchdog();
					currentSystemStatus = FlashFault;
					break;
				}
				joystick.gFFBDevice.unsavedSettings = 0;


				// save also flags that are set to drive.
				SetTorque(0, 0);
				resetMaxMMC();
				saveDriveCfg();
				joystick.gFFBDevice.torqueOffSavingDelay=1;
				joystick.gFFBDevice.torqueOffSavingDelayStart=millis;

				enableSMWatchdog();
				setCurrentProfileMMC();

				// todo: check for sc1
				if(!joystick.gFFBDevice.angle.readStartPosition()) {
					if(debugMode) {
						printf("couldn't read 32bit enc pos\r\n");
					}
				}

				// success -> currentSystemStatus = nextSystemStatus;
				currentSystemStatus = nextSystemStatus;
				break;

			case goFreshStart:
				if(debugMode && statusChanged) printf("state: goFreshStart \r\n");
				if(statusChanged) simucubelog.addEvent(StateFreshStart);
				resetSettings();
				break;

			case FlashFault:
				if(debugMode && statusChanged) printf("state: FlashFault \r\n");
				if(statusChanged) simucubelog.addEvent(StateFlashFault);
				// flash writing fault. Provisional state to indicate it somehow?
				if(!handleHidReports()) {
					break;
				}
				else {
					currentSystemStatus = nextSystemStatus;
					break;
				}

			case releaseSMBus:
				if(debugMode && statusChanged) printf("state: releaseSMBus \r\n");
				if(statusChanged) simucubelog.addEvent(StateReleaseSMBus);
				SMPortSetMaster(true);
				// have gotten a command to release smbus
				if(enableSMWatchdog()) {
					resetMaxMMC();
					initSMBusBaudrate(false);
				} // do not try to access smbus if simple watchdog command failed.
				else {
					initSMBusBaudrate(false);
				}
				SMPortSetMaster(false);
				currentSystemStatus = SMBusReleased;
				joystick.gFFBDevice.setSmReleased(true);
				break;

			case SMBusReleased:
				if(debugMode && statusChanged) printf("state: SMBusReleased \r\n");
				if(statusChanged) simucubelog.addEvent(StateSMBusReleased);
				handleHidReports();
				reconnectCounter = 0;
				break;

			case regainSMBus:
			{
				if(debugMode && statusChanged) printf("state: regainSMBus \r\n");
				if(statusChanged) simucubelog.addEvent(StateRegainSMBus);
				bool hidcommand = handleHidReports();
				if(!hidcommand) {
					break;	// was told to do something else
				}
				if(reconnectCounter>4) {
					simucubelog.addEvent(StateRegainSMFailedTimeOut);
					SMPortSetMaster(false);
					currentSystemStatus = DriveConnectionError;
					break;
				}
				if(millis > (timervalue+3000)) {
					reconnectCounter++;
					SMPortSetMaster(true);
					int status = initSMBusBaudrate(false);
					if(status == -1) {
						if(debugMode) printf("couldn't initialize smbus baud rate!\r\n");
						simucubelog.addEvent(StateRegainSMFailed);
						SMPortSetMaster(false);
						timervalue = millis;
						break;
					}
					restartSMDrive();
					if(!readDriveParams()) {
						if(debugMode) printf("couldn't read some parameters\r\n");
						break; // something not working
					}

					if((joystick.gFFBDevice.mConfig.hardwareConfig.hwSettings[addrVariousSettingsBits1]&(1<<bitInitialConfigDone))==0) {
						if(!InitializeDrive(true)) { // to make sure still have correct FW version
							// go to IoniFWUpgradeError?
							currentSystemStatus = SystemNotConfigured;
							break;
						}
						currentSystemStatus = SystemNotConfigured;
						break;
					} else {
						if(!InitializeDrive()) {
							currentSystemStatus = DriveConnectionError;
							break;
						}
						initFastUpdateCycleMode();

						const auto resp = SetTorque(0, 0);
						joystick.gFFBDevice.angle.reset();

						// do this only, if there is a chance that system will go operational.
						if((joystick.gFFBDevice.mConfig.hardwareConfig.hwSettings[addrVariousSettingsBits1]&(1<<bitInitialConfigDone)) && nextSystemStatus==Operational) {
							// go to safe mode
							joystick.gFFBDevice.currentprofileindex=0;
							enableSMWatchdog();
							nextSystemStatus=SetBaudrateForOperational;
							joystick.gFFBDevice.driveParamTracker.updateParameters(true);
							simucubelog.addEventParam(Command_disableForces,1);
							joystick.gFFBDevice.forcesEnabled = 0;
							joystick.gFFBDevice.forcesDisabledForSafety=1;
						}
						break;
					}
					timervalue=millis;
				}
				break;
			}

			case DriveConnectionError:
			{

				if(debugMode && statusChanged) printf("state: DriveConnectionError \r\n");
				if(statusChanged) simucubelog.addEvent(StateDriveConnectonError);
				// drive connection was lost. Handle hid reports here.
				bool hidcommand = handleHidReports();
				if(!hidcommand) {
					break;	// was told to do something else
				}
				else {
					// try to re-init comms every 10000 ms
					if(millis > (timervalue+10000)) {
						SMPortSetMaster(true);
						if(initSMBusBaudrate(false)==0) {
							//success. Return to the state where system was last.
							if(joystick.gFFBDevice.mConfig.hardwareConfig.hwSettings[addrVariousSettingsBits1]&(1<<bitInitialConfigDone)) {
								currentSystemStatus = DriveInit;
							} else {
								currentSystemStatus = SystemNotConfigured;
							}
							//currentSystemStatus = nextSystemStatus;
						} else {
							SMPortSetMaster(false);
							if(debugMode) printf("E!\r\n");
							timervalue=millis;
						}
					}
					break;
				}
			}
			case DriveWaitClearfaults:
			{
				if(debugMode && statusChanged) printf("state: DriveWaitClearfaults \r\n");
				if(statusChanged) simucubelog.addEvent(StateWaitClearFaults);
				bool hidcommand = true;
				hidcommand = handleHidReports();
				if(!hidcommand) {
					break;  // was told to do something else
				}
				else {
					// try to do what was being done
					currentSystemStatus = nextSystemStatus;
					break;
				}
			}

			case UploadingDriveFirmware:
				if(debugMode && statusChanged) printf("state: Uploading Drive Firmware \r\n");
				if(statusChanged) simucubelog.addEvent(StateUploadDriveFW);
				handleHidReports();
				break;

			case IoniFWUpgradeError:
				if(debugMode && statusChanged) printf("state: Drive Firmware upgrade error \r\n");
				if(statusChanged) simucubelog.addEvent(StateUploadDriveFWError);
				handleHidReports();
				break;

			case ApplyIoniDrcData:
			{
				disableSMWatchdog();
				if(debugMode && statusChanged) printf("state: ApplyIoniDrcData \r\n");
				if(statusChanged) simucubelog.addEvent(StateApplyDRCData);

				SMApplyDRCDataFromBuffer(&IoniDrcFile[0], joystick.gFFBDevice.IoniDrcDataReceivedBytes);

				enableSMWatchdog();

				// after applying DRC, read MMC and other important parameters
				smint32 res;
				smRead1Parameter(joystick.gFFBDevice.mSMBusHandle, 1, SMP_TORQUELIMIT_CONT, &res);
				joystick.gFFBDevice.mConfig.hardwareConfig.hwSettings[addrMaxMotorCurrent] = res;
				joystick.gFFBDevice.unsavedSettings = 1;
				simucubelog.addEvent(readMMCFromIONI, res);
				currentSystemStatus = DriveInit;
				break;
			}

			case DriveWaitReady:
				if(debugMode && statusChanged) printf("state: Waiting for drive to get ready\r\n");
				if(statusChanged) simucubelog.addEvent(StateDriveWaitReady);
				handleHidReports();
				break;

			case JumpToBootLoader:
				if(debugMode && statusChanged) printf("state: going to simucube bootloader!\r\n");
				if(statusChanged) simucubelog.addEvent(StateJumpBootloader);
				resetMaxMMC();
				HAL_Delay(5);
				// init drive to default baudrate
				initSMBusBaudrate(false);
				simucube_bootloader(RESET_TO_BOOTLOADER_MAGIC_CODE);
				while(1) {
					asm("nop");
				}
				break;

			case AutoDetectCommutationSensors:
            {
            	simucubelog.addEvent(StateAutoSetupCommucation, true);
            	handleHidReports();
            	disableSMWatchdog();
            	int32_t fbdtype = 0;
            	smRead1Parameter(joystick.gFFBDevice.mSMBusHandle, 1, SMP_FB1_DEVICE_SELECTION, &fbdtype);
            	const bool isBiss = fbdtype == SMP_FBD_SERIALDATA;
                if(!isBiss) {
                	joystick.gFFBDevice.mConfig.hardwareConfig.hwSettings[addrAutoCommutationStatus]=FailedUnsupportedEncoder;
                	printf("wrong encodertype\r\n");
                    currentSystemStatus=previousSystemStatus2;
                    simucubelog.addEventParam(AutoCommutationFault, 1, true);
                	break;
                }
                if((previousSystemStatus2 != Operational) && (previousSystemStatus2 != DriveInitSuccessPause)) {
                	joystick.gFFBDevice.mConfig.hardwareConfig.hwSettings[addrAutoCommutationStatus]=FailedNotInitialized;
                    currentSystemStatus=previousSystemStatus2;
                    simucubelog.addEventParam(AutoCommutationFault, 2, true);
                    printf("was not operational\r\n");
                    break;
                }
                resetMaxMMC();
                joystick.gFFBDevice.mConfig.hardwareConfig.hwSettings[addrAutoCommutationStatus] = Started;
                smSetParameter(joystick.gFFBDevice.mSMBusHandle, 1, SMP_SYSTEM_CONTROL,SMP_SYSTEM_CONTROL_START_COMMUTATION_SENSOR_AUTOSETUP);
                int elapsedtime_ms=0, timelimit=30000;
                bool abort=false;
                while((elapsedtime_ms<timelimit)&&(abort==false))
                {
                	uint16_t delaycounter=0;
                	while(delaycounter<1000) {
                		HAL_Delay(2);
                		handleHidReports();
                		delaycounter+=2;
                	}
                    elapsedtime_ms+=1000;
                    smint32 state = updateAutoCommutationSetup();

                    switch(state)
                    {
						case 400000://busy
							joystick.gFFBDevice.autoCommutationStatus=Busy;
							simucubelog.addEvent(AutoCommutationBusy, false);
							printf("busy\r\n");
							break;
						case 400100://torq ctrl not ready
							joystick.gFFBDevice.mConfig.hardwareConfig.hwSettings[addrAutoCommutationStatus]=FailedUninitialized;
							abort=true;
							printf("fault_torq\r\n");
							simucubelog.addEventParam(AutoCommutationFault, 3, true);
							break;
						case 400200://no torque
							joystick.gFFBDevice.mConfig.hardwareConfig.hwSettings[addrAutoCommutationStatus]=FailedLowCurrent1;
							simucubelog.addEventParam(AutoCommutationFault, 4, true);
							printf("fault_lowcurrent\r\n");
							abort=true;
						   break;
						case 400210://
							joystick.gFFBDevice.mConfig.hardwareConfig.hwSettings[addrAutoCommutationStatus]=FailedNoisyBadHall;
							simucubelog.addEventParam(AutoCommutationFault, 5, true);
							printf("fault_noisebad\r\n");
							abort=true;
							break;
						case 400220://
							joystick.gFFBDevice.mConfig.hardwareConfig.hwSettings[addrAutoCommutationStatus]=FailedInvalidHallSequence;
							simucubelog.addEventParam(AutoCommutationFault, 6, true);
							printf("fault_invalidseq\r\n");
							abort=true;
							break;
						case 400400://
							joystick.gFFBDevice.mConfig.hardwareConfig.hwSettings[addrAutoCommutationStatus]=FailedLowCurrent2;
							simucubelog.addEventParam(AutoCommutationFault, 7, true);
							printf("fault_lowcurrent2\r\n");
							abort=true;
							break;
						case 400999://ok
							joystick.gFFBDevice.mConfig.hardwareConfig.hwSettings[addrAutoCommutationStatus]=Success;
							joystick.gFFBDevice.mConfig.hardwareConfig.hwSettings[addrVariousSettingsBits1]|=(1<<bitAutoCommutationMode);
							saveDriveCfg();
							simucubelog.addEvent(AutoCommutationSuccess, true);
							printf("success\r\n");
							joystick.gFFBDevice.unsavedSettings=1;
							abort=true;
							break;
						default://wtf
							joystick.gFFBDevice.mConfig.hardwareConfig.hwSettings[addrAutoCommutationStatus]=UnknownResult;
							simucubelog.addEventParam(AutoCommutationFault, 10, true);
							printf("??\r\n");
							abort=true;
							break;
                    }
                }
                setCurrentProfileMMC();
                if(elapsedtime_ms>=timelimit)
                {
                	joystick.gFFBDevice.mConfig.hardwareConfig.hwSettings[addrAutoCommutationStatus]=Timeout;
                	simucubelog.addEventParam(AutoCommutationFault, 11, true);
                }
                currentSystemStatus=previousSystemStatus2;
                enableSMWatchdog();
                break;
            }
			case Internal_ForceMyEnumIntSize:
			default:
				if(debugMode) printf("state: default \r\n");
				if(statusChanged) simucubelog.addEvent(StateError);
				// failure?
				while(1) {
					asm("nop");
				}
		}
	}
}

/* End The Main function */




/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  //HAL_UART_Transmit(&huart4, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}



/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 288;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  //RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  //for much slower ADC, to get rid of crosstalk issues:
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 1, 0);
}

void start_timer2_15ms_delay() {
	joystick.USBReportsEnabled = false;
	MX_TIM2_Init();
	HAL_TIM_Base_Start_IT(&htim2_usbwaitdelay);
}

//note: for timers, you can use mikroelektronika timer calculator
// with stm32f2xx/3xx/4xx, MCU 144 MHz, /4.

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;


#if 1
  // 15ms:
  htim2_usbwaitdelay.Instance = TIM2;
  htim2_usbwaitdelay.Init.Prescaler = 17;
  htim2_usbwaitdelay.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2_usbwaitdelay.Init.Period = 59999;
  htim2_usbwaitdelay.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
#endif

#if 0
  // 1s
  htim2_usbwaitdelay.Instance = TIM2;
  htim2_usbwaitdelay.Init.Prescaler = 1124;
  htim2_usbwaitdelay.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2_usbwaitdelay.Init.Period = 63999;
  htim2_usbwaitdelay.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
#endif
  if (HAL_TIM_Base_Init(&htim2_usbwaitdelay) != HAL_OK)
  {
	  Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2_usbwaitdelay, &sClockSourceConfig) != HAL_OK)
  {
	  Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2_usbwaitdelay, &sMasterConfig) != HAL_OK)
  {
	  Error_Handler();
  }

}



/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;


  // 400us (2500Hz)
  htim4_effecttimer.Instance = TIM4;
  htim4_effecttimer.Init.Prescaler = 0;
  htim4_effecttimer.Init.CounterMode = TIM_COUNTERMODE_UP;
  // 2500 Hz = 28799, 1250 Hz = 57999
  htim4_effecttimer.Init.Period = 57599;//28799;//57599;//28799;//57599;;
  htim4_effecttimer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;


  if (HAL_TIM_Base_Init(&htim4_effecttimer) != HAL_OK)
  {
	  Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4_effecttimer, &sClockSourceConfig) != HAL_OK)
  {
	  Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4_effecttimer, &sMasterConfig) != HAL_OK)
  {
	  Error_Handler();
  }

}


/* TIM5 init function */
static void MX_TIM5_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;


  // 15s
  htim5_usbsuspenddelay.Instance = TIM5;
  htim5_usbsuspenddelay.Init.Prescaler = 16874;
  htim5_usbsuspenddelay.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5_usbsuspenddelay.Init.Period = 63999;
  htim5_usbsuspenddelay.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;


  if (HAL_TIM_Base_Init(&htim5_usbsuspenddelay) != HAL_OK)
  {
	  Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5_usbsuspenddelay, &sClockSourceConfig) != HAL_OK)
  {
	  Error_Handler();
  }
  __HAL_TIM_CLEAR_IT(&htim5_usbsuspenddelay,TIM_IT_UPDATE);
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5_usbsuspenddelay, &sMasterConfig) != HAL_OK)
  {
	  Error_Handler();
  }

}

/* TIM5 init function */
static void MX_TIM6_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;


  // 15s
  htim6_1MhzEffectClock.Instance = TIM6;
  htim6_1MhzEffectClock.Init.Prescaler = SystemCoreClock/1/1000000U-1;
  htim6_1MhzEffectClock.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6_1MhzEffectClock.Init.Period = 1000000U;
  htim6_1MhzEffectClock.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;


  if (HAL_TIM_Base_Init(&htim6_1MhzEffectClock) != HAL_OK)
  {
	  Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim6_1MhzEffectClock, &sClockSourceConfig) != HAL_OK)
  {
	  Error_Handler();
  }
  __HAL_TIM_CLEAR_IT(&htim6_1MhzEffectClock,TIM_IT_UPDATE);
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6_1MhzEffectClock, &sMasterConfig) != HAL_OK)
  {
	  Error_Handler();
  }

}


static void MX_TIM7_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;


  // 250ms
  htim7_250msledclock.Instance = TIM7;
  htim7_250msledclock.Init.Prescaler = 287;
  htim7_250msledclock.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7_250msledclock.Init.Period = 62499;
  htim7_250msledclock.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;


  if (HAL_TIM_Base_Init(&htim7_250msledclock) != HAL_OK)
  {
	  Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim7_250msledclock, &sClockSourceConfig) != HAL_OK)
  {
	  Error_Handler();
  }
  __HAL_TIM_CLEAR_IT(&htim7_250msledclock,TIM_IT_UPDATE);
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7_250msledclock, &sMasterConfig) != HAL_OK)
  {
	  Error_Handler();
  }

}
extern bool waitingWatchDog;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM5) {
		HAL_TIM_Base_Stop_IT(&htim5_usbsuspenddelay);
		usbsuspended=0;
		joystick.gFFBDevice.toggleEndstopLed();
		printf("resume\r\n");
	}
	else if(htim->Instance == TIM2) {
		//HAL_GPIO_TogglePin(LED3_OUT_GPIO_Port, LED3_OUT_Pin);
		joystick.USBReportsEnabled = true;
		HAL_TIM_Base_Stop_IT(&htim2_usbwaitdelay);
		//HAL_TIM_Base_Start_IT(&htim2_usbwaitdelay);
	}
	else if(htim->Instance == TIM3) {
		waitingWatchDog = false;
		HAL_TIM_Base_Stop_IT(&htim3_watchdogwaitdelay);
	}
	else if(htim->Instance == TIM4) {
		//HAL_GPIO_TogglePin(LED3_OUT_GPIO_Port, LED3_OUT_Pin);
		//HAL_GPIO_TogglePin(LED4_OUT_GPIO_Port, LED4_OUT_Pin);
		joystick.gFFBDevice.FFBLoopWait = false;
	}
	else if(htim->Instance == TIM7) {
		joystick.gFFBDevice.doLeds();
	}
}



/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  //HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 1, 0);   // usart3, do not enable!
  //HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);           // usart3, do not enable!

  // ADC DMA
  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  //HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 0);
  //HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  //HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 1, 0);
  //HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}





/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler */
}

void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#if 0
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{
	printf("RST\r\n");
}

void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{
	printf("SPND\r\n");
}

void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
	printf("RSME\r\n");
}

void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
	printf("CNNCT\r\n");
}

void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
	printf("DSC\r\n");
}
#endif

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

/**
 * HardFaultHandler_C:
 * This is called from the HardFault_HandlerAsm with a pointer the Fault stack
 * as the parameter. We can then read the values from the stack and place them
 * into local variables for ease of reading.
 * We then read the various Fault Status and Address Registers to help decode
 * cause of the fault.
 * The function ends with a BKPT instruction to force control back into the debugger
 */
void HardFault_HandlerC(unsigned long *hardfault_args) __asm__("HardFault_HandlerC");;
void HardFault_HandlerC(unsigned long *hardfault_args)
{
  volatile unsigned long stacked_r0 ;
  volatile unsigned long stacked_r1 ;
  volatile unsigned long stacked_r2 ;
  volatile unsigned long stacked_r3 ;
  volatile unsigned long stacked_r12 ;
  volatile unsigned long stacked_lr ;
  volatile unsigned long stacked_pc ;
  volatile unsigned long stacked_psr ;
  volatile unsigned long _CFSR ;
  volatile unsigned long _HFSR ;
  volatile unsigned long _DFSR ;
  volatile unsigned long _AFSR ;
  volatile unsigned long _BFAR ;
  volatile unsigned long _MMAR ;
  stacked_r0 = ((unsigned long)hardfault_args[0]) ;
  stacked_r1 = ((unsigned long)hardfault_args[1]) ;
  stacked_r2 = ((unsigned long)hardfault_args[2]) ;
  stacked_r3 = ((unsigned long)hardfault_args[3]) ;
  stacked_r12 = ((unsigned long)hardfault_args[4]) ;
  stacked_lr = ((unsigned long)hardfault_args[5]) ;
  stacked_pc = ((unsigned long)hardfault_args[6]) ;
  stacked_psr = ((unsigned long)hardfault_args[7]) ;
  // Configurable Fault Status Register
  // Consists of MMSR, BFSR and UFSR
  _CFSR = (*((volatile unsigned long *)(0xE000ED28))) ;
  // Hard Fault Status Register
  _HFSR = (*((volatile unsigned long *)(0xE000ED2C))) ;
  // Debug Fault Status Register
  _DFSR = (*((volatile unsigned long *)(0xE000ED30))) ;
  // Auxiliary Fault Status Register
  _AFSR = (*((volatile unsigned long *)(0xE000ED3C))) ;
  // Read the Fault Address Registers. These may not contain valid values.
  // Check BFARVALID/MMARVALID to see if they are valid values
  // MemManage Fault Address Register
  _MMAR = (*((volatile unsigned long *)(0xE000ED34))) ;
  // Bus Fault Address Register
  _BFAR = (*((volatile unsigned long *)(0xE000ED38))) ;
  __asm("BKPT #0\n") ; // Break into the debugger
}


void NMI_Handler(void)
{
    //ks http://mcuoneclipse.com/2012/11/24/debugging-hard-faults-on-arm-cortex-m/
      __asm volatile (
        " movs r0,#4       \n"
        " movs r1, lr      \n"
        " tst r0, r1       \n"
        " beq _MSP         \n"
        " mrs r0, psp      \n"
        " b _HALT          \n"
      "_MSPNMI:               \n"
        " mrs r0, msp      \n"
      "_HALTNMI:              \n"
        " ldr r1,[r0,#20]  \n"
        " b HardFault_HandlerC \n"
        " bkpt #0          \n"
      );
      while(1)asm("nop");
}
void HardFault_Handler(void)
{
    //ks http://mcuoneclipse.com/2012/11/24/debugging-hard-faults-on-arm-cortex-m/
      __asm volatile (
        " movs r0,#4       \n"
        " movs r1, lr      \n"
        " tst r0, r1       \n"
        " beq _MSP         \n"
        " mrs r0, psp      \n"
        " b _HALT          \n"
      "_MSP:               \n"
        " mrs r0, msp      \n"
      "_HALT:              \n"
        " ldr r1,[r0,#20]  \n"
        " b HardFault_HandlerC \n"
        " bkpt #0          \n"
      );
      while(1)asm("nop");
}



#endif


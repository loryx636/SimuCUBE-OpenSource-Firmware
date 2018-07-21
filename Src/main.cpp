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

/**
  ******************************************************************************
  * File Name          : main.cpp
  * Description        : Main program body
  ******************************************************************************
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <eventlog.h>
#include <main.h>
#include "stm32f4xx_hal.h"
#include "types.h"
#include "malloc.h"
#include "../SimpleMotion/simplemotion_private.h"
#include "../SimpleMotion/simplemotion_defs.h"
#include "cAnalogConfig.h"
#include "config_comm_defines.h"
#include "cHardwareConfig.h"
#include "usbgamecontroller.h"
#include "usb_device.h"
#include "usbd_customhid.h"
#include "ffbengine.h"
//#include "Command.h"
#include "../SimpleMotion/devicedeployment.h"
#include "stm32f4xx_hal_tim.h"


/* Toggle general debug mode here! */
bool debugMode = true;
bool debugMode2 = false;


/* VERSION INFORMATION
 	 change these for releases! */
uint8_t majorVersion= 0;
uint8_t minorVersion= 10;
uint8_t buildVersion= 4;

/* Lowest compatible settings version this firmware is compatible with.
 * Write a converter function if the compatibility changes!
 */
uint8_t lowestCompatibleFlashSettingsMajorVersion = 0;
uint8_t lowestCompatibleFlashSettingsMinorVersion = 8;

/* Ioni Firmware version requirement */
uint16_t ioniFirmwareMinimumVersion = 10707;



/* Init version data table.
 * The values in table are used in replying a version request via USB.
 * DO NOT CHANGE THESE!!
 * */
uint8_t firmwareVersion[3];// = {majorVersion, minorVersion, buildVersion};

uint8_t IoniDrcFile[16384];



/* Private variables ---------------------------------------------------------*/

// HAL (init) variables
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
TIM_HandleTypeDef htim2_usbwaitdelay;
TIM_HandleTypeDef htim3_watchdogwaitdelay;
TIM_HandleTypeDef htim4_effecttimer;		//2500 Hz effect operation
TIM_HandleTypeDef htim5_usbsuspenddelay;		// used to suspend usb comms on SUSPEND
TIM_HandleTypeDef htim6_1MhzEffectClock;

bool FFB2500HzWait;
s32 encoderCounter=0;
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
void delay_us(uint32_t delay_us) {
	volatile uint32_t counter = 0;
	counter = (delay_us * (SystemCoreClock / 1000000U));
	while(counter != 0U)
	{
		counter--;
	}
}



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
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);

static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);

static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM6_Init(void);









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




/* HID report handling function
 * return false, if such a report was received
 * that changed currentSystemStatus.
 * The return value should be checked for not
 * to overwrite currentSystemStatus in the main loop
 * state machine.
 */
bool handleHidReports() {
	if(!joystick.USBReportsEnabled) {
		// can't risk spamming any answers to usb output buffer.
		return true;
	}
    // handle HID reports
	if(joystick.getPendingReceivedReportCount())
	{
		HID_REPORT recv_report=joystick.getReceivedReport();
       	return joystick.handleReceivedHIDReport(recv_report);
	}
	return true;
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

static void simucube_bootloader(void) {
	//dfu_reset_to_bootloader_magic = RESET_TO_BOOTLOADER_MAGIC_CODE; // Write a scratch location at end of RAM (or wherever)
	*((unsigned long *)0x2001FFF0) = RESET_TO_BOOTLOADER_MAGIC_CODE;
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_UART_DMAStop(&huart3);
	/*TIM_HandleTypeDef htim2_usbwaitdelay;
	TIM_HandleTypeDef htim3_watchdogwaitdelay;
	TIM_HandleTypeDef htim4_effecttimer;*/
	HAL_TIM_Base_Stop_IT(&htim2_usbwaitdelay);
	HAL_TIM_Base_Stop_IT(&htim3_watchdogwaitdelay);
	HAL_TIM_Base_Stop_IT(&htim4_effecttimer);
	HAL_TIM_Base_Stop_IT(&htim5_usbsuspenddelay);
	HAL_TIM_Base_Stop(&htim6_1MhzEffectClock);
	USBD_Stop(&hUsbDeviceFS);
	USBD_DeInit(&hUsbDeviceFS);
	HAL_RCC_DeInit();
	HAL_DeInit();
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
    //__disable_irq();
    __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
	//__set_MSP(ADDR_FLASH_SECTOR_4);
	NVIC_SystemReset();
}
/* END BOOTLOADER JUMP FUNCTION */




/* Reading of Hardware Version ID */
void readHWVersion() {
	uint8_t id = 0;
	if(HAL_GPIO_ReadPin(IDSEL_0_GPIO_Port,IDSEL_0_Pin)==GPIO_PIN_RESET) {
		id=id+1;
	}
	if(HAL_GPIO_ReadPin(IDSEL_1_GPIO_Port,IDSEL_1_Pin)==GPIO_PIN_RESET) {
		id=id+2;
	}
	if(HAL_GPIO_ReadPin(IDSEL_2_GPIO_Port,IDSEL_2_Pin)==GPIO_PIN_RESET) {
		id=id+4;
	}
	switch(id) {
	case 0:
		joystick.gFFBDevice.scHWVersion = hwv1;
		break;
	case 1:
		joystick.gFFBDevice.scHWVersion = hwv2;
		break;
	default:
		joystick.gFFBDevice.scHWVersion = hwunknown;
		break;
	}
}
/* End Reading of Hardware Version ID */




/* The Main function */


int main(void)
{
	joystick.gFFBDevice.waitLastSimpleMotion = false;
	/* HAL initializations */

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();
	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	SMPortSetMaster(false);
	MX_DMA_Init();
	MX_ADC1_Init();
	//MX_ADC2_Init();
	MX_USART3_UART_Init();
	MX_USART1_UART_Init();
	MX_I2C1_Init();

#if 0
	while(true) {
		asm("nop");
	}
#endif


	// start FFB 2500Hz timer
	FFB2500HzWait = false;
	MX_TIM4_Init();
	MX_TIM5_Init();
	MX_TIM6_Init();
	HAL_TIM_Base_Start_IT(&htim4_effecttimer);
	HAL_TIM_Base_Start(&htim6_1MhzEffectClock);
	//HAL_TIM_Base_Start(&htim4_effecttimer);
	//HAL_TIM_Base_Start_IT(&htim5_usbsuspenddelay);
	//while(1);

	/* Initialize PWM&DIR outputs to LOW, so that "default" Ioni configs, that take
	 * those kind of signals, don't try to interpret anything....
	 */
	HAL_GPIO_WritePin(STM_IONI_PWM_OUT_HSIN2_GPIO_Port, STM_IONI_PWM_OUT_HSIN2_Pin, GPIO_PIN_RESET); // sets pin low
	HAL_GPIO_WritePin(STM_IONI_DIR_OUT_HSIN1_GPIO_Port, STM_IONI_DIR_OUT_HSIN1_Pin, GPIO_PIN_RESET); // sets pin low

	// start all simucube ADC DMA traffics
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1_Buffer, ADC1_BUFFER_LENGTH);
	//HAL_ADC_Start_DMA(&hadc2, (uint32_t*)ADC2_Buffer, ADC2_BUFFER_LENGTH);

	// start simplemotion SMBUS reception
	HAL_UART_Receive_DMA(&huart3,rxBuffer, RXBUFSIZE);
	//joystick.gFFBDevice.mSMBusHandle = smOpenBus("MBEDSERIAL");

	MX_USB_DEVICE_Init();
	//  read hw version here
	readHWVersion();

#if 0
	char data[11];
	for(int i=0; i<11; i++) {
		data[i]=i;
	}
	volatile HAL_StatusTypeDef status;

	for(int i=0; i<10; i++) {
		status= HAL_UART_Transmit_IT(&huart3, (uint8_t*)data[i], 1);
		HAL_Delay(10);
	}
	while(1);
#endif

	if(debugMode) printf("------- SimuCUBE BOOT -------\r\n");
	if(debugMode) printf("HAL initializations complete.\r\n");
	simucubelog.addEventParam(SimuCUBEBoot, majorVersion, true);
	simucubelog.addEventParam(SimuCUBEBoot, minorVersion, true);
	simucubelog.addEventParam(SimuCUBEBoot, buildVersion, true);



	/* init firmware version table  */
	firmwareVersion[majorVersionIdx] = majorVersion;
	firmwareVersion[minorVersionIdx] = minorVersion;
	firmwareVersion[buildVersionIdx] = buildVersion;

	/* state machine startup initializations */
	if(debugMode) printf("Trying to load config from flash..\r\n");

	// load settings from flash
	bool loadflash = joystick.gFFBDevice.loadConfigsFromFlash();
	if(debugMode) printf("Done trying to load config from flash\r\n");
	simucubelog.addEvent(loadedFlash, true);


	HAL_GPIO_WritePin(LED3_OUT_GPIO_Port, LED3_OUT_Pin, GPIO_PIN_SET);

	if(!loadflash) {
		if(debugMode) printf("flash data was for old version. Saving default configs to flash!\r\n");
		simucubelog.addEvent(flashDataOld, true);
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
		if(joystick.gFFBDevice.rewriteDefault == true) {
			if(debugMode) printf("rewriting profile 0\r\n");
			simucubelog.addEvent(rewriteDefaultProfile, true);
			// need to rewrite default profile
			joystick.gFFBDevice.mConfig.setReadOnlyDefaultProfileValues();
			//disableSMWatchdog();
			if(!joystick.gFFBDevice.saveConfigsToFlash()) {
				// failure -> currentSystemStatus = FlashFault;
				//enableSMWatchdog();
				currentSystemStatus = FlashFault;
				if(debugMode)printf("rewriting profile 0 failed\r\n");
			}
			joystick.gFFBDevice.unsavedSettings = 0;
			//enableSMWatchdog();
		}

		if(joystick.gFFBDevice.mConfig.hardwareConfig.mInitialConfigDone == InitialConfigDone)  {
			if(debugMode) printf("Initial configuration has been performed. Initializing drive.\r\n");
			simucubelog.addEvent(EventInitialConfigDone, true);
			currentSystemStatus = DriveInit;
		} else {
			currentSystemStatus = SystemNotConfigured;
			if(debugMode) printf("Initial configuration has not been performed. Drive not operational.\r\n");
			simucubelog.addEvent(EventInitialConfigNotDone, true);
			// connect drive also, so that it will automatically update firmware on it.
			if(!InitializeDrive(true)) {
				// go to IoniFWUpgradeError
				currentSystemStatus = IoniFWUpgradeError;
			} else {
				currentSystemStatus = SystemNotConfigured;
			}
			//SMPortSetMaster(false);
			//disableSMWatchdog;
		}
	}

	joystick.gFFBDevice.IRFFBModeEnabled = false;

	int reconnectCounter = 0;


	/* end state machine startup initializations */



	// variable needed for steering calculation
	uint16_t steeringValue=0;// = 32768;


	// for cleanup of debug printing:
	//SystemStatus previousSystemStatus = Internal_ForceMyEnumIntSize;
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
				enableSMWatchdog();
				bool hidcommand = handleHidReports();
				if(!hidcommand) break; // got a hid report already telling to do something else
				if(joystick.gFFBDevice.mConfig.hardwareConfig.mIndexingMode == indexPointIndexing) {
					currentSystemStatus = WaitFindingIndexPoint;
				} else {
					if(joystick.gFFBDevice.mConfig.hardwareConfig.mInitialConfigDone != InitialConfigDone) {
						currentSystemStatus = DriveInitSuccessPause;
					} else {
						currentSystemStatus = BeforeOperational;
					}
				}
				break;
			}

			case WaitFindingIndexPoint:
			{
				joystick.gFFBDevice.indexpointfound = 0;
				if(debugMode && statusChanged) printf("state: Waiting for index point \r\n");
				if(statusChanged) simucubelog.addEvent(StateWaitingIndex);
				int indexpoint = 0;
				bool status = WaitForIndexPulse(indexpoint);
				if(!status) {
					// failure,or was told to go elsewhere
					break;
				}
				joystick.gFFBDevice.indexpointfound = 1;
				volatile int p1,p2;
				smint32 positionFB=0;
				p1=SetTorque(0);//call this twice to have 16 bit differential encoder unwrapper initialized
				p2=SetTorque(0);
				smRead1Parameter(joystick.gFFBDevice.mSMBusHandle, 1, SMP_ACTUAL_POSITION_FB_NEVER_RESETTING, &positionFB);//read 32 bit position
				joystick.gFFBDevice.indexPointEncPos = positionFB;
				resetPositionCountAt(positionFB);
				//resetPositionCountAt(0);

				if(joystick.gFFBDevice.mConfig.hardwareConfig.mInitialConfigDone == InitialConfigDone) {
					currentSystemStatus = BeforeOperational;
				} else {
					currentSystemStatus = DriveInitSuccessPause;
				}
				break;
			}
			case DriveInitSuccessPause:
			{
				if(debugMode && statusChanged) printf("state: DriveInitSuccessPause \r\n");
				if(statusChanged) simucubelog.addEvent(StateDriveInitSuccessPause);
				handleHidReports();
				SetTorque(0); // to keep watchdog happy
				HAL_Delay(1); // to not overflow IONI simplemotion task.
				break;
			}
			case BeforeOperational:
				if(debugMode && statusChanged) printf("state: BeforeOperational \r\n");
				if(statusChanged) simucubelog.addEvent(StateBeforeOperational);
				//handleHidReports();
				joystick.gFFBDevice.initVariables(); // this needs to be done to get from encoder resolution to real angle

				// set ioni filters
				writeChangedIoniParameters(true);
				if(!joystick.gFFBDevice.readEncoder32bit()) {
					if(debugMode) {
						printf("couldn't read 32bit enc pos\r\n");
					}
				}
				currentSystemStatus = Operational;
				break;













			/* main operating state! */
			case Operational:
			{
				if(debugMode && statusChanged){
					printf("state: Operational \r\n");
				}
				if(joystick.gFFBDevice.mConfig.hardwareConfig.mInitialConfigDone!=InitialConfigDone) {
					currentSystemStatus = SystemNotConfigured;
					break;
				}
				if(statusChanged) {
					simucubelog.addEvent(StateOperational);
					joystick.gFFBDevice.prevEffectCalcTime = -1;
				}
				bool newCalculation = joystick.gFFBDevice.CalcTorqueCommand(&encoderCounter); //reads encoder counter too
				if(newCalculation) {
					steeringValue=joystick.gFFBDevice.calcSteeringAngle(encoderCounter);
				}
				joystick.update(steeringValue); // updates also everything else (buttons)
				handleHidReports();
				break;
			}
			/* end main operating state! */










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
				enableSMWatchdog();

				// save also flags that are set to drive.
				resetMaxMMC();
				saveDriveCfg();
				setCurrentProfileMMC();

				if(!joystick.gFFBDevice.readEncoder32bit()) {
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
				// todo:
				// disable drive
				// reset all configs to default (done)
				// include ioni reset?
				joystick.gFFBDevice.SetDefault();
				joystick.gFFBDevice.mConfig.hardwareConfig.mInitialConfigDone = InitialConfigNotDone;
				if(!joystick.gFFBDevice.saveConfigsToFlash()) {
					currentSystemStatus = FlashFault;
				} else {
					// reboot MCU here
					NVIC_SystemReset();
					while(1) {
						asm("nop");
					}
				}
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

					//disableSMWatchdog();
				} // do not try to access smbus if simple watchdog command failed.
				else {
					initDefaultSMBusBaudrate(true);
				}
				SMPortSetMaster(false);
				currentSystemStatus = SMBusReleased;
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
					currentSystemStatus=DriveConnectionError;
				}
				if(millis > (timervalue+3000)) {
					reconnectCounter++;
					SMPortSetMaster(true);
					int status = initSMBusBaudrate();
					if(status == -1) {
						if(debugMode) printf("couldn't initialize smbus baud rate!\r\n");
						simucubelog.addEvent(StateRegainSMFailed);
						SMPortSetMaster(false);
						timervalue=millis;
						break;
					}
					//joystick.gFFBDevice.readIoniParameters();
					//joystick.gFFBDevice.writeChangedIoniParameters(true);

					if(joystick.gFFBDevice.mConfig.hardwareConfig.mInitialConfigDone == InitialConfigNotDone) {
						if(!InitializeDrive(true)) { // to make sure still have correct FW version
							// go to IoniFWUpgradeError?
							currentSystemStatus = SystemNotConfigured;
							break;
						}
						currentSystemStatus = SystemNotConfigured;
						break;
					} else {
						enableSMWatchdog();
						currentSystemStatus = nextSystemStatus;
						if(!InitializeDrive()) {
							currentSystemStatus = DriveConnectionError;
							break;
						}
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
						if(initSMBusBaudrate()==0) {
							//success. Return to the state where system was last.
							if(joystick.gFFBDevice.mConfig.hardwareConfig.mInitialConfigDone==InitialConfigDone) {
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
				int skippedcount = 0;
				int errorcount = 0;
				unsigned int mode;
				mode = (CONFIGMODE_DISABLE_DURING_CONFIG | CONFIGMODE_CLEAR_FAULTS_AFTER_CONFIG);

				printf("\r\n--\r\n");
				for(int i = 0; i<joystick.gFFBDevice.IoniDrcDataReceivedBytes; i++) {
					printf("%c",IoniDrcFile[i]);
				}
				printf("--\r\n");
				LoadConfigurationStatus status = smLoadConfigurationFromBuffer(joystick.gFFBDevice.mSMBusHandle, 1, &IoniDrcFile[0], joystick.gFFBDevice.IoniDrcDataReceivedBytes, mode, &skippedcount, &errorcount);
				// todo: error checking?
				if(status != CFGComplete) {
					// failed?
					if(debugMode && statusChanged) printf("ApplyIoniDrcData ERRORED\r\n");
					simucubelog.addEvent(ApplyDriveDRCError, (uint32_t)status);
				} else {
					if(debugMode && statusChanged) printf("ApplyIoniDrcData SUCCESS\r\n");
					simucubelog.addEvent(ApplyDriveDRCSuccess);
				}
				enableSMWatchdog();

				// after applying DRC, read MMC and other important parameters
				smint32 res;
				smRead1Parameter(joystick.gFFBDevice.mSMBusHandle, 1, SMP_TORQUELIMIT_CONT, &res);
				joystick.gFFBDevice.mConfig.hardwareConfig.mMaxMotorCurrent = res;
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
				simucube_bootloader();
				while(1) {
					asm("nop");
				}
				break;

			case AutoDetectCommutationSensors:
            {
            	simucubelog.addEvent(StateAutoSetupCommucation, true);
            	handleHidReports();
            	disableSMWatchdog();
                if(joystick.gFFBDevice.encoderType != 1) {
                	joystick.gFFBDevice.mConfig.hardwareConfig.autoCommutationStatus=FailedUnsupportedEncoder;
                	printf("wrong encodertype\r\n");
                    currentSystemStatus=previousSystemStatus2;
                    simucubelog.addEventParam(AutoCommutationFault, 1, true);
                	break;
                }
                if((previousSystemStatus2 != Operational) && (previousSystemStatus2 != DriveInitSuccessPause)) {
                	joystick.gFFBDevice.mConfig.hardwareConfig.autoCommutationStatus=FailedNotInitialized;
                    currentSystemStatus=previousSystemStatus2;
                    simucubelog.addEventParam(AutoCommutationFault, 2, true);
                    printf("was not operational\r\n");
                    break;
                }
                resetMaxMMC();
                joystick.gFFBDevice.mConfig.hardwareConfig.autoCommutationStatus = Started;
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
							joystick.gFFBDevice.mConfig.hardwareConfig.autoCommutationStatus=FailedUninitialized;
							abort=true;
							printf("fault_torq\r\n");
							simucubelog.addEventParam(AutoCommutationFault, 3, true);
							break;
						case 400200://no torque
							joystick.gFFBDevice.mConfig.hardwareConfig.autoCommutationStatus=FailedLowCurrent1;
							simucubelog.addEventParam(AutoCommutationFault, 4, true);
							printf("fault_lowcurrent\r\n");
							abort=true;
						   break;
						case 400210://
							joystick.gFFBDevice.mConfig.hardwareConfig.autoCommutationStatus=FailedNoisyBadHall;
							simucubelog.addEventParam(AutoCommutationFault, 5, true);
							printf("fault_noisebad\r\n");
							abort=true;
							break;
						case 400220://
							joystick.gFFBDevice.mConfig.hardwareConfig.autoCommutationStatus=FailedInvalidHallSequence;
							simucubelog.addEventParam(AutoCommutationFault, 6, true);
							printf("fault_invalidseq\r\n");
							abort=true;
							break;
						case 400400://
							joystick.gFFBDevice.mConfig.hardwareConfig.autoCommutationStatus=FailedLowCurrent2;
							simucubelog.addEventParam(AutoCommutationFault, 7, true);
							printf("fault_lowcurrent2\r\n");
							abort=true;
							break;
						case 400999://ok
							joystick.gFFBDevice.mConfig.hardwareConfig.autoCommutationStatus=Success;
							joystick.gFFBDevice.mConfig.hardwareConfig.mAutoCommutationMode=1;
							saveDriveCfg();
							simucubelog.addEvent(AutoCommutationSuccess, true);
							printf("success\r\n");
							joystick.gFFBDevice.unsavedSettings=1;
							abort=true;
							break;
						default://wtf
							joystick.gFFBDevice.mConfig.hardwareConfig.autoCommutationStatus=UnknownResult;
							simucubelog.addEventParam(AutoCommutationFault, 10, true);
							printf("??\r\n");
							abort=true;
							break;
                    }
                }
                setCurrentProfileMMC();
                if(elapsedtime_ms>=timelimit)
                {
                	joystick.gFFBDevice.mConfig.hardwareConfig.autoCommutationStatus=Timeout;
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
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

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

extern bool waitingWatchDog;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM5) {
		HAL_TIM_Base_Stop_IT(&htim5_usbsuspenddelay);
		usbsuspended=0;
		HAL_GPIO_TogglePin(LED3_OUT_GPIO_Port, LED3_OUT_Pin);
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
		joystick.gFFBDevice.FFB2500HzWait = false;
	}
}




/* ADC1 init function */
static void MX_ADC1_Init(void)
{

	ADC_ChannelConfTypeDef sConfig;

	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	*/
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_LEFT;
	hadc1.Init.NbrOfConversion = NUM_ADC_CHANNELS;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
	Error_Handler();
	}

	// X11 UPPER

	// x11 upper 1
	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}

	// x11 upper 2
	sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = 2;
	sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}
	// x11 upper 3
	sConfig.Channel = ADC_CHANNEL_15;
	sConfig.Rank = 3;
	sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}
	// x11 upper 5
	sConfig.Channel = ADC_CHANNEL_14;
	sConfig.Rank = 4;
	sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}
	// x11 upper 6
	sConfig.Channel = ADC_CHANNEL_7;
	sConfig.Rank = 5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}


	// X11 LOWER

	// x11 lower 2
	sConfig.Channel = ADC_CHANNEL_13;
	sConfig.Rank = 6;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}
	// x11 lower 3
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = 7;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}
	// x11 lower 5
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = 8;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}
	// x11 lower 6
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = 9;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}
	// x11 lower 7
	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = 10;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}


	// EXT POT

	// EXT POT 1
	sConfig.Channel = ADC_CHANNEL_12;
	sConfig.Rank = 11;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}

	// EXT POT 2
	sConfig.Channel = ADC_CHANNEL_11;
	sConfig.Rank = 12;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}

	// EXT POT 3
	sConfig.Channel = ADC_CHANNEL_10;
	sConfig.Rank = 13;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

}




/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 230400;//115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
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




/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PB6   ------> I2C1_SCL
     PB7   ------> I2C1_SDA
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : X12_LOWER_1_Pin X12_LOWER_2_Pin X12_LOWER_4_Pin X12_UPPER_6_Pin 
                           STM_PE6_Pin STM_PE7_Pin STM_PE8_Pin STM_PE10_Pin 
                           STM_PE13_Pin STM_PE14_Pin STM_PE15_Pin X15_3_Pin 
                           X12_LOWER_3_Pin */
  GPIO_InitStruct.Pin = X12_LOWER_1_Pin|X12_LOWER_2_Pin|X12_LOWER_4_Pin|X12_UPPER_6_Pin 
                          |STM_PE6_Pin|STM_PE7_Pin|STM_PE8_Pin|STM_PE10_Pin 
                          |STM_PE13_Pin|STM_PE14_Pin|STM_PE15_Pin|X15_3_Pin 
                          |X12_LOWER_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : X12_LOWER_6_Pin X12_LOWER_7_Pin X11_LOWER_1_Pin X12_UPPER_2_Pin 
                           X12_UPPER_3_Pin X12_UPPER_4_Pin X12_UPPER_5_Pin X12_UPPER_7_Pin 
                           X12_UPPER_1_Pin X12_LOWER_5_Pin */
  GPIO_InitStruct.Pin = X12_LOWER_6_Pin|X12_LOWER_7_Pin|X11_LOWER_1_Pin|X12_UPPER_2_Pin 
                          |X12_UPPER_3_Pin|X12_UPPER_4_Pin|X12_UPPER_5_Pin|X12_UPPER_7_Pin 
                          |X12_UPPER_1_Pin|X12_LOWER_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : STM_ENC_IN_A_Pin STM_ENC_IN_B_Pin STM_ENC_IN_C_Pin PA8 */
  GPIO_InitStruct.Pin = STM_ENC_IN_A_Pin|STM_ENC_IN_B_Pin|STM_ENC_IN_C_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : GP01_STM_Pin PB12 PB13 PB14 
                           PB15 DIPSW_2_Pin STM_PB9_Pin */
  GPIO_InitStruct.Pin = GP01_STM_Pin|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15|DIPSW_2_Pin|STM_PB9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : STM_IONI_PWM_OUT_HSIN2_Pin STM_IONI_DIR_OUT_HSIN1_Pin  */
  GPIO_InitStruct.Pin = STM_IONI_PWM_OUT_HSIN2_Pin|STM_IONI_DIR_OUT_HSIN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : HX711_CLKOUT_Pin */
  GPIO_InitStruct.Pin = HX711_CLKOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : RS485_TXEN_STM_Pin LED4_OUT_Pin LED3_OUT_Pin LED2_OUT_Pin 
                           LED1_CLIPPING_OUT_Pin */
  GPIO_InitStruct.Pin = RS485_TXEN_STM_Pin|LED4_OUT_Pin|LED3_OUT_Pin|LED2_OUT_Pin 
                          |LED1_CLIPPING_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : STM_PD9_Pin STM_PD10_Pin STM_PD11_Pin IDSEL_0_Pin 
                           IDSEL_1_Pin IDSEL_2_Pin FTDI_USB_SLEEP_IN_Pin USB_GRANITY_VCC_SENSE_Pin 
                           USB_HID_VCC_SENSE_Pin STM_PD6_Pin STM_PD7_Pin */
  GPIO_InitStruct.Pin = STM_PD9_Pin|STM_PD10_Pin|STM_PD11_Pin|
                          FTDI_USB_SLEEP_IN_Pin|USB_GRANITY_VCC_SENSE_Pin
                          |USB_HID_VCC_SENSE_Pin|STM_PD6_Pin|STM_PD7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* Configure GPIO IDSEL pins with Internal Pull-up */
  GPIO_InitStruct.Pin = IDSEL_0_Pin | IDSEL_1_Pin | IDSEL_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : GPI4_CLEAR_FAULTS_Pin PB5 */
  GPIO_InitStruct.Pin = GPI4_CLEAR_FAULTS_Pin|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, STM_IONI_PWM_OUT_HSIN2_Pin|STM_IONI_DIR_OUT_HSIN1_Pin|HX711_CLKOUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RS485_TXEN_STM_Pin|LED4_OUT_Pin|LED3_OUT_Pin|LED2_OUT_Pin 
                          |LED1_CLIPPING_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPI4_CLEAR_FAULTS_Pin|GPIO_PIN_5, GPIO_PIN_RESET);

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


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

#ifndef HW_FUNCTIONS_H_
#define HW_FUNCTIONS_H_

/* Uncomment this if you have special modified SimuCUBE that can output UART4 debug */
// #define UART4DEBUG 1

#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */

#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Base @ of Sector     8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base @ of Sector     9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base @ of Sector     10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base @ of Sector     11, 128 Kbytes */


void readHwVersion();
void setPWMDIRPinslow();
void testAndInitBLE();
void testBLE();
void initBLEUART();
void resetUSART1vars();
void MX_USART1_UART_Init_Debug();
void MX_UART4_Init_Debug(void);

void MX_ADC1_Init(void);
void MX_GPIO_Init(void);
int init_test();
void resetSettings();
void postInitDrive();

void setSTO(int reason);
void disableSTO();


void configureSystem();
#ifdef __cplusplus
extern "C" {
#endif
void printfTransmit(char *ptr);
#ifdef __cplusplus
}
#endif

#define SETTINGSSTARTADDRESS ADDR_FLASH_SECTOR_7
#define HWSETTINGSADDRESS SETTINGSSTARTADDRESS+16
#define ANALOGSETTINGSADDRESS HWSETTINGSADDRESS+1024
#define BUTTONSSETTINGSADDRESS ANALOGSETTINGSADDRESS+8192
#define PROFILESETTINGSADDRESS BUTTONSSETTINGSADDRESS+1024
#define SETTINGSENDADDRESS ADDR_FLASH_SECTOR_8

#endif /* HW_FUNCTIONS_H_ */

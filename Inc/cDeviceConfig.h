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
 *
 *
 *
 * ---------------------------------------------------------------------------
*/

/*
 * cDeviceConfig.h
 *
 *  Created on: Jan 30, 2017
 *      Author: Mika
 */

#ifndef CDEVICECONFIG_H_
#define CDEVICECONFIG_H_

#include "cHardwareConfig.h"
#include "cProfileConfig.h"
#include "cAnalogConfig.h"
#include <cstring>

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

class cDeviceConfig {
public:
	cDeviceConfig();
	virtual ~cDeviceConfig();

	void SetDefault();

	void setReadOnlyDefaultProfileValues();
	cHardwareConfig hardwareConfig;
	cAnalogConfig analogConfig;
	cProfileConfig profileConfigs[maxnumprofiles];


	uint8_t * GetProfileConfigAddr(uint16_t profileindex);
	uint8_t * GetHardwareConfigAddr();
	uint8_t * GetAnalogConfigAddr();

	void getProfile(int *conf, uint16_t profileindex);
	void setProfile(int *conf, uint16_t profileindex);
	void getHardware(int *conf);
	void setHardware(int *conf);
	void getAnalog(int *conf);
	void setAnalog(int *conf);
private:


} __attribute__((packed));

#endif /* CDEVICECONFIG_H_ */

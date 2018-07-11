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

#ifndef __WRAPPERS_H
#define __WRAPPERS_H
/*
 * wrappers.cpp
 *
 * This file includes all the callbacks to SimuCUBE c++ functions/classes from the
 * STM HAL C code.
 */

#include "stdint.h"
/* some callback wrappers from C USB HAL code. */
extern "C" {

uint8_t gamecontroller_callback_wrapper(uint8_t *report);

void FfbOnCreateNewEffect_wrapper();

void send_mSetReportAnswer_wrapper();
void send_mGetReportAnswer_wrapper(uint8_t report_id);
void updateButtons_wrapper();
void simucubelog_addevent_wrapper(uint16_t event, bool forced);
void simucubelog_addeventParam_wrapper(uint16_t event, int32_t parameter, bool forced);
uint32_t getVariousSettingsBits();
}


#endif

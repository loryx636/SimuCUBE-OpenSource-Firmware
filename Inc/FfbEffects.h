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

#ifndef FFBEFFECTS_H_
#define FFBEFFECTS_H_

#define TM_PROCESS_STOP_MS 20

#include <cstdint>
#include "FfbEngine.h" // cEffectState

class Biquad1StOrder;

// effect calculations
float constantForceEffect(volatile cEffectState* effect);
float squareEffect(volatile cEffectState* effect, uint8_t gain);
float sineEffect(volatile cEffectState* effect, uint8_t gain);
float triangleEffect(volatile cEffectState* effect, uint8_t gain);
float sawtoothDownEffect(volatile cEffectState* effect, uint8_t gain);
float sawtoothUpEffect(volatile cEffectState* effect, uint8_t gain);
float springEffect(volatile cEffectState* effect, uint8_t gain);
float frictionEffect(int32_t readencoderpos, uint32_t cpr, volatile cEffectState* effect, uint8_t gain);
float desktopSpringEffect(uint8_t gain);
float calcDampingEffect(uint8_t gain, float speed, volatile cEffectState *effect, Biquad1StOrder& lpf);
float irFFBEffects();


#endif /* FFBEFFECTS_H_ */

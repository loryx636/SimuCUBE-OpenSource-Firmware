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
 * FfbEffects.h
 *
 *  Created on: 11.5.2018
 *      Author: mikat
 */

#ifndef FFBEFFECTS_H_
#define FFBEFFECTS_H_


	// effect calculations
	float constantForceEffect(volatile cEffectState* effect);
	float squareEffect(volatile cEffectState* effect, uint8_t gain);
	float sineEffect(volatile cEffectState* effect, uint8_t gain);
	float triangleEffect(volatile cEffectState* effect, uint8_t gain);
	float sawtoothDownEffect(volatile cEffectState* effect, uint8_t gain);
	float sawtoothUpEffect(volatile cEffectState* effect, uint8_t gain);
	float springEffect(volatile cEffectState* effect, uint8_t gain);
	float frictionEffect(s32 *readencoderpos, float cntsperinverse, uint32_t cpr, volatile cEffectState* effect, uint8_t gain);
	float calcDamperEffectGain(volatile cEffectState* effect, uint8_t gain);
	float calcDampingEffect(float gain, volatile cEffectState *effect);
	float desktopSpringEffect(uint8_t saturation, uint8_t gain);
	float endstopEffect(float speed);
	float irFFBEffects();


#endif /* FFBEFFECTS_H_ */

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

#ifndef ENDSTOPEFFECT_H_
#define ENDSTOPEFFECT_H_

#include "Biquad1storder.h"
#include "encoderangle.h"
#include <cmath>

struct endstopSetting {
    int32_t damperGain; // based on testing
    int32_t rawMaxTorque; // = 100
    int32_t rangeDegrees; // tuned to suitable range (soft, medium, hard)
};

/**
 * Endstop effect or the bumpstop effect can be configured with profile and hardware settings.
 * It appears as an obstacle when the player tries to go beyond the configured steering range.
 */
class EndstopEffect {
private:
    const endstopSetting endstopSettings[3] = {
            {30, 100, 35},
            {40, 100, 17},
            {50, 100, 10}
    };

    const int _numEndstopSettings = sizeof(endstopSettings)/sizeof(endstopSetting);
    const float _deadBand = 20.0f;
	bool _enabled;
	float _configuredGain;
	float _maxTorque;

	// range is the actual ramp angle length where the endstop will go from 0 to 100%
	float _range;

	float _maxAngle;
	float _offset;

	int _direction; // -1 for over left, 0 inside range, 1 for over right
	float _distance; // positive distance to nearest endstop (left or right)

	Biquad1StOrder lpf;

	// debugging aide directly from profile settings, might be useful if the sign is wrong
	int32_t _options;

	// not the full endstop, lacks maxEndTorque; in the direction of endstop
	float _endstop;
	// not the full damping, lacks maxEndTorque; in the direction of speed
	float _damping;

	// this turns ON if user configures bumbstop that would cause immediate torque.
	bool _unsafe;

    // Called when the effect is needed (every cycle)
    float torque(float speed);

    //test whether effect is safe to start without causing torque right away
    void testSafeStart(encoderAngle* angle);

public:
    /** @brief Constructor and default destructor
     */
	EndstopEffect() : _enabled(false), _direction(0), _distance(0.0f), _options(0), _endstop(0.0f), _unsafe(true) {}
	~EndstopEffect() = default;

	/** @brief Returns closest endstop direction
	 *
	 * @return -1 when closest bumpstop is the left one, 1 when the right one, zero when neither.
	 */
	int direction() const {
		return _direction;
	}

	/** @brief distance from endstop
	 *
	 * @return returns distance from closest endstop limit as degrees in float
	 */
	float distance() const {
		return _distance;
	}

	/** @brief reset endstop parameters and internal safety flags when
	 *
	 * @param angle: pointer to encoderAngle object to use to query positions
	 */
	void position_reset(encoderAngle* angle) {
		update(0.0f, angle);
	}

    /** @brief reset endstop parameters and internal safety flags when user changes parameters
     * 	       Called when settings have changed with new values (user configures)
     * @param enabled: is endstop enabled (usually true)
     * @param rawOffset: offset from maximum steering angle where endstop limit is to be
     * @param newBumpstopSetting: index for new endstop setting parameters in internal table
     * @param maxAngle: maximum steering angle in use
     * @param fc: lowpass filter rate for endstop effect
     * @param angle: pointer to encoderAngle object to use to query positions
     */
	void refresh(bool enabled, int32_t rawOffset, int32_t newEndstopSetting, float maxAngle, float fc, encoderAngle* angle);

	/** @brief this is called every cycle to update internal parameters
	 * @param steering_angle: current steerign angle
	 * @param angle: pointer for encoderAngle object to ask more details if needed
	 */
	void update(float steeringAngle, encoderAngle* angle);

    // true if endstop effect is in the unsafe region right after settings have been changed.
    /** @brief returns true if endstop is unsafe to be started now (would cause 100% torque)
     * @return true if unsafe
     */
    bool unsafe() const;

    /** @brief Calculate endstop torque and insert them to existing torque signals.
     * @param speed: axis speed in degrees/ms
     * @param filteredTorque: torque signal that will go to filtering
     * @param directTorque: torque signal that contains sine and other effects
     */
    void totalOut(float speed, float &filteredTorque, float &directTorque);
};


#endif /* ENDSTOPEFFECT_H_ */

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

#include "EndstopEffect.h"

#define ENDSTOPDEBUG 0
#if ENDSTOPDEBUG
#define endstopprintf(...) printf(__VA_ARGS__);
#else
#define endstopprintf(...)
#endif


// Called when settings have changed with new values (user configures)
void EndstopEffect::refresh(bool enabled, int32_t rawOffset, int32_t newEndstopSetting, float maxAngle, float fc, encoderAngle* angle) {
    _enabled = enabled;

    int32_t index = newEndstopSetting;

    // safety if previous version has problematic parameters
    if(index > (_numEndstopSettings - 1)) {
        index = _numEndstopSettings-1;
    }
    if(index < 0) {
    	index = 0;
    }

    endstopSetting newSetting = endstopSettings[index];
    int32_t raw_gain = newSetting.damperGain;
    int32_t raw_max_torque = newSetting.rawMaxTorque;
    int32_t range_degrees = newSetting.rangeDegrees;
    endstopprintf("new settings: index %d,  torq %d, damper %d, range %d\r\n", index, raw_max_torque, raw_gain, range_degrees);


    _configuredGain = ((float)raw_gain) / 100.0f;
    _maxTorque = (float)raw_max_torque * 100.0f * 255.0f;

    _maxAngle = maxAngle/2;
    _offset = (float) rawOffset;

    if(range_degrees <= 0) {
        // protect against dividing by zero with this. this might need to be longer.
        _range = 0.001f;
    } else {
        _range = (float)range_degrees;
    }

    lpf.setBiquad(Biquad1StOrder::bq_type_lowpass_1st_order, fc);

    testSafeStart(angle);
}

// Called when unlimited steering angle has updated (every cycle)
void EndstopEffect::update(float steeringAngle, encoderAngle* angle) {
    // if the offset is by a bug larger than (max_angle / 2.0) this will always go to zero, and the direction
    // is selected by the sign so this just works (tm). this does require max_angle to be sensible. there
    // has been discussions of max_angle > 90
    _distance = fmax(0.0f, (_maxAngle) - _offset - fabs(steeringAngle));

    if (_distance >= _range) {
        // too far away from ends
        _direction = 0;
    } else {
        // direction is used as a coefficient so fix it's sign. according to this, the force
        // is in the direction of the endstop which sounds wrong.
        _direction = steeringAngle > 0 ? 1 : -1;
    }

    if(_unsafe) {
        testSafeStart(angle);
    }
}

//test whether effect is safe to start without causing torque right away
void EndstopEffect::testSafeStart(encoderAngle* angle) {
    float currentAngle = abs(angle->getUnlimitedSteeringAngle());
    float margin = _maxAngle - abs(angle->getUnlimitedSteeringAngle()) - _offset - _range - _deadBand;

    // warning: these float prints are very slow when enabled, causes true drive disconnects.
    if(margin<0) {
        _unsafe = true;
        endstopprintf("ENDSTOP: unsafe! margin = %f - %f - %f - %f = %f\r\n", _maxAngle/2.0, currentAngle, _offset, _range, margin);
    } else{
        endstopprintf("ENDSTOP: safe to start,  margin = %f - %f - %f - %f = %f\r\n", _maxAngle/2.0, currentAngle, _offset, _range, margin);
        _unsafe = false;
    }
}

// Called when the effect is needed (every cycle)
float EndstopEffect::torque(float speed) {
    if (!_enabled) {
        return 0.0f;
    }

    if(_unsafe) {
        return 0.0f;
    }

    if (_distance >= _range) {
        // this is probably to avoid hard cutoff from the effect
        return lpf.process(0.0f);
    }

    _endstop = (float)_direction * fmax(0.0f, fmin(1.0f, (_range - _distance)/_range));

    // 80 is because the original version used angle difference instead of time. the speed is based
    // on axis count (0..65535) difference per encoder read timestamps.
    // lower divisors cause audible noise and oscillation especially on ultimate with full power.
    _damping = (speed / 80.0f) * _configuredGain;

    // note that this function is further filtered by a jungle of if-elses in CalcTorqueCommand
    return lpf.process(_maxTorque * (_endstop + _damping));
}

// returns true if endstop is in unsafe state
bool EndstopEffect::unsafe() const {
    return _unsafe;
}

void EndstopEffect::totalOut(float speed, float &filteredTorque, float &directTorque) {
    // assumption that seems to be valid for every realistic situation:
    // directTorque (effects) is at lower scale than the main filteredTorque signal.

    const float endstopTorque = torque(speed);
    if ((endstopTorque < 0.0f) == (filteredTorque < 0.0f)) {
        // both are to the same direction, pick the greatest
        if (endstopTorque < 0.0f) {
            filteredTorque = fmin(filteredTorque, endstopTorque);
        } else {
            filteredTorque = fmax(filteredTorque, endstopTorque);
        }
    } else {
        // different direction. scale filteredTorque down and add the endstop torque
        filteredTorque = filteredTorque * fmax(0.0f, fmin(1.0f, _distance / _range));
        filteredTorque += endstopTorque;
    }
    // scale effect torques (vibration, etc) so that they will be already
    // zero at half way of ramp
    // directTorque = directTorque * fmax(0.0f, fmin(1.0f, (_range - ((_range - _distance) * 2.0f)) / _range));
    // optimized:
    directTorque = directTorque * fmax(0.0f, fmin(1.0f, (_range * -1.0f + 2.0f * _distance ) / _range));
}

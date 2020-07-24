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

#include <eventLog.h>
#include "stm32f407xx.h"

extern uint64_t millis;

eventLog::eventLog() {
	// TODO Auto-generated constructor stub
	clearLog();
	enabled=true;
	lastEventIndex = logSize-1;
	verbosity = logVerbosityNormal;
	startTime=0;
}

eventLog::~eventLog() {
	// TODO Auto-generated destructor stub
}

void eventLog::clearLog() {
	__disable_irq();
	for(int i = 0; i<logSize; i++) {
		eventIDs[i]=0;
		eventParams[i]=0;
	}
	lastEventIndex=logSize-1;
	__enable_irq();
}

void eventLog::setVerbosity(uint8_t newverbosity) {
	verbosity = newverbosity;
}

uint8_t eventLog::getVerbosity() {
	return verbosity;
}

bool eventLog::testVerbosity(uint16_t event) {
	if((verbosity == logVerbosityNormal) && (event >= 300)) {
		return false;
	}
	else if(
		((verbosity == logVerbosityFFBEvents ) && (event<300)) ||
		((verbosity == logVerbosityFFBEvents ) && (event>=350))) {
		return false;
	}
	else if(
		((verbosity == logVerbosityFFBEffects) && (event<350))) {
		return false;
	}
	return true;
}

void eventLog::addEventParam(uint16_t event, int32_t parameter, bool forceAddEvent) {
	if(!enabled) {
		return;
	}
	if(!testVerbosity(event)) {
		return;
	}

	if(!forceAddEvent) {
		if(eventIDs[lastEventIndex]==event && eventParams[lastEventIndex]==parameter) {
			return;
		}
	}
	__disable_irq();
	if(lastEventIndex == (logSize-1) ) {
		lastEventIndex = 0;
	} else {
		lastEventIndex++;
	}
	eventIDs[lastEventIndex] = event;
	eventParams[lastEventIndex] = parameter;
	__enable_irq();
}

void eventLog::addEvent(uint16_t event, bool forceAddEvent) {
	addEventParam(event, 0, forceAddEvent);
}

void eventLog::resetLogStartTime() {
	startTime=millis;
}
void eventLog::addTimeStamp() {
	addEventParam(eventTimeStamp, (uint32_t)(millis-startTime), true);
}

void eventLog::getEvent(uint16_t eventIndex, uint16_t &event, int32_t &parameter) {
	event = eventIDs[eventIndex];
	parameter=eventParams[eventIndex];
}

void eventLog::setEnabled() {
	enabled=true;
}

void eventLog::setDisabled() {
	enabled=false;
}

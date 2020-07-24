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

#ifndef EVENTLOG_H_
#define EVENTLOG_H_

#include "config_comm_defines.h"

class eventLog {
public:
	eventLog();
	virtual ~eventLog();
	void clearLog();
	void addEventParam(uint16_t event, int32_t parameter=0, bool forceAddEvent=false);
	void addEvent(uint16_t event, bool forceAddEvent=false);
	void getEvent(uint16_t eventIndex, uint16_t &event, int32_t &parameter);
	void resetLogStartTime();
	void addTimeStamp();
	void setEnabled();
	void setDisabled();
	void setVerbosity(uint8_t newverbosity);
	uint8_t getVerbosity();
	uint16_t lastEventIndex;

private:
	uint16_t eventIDs[logSize];
	int32_t eventParams[logSize];
	uint8_t verbosity;

	bool enabled;
	bool testVerbosity(uint16_t event);
	uint64_t startTime;
};

#endif /* EVENTLOG_H_ */

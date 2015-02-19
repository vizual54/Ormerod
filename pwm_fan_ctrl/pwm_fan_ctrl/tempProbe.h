#pragma once

#include <OneWire.h>

#define SENSOR_NOT_READY 0
#define SENSOR_NOT_OK -1
#define SENSOR_READY 1

class TempProbe
{
	OneWire* _oneWire;
	uint8_t _sensorFound;
	byte _address[8];
	float _temperature;
	boolean _getAddress();
	boolean _updateTemp();
	void _startConversion();
	byte _type_s;
	unsigned long _last_sample;

public:
	TempProbe(OneWire* oneWire) { _oneWire = oneWire; /*_getAddress();*/ }
	bool init();
	void update();
	float getTemp();
	int8_t isReady();
};


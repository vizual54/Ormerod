#pragma once

#include <OneWire.h>

class TempProbe
{
	OneWire* _oneWire;
	byte _address[8];
	float _temperature;
	boolean _getAddress();
	boolean _updateTemp();
	void _startConversion();
	byte _type_s;
	unsigned long _last_sample;

public:
	TempProbe(OneWire* oneWire) { _oneWire = oneWire; _getAddress(); }
	void init();
	void update();
	float getTemp();
	bool isReady();
};


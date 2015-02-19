#include "tempProbe.h"

bool TempProbe::init()
{
	if (_getAddress() && _sensorFound != 0)
	{
		_oneWire->reset();
		_oneWire->skip();
		_oneWire->write(0x44);
		_last_sample = millis();
		return true;
	}
	return false;
}

int8_t TempProbe::isReady()
{
	if (_sensorFound)
	{
		if (millis() - _last_sample >= 750)
		{
			if (_updateTemp())
			{
				return SENSOR_READY;
			}
			else
			{
				return SENSOR_NOT_READY;
			}
		}
		else
		{
			return SENSOR_NOT_READY;
		}
	}
	return SENSOR_NOT_OK;
}

void TempProbe::update()
{
	_startConversion();
}

float TempProbe::getTemp()
{
	return _temperature;
}

void TempProbe::_startConversion()
{
	_oneWire->reset();
	_oneWire->select(_address);
	_oneWire->write(0x44);
}

boolean TempProbe::_getAddress()
{
	_sensorFound = _oneWire->search(_address);

	if (_sensorFound != 0)
	{
		if (OneWire::crc8(_address, 7) != _address[7])
		{
			return false;
		}

		if (_address[0] == 0x10)
		{
			_type_s = 1;
#ifdef DEBUG
			Serial.println("Tempsensor is type S.");
#endif
		}
		else
		{
			_type_s = 0;
		}

#ifdef DEBUG
		Serial.print("New sensor found at address: ");
		for (int i = 0; i < 6; i++)
		{
			Serial.print(_address[i], HEX);
			Serial.print(" ");
		}
		Serial.println(_address[7]);
#endif

		return true;
	}
	else
	{
#ifdef DEBUG
		Serial.println("No temp sensor found");
#endif
		return false;
	}
}

boolean TempProbe::_updateTemp()
{
	byte data[12];
	uint8_t res = _oneWire->reset();
	if (res == 0)
	{
		_sensorFound = 0;
		return false;
	}
	_oneWire->select(_address);
	_oneWire->write(0xBE);       // read scratchpad
	for (int i = 0; i < 9; i++)
	{
		data[i] = _oneWire->read();
	}

	if (OneWire::crc8(data, 8) != data[8])
	{
		return false;			// crc failed
	}
	
	int16_t raw = ((data[1] << 8) | data[0]);
	
	if (_type_s)
	{
		raw = raw << 3; // 9 bit resolution default
		if (data[7] == 0x10) 
		{
			raw = (raw & 0xFFF0) + 12 - data[6];  // "count remain" gives full 12 bit resolution
		}
	}
	else
	{
		byte cfg = (data[4] & 0x60);		// at lower res, the low bits are undefined, so let's zero them

		if (cfg == 0x00)					// 9 bit resolution, 93.75 ms
		{
			raw = raw & ~7;		
		}  
		else if (cfg == 0x20)				// 10 bit res, 187.5 ms
		{
			raw = raw & ~3;		
		}
		else if (cfg == 0x40)				// 11 bit res, 375 ms
		{
			raw = raw & ~1;
		}
		//// default is 12 bit resolution, 750 ms conversion time
	}

	_temperature = (float)raw / 16.0f;
	_last_sample = millis();
#ifdef DEBUG
	Serial.print("Temperature: ");
	Serial.println(_temperature);
#endif
	return true;
}
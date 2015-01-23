// Change to delayMicroseconds since delay() is affected by the interrupt
// see http://arduino.cc/en/Reference/attachInterrupt

#include <dht.h>

#define INTERRUPT_PIN0 0

const byte pushButton1_pin	= A0;
const byte pushButton2_pin	= A1;
const byte relay1_pin		= A2;
const byte relay2_pin		= A3;
const byte rpm_pin			= 2;
const byte pwm_pin			= 3;
const byte dht_pin			= 4;

uint16_t		rpc_min = 30;
uint16_t		rpc_max = 100;
uint16_t		rpc_mid = rpc_min + (rpc_max - rpc_min) / 2;
uint16_t		rpc_start = 50;
uint16_t		rpc_out = rpc_min;
volatile byte	half_revolutions;
uint16_t		rpm;
unsigned long	rpm_time_old;
double			current_temp;
uint16_t		max_temp = 40;
uint16_t		min_temp = 20;
enum			mode {start, min, max, user, high_temp, low_temp};
mode			current_mode;
mode			last_mode;
dht				DHT;
uint16_t		ms_per_ramp_step = 43;
unsigned long   temp_time_old;

void set_duty_cycle(uint16_t cycle)
{
	/*Serial.print("cycle: ");
	Serial.println(cycle);*/
	int value = (int)((float)cycle / (float)rpc_max * (float)79);
	OCR2B = value; //(uint8_t)((float)(cycle - rpc_min) / (float)(rpc_max - rpc_min)) * (79 - 0);
	/*Serial.print("rpc_out: ");
	Serial.print(rpc_out);
	Serial.print("\n");
	Serial.print("OCR2B: ");
	Serial.println(value);*/
}

void ramp_up(uint16_t rpc_end)
{
	while (rpc_out < rpc_end)
	{
		set_duty_cycle(++rpc_out);
		delay(ms_per_ramp_step);
	}
}

void ramp_up(uint16_t ramp_ms, uint16_t rpc_end)
{
	uint16_t steps = rpc_end - rpc_out;
	uint16_t ms_per_step = ramp_ms / steps;

	while (rpc_out < rpc_end)
	{
		set_duty_cycle(++rpc_out);
		delay(ms_per_step);
	}
}

void ramp_down(uint16_t rpc_end)
{
	while (rpc_out > rpc_end)
	{
		set_duty_cycle(--rpc_out);
		delay(ms_per_ramp_step);
	}
}

void ramp_down(uint16_t ramp_ms, uint16_t rpc_end)
{
	uint16_t steps = rpc_out - rpc_end;
	uint16_t ms_per_step = ramp_ms / steps;

	while (rpc_out > rpc_end)
	{
		set_duty_cycle(--rpc_out);
		delay(ms_per_step);
	}
}

void test(uint32_t ms_to_ramp)
{
	ramp_up(ms_to_ramp, 100);
	delay(ms_to_ramp);
	ramp_down(ms_to_ramp, 0);
	/*uint16_t start_rpc = rpc_out;
	uint16_t steps_to_max = rpc_max - start_rpc;
	uint32_t time_per_step = ms_to_ramp / steps_to_max;
	Serial.print("time_per_step: ");
	Serial.println(time_per_step);
	while (rpc_out < rpc_max)
	{
	set_duty_cycle(rpc_out++);
	delay(time_per_step);
	}

	delay(3000);

	uint16_t steps_to_min = rpc_out - rpc_min;
	time_per_step = ms_to_ramp / steps_to_min;
	Serial.print("time_per_step: ");
	Serial.println(time_per_step);
	while (rpc_out > rpc_min)
	{
	set_duty_cycle(rpc_out--);
	delay(time_per_step);
	}*/
}

void rpm_function()
{
	// runs twice per revolution
	half_revolutions++;
}

double read_temperature()
{
	int chk = DHT.read21((uint8_t)dht_pin);
	if (chk == DHTLIB_OK)
		return DHT.temperature;
	else
		return -1;
}

void setup() {
	Serial.begin(19200);
	current_mode = start;
	// Setup interrupt on digital pin 2, enable pull up resistor for digital 2
	digitalWrite(2, HIGH);
	attachInterrupt(INTERRUPT_PIN0, rpm_function, RISING);
	half_revolutions = 0;
	rpm = 0;
	rpm_time_old = 0;
	temp_time_old = 0;
	// Setup fast pwm on digital 3
	pinMode(11, OUTPUT); // OCR2
	pinMode(3, OUTPUT);  // OCR2
	TCCR2A = _BV(COM2A0) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
	TCCR2B = _BV(WGM22) | _BV(CS21);
	OCR2A = 79;
	//OCR2B = 0;
	set_duty_cycle(rpc_mid); // set OCR2B
	Serial.println("Run initial test");
	test(3000);
}

void loop()
{
	
	if (half_revolutions > 20)
	{
		rpm = 30 * 1000 / (millis() - rpm_time_old);
		rpm_time_old = millis();
		half_revolutions = 0;
	}

	// Read temperature every 2 seconds
	if (millis() - temp_time_old > 2000)
	{
		current_temp = read_temperature();
		Serial.print("Temperature:");
		Serial.println(current_temp);
		temp_time_old = millis();
	}
	// Discard if less than 0
	if (current_temp > 0)
	{
		if (current_temp > max_temp && current_mode != high_temp)
		{
			// Go into full speed mode until temp is down
			Serial.println("Full speed mode");
			last_mode = current_mode;
			current_mode = high_temp;
		}
		else if (current_temp < min_temp && current_mode != low_temp)
		{
			Serial.println("Min speed mode");
			last_mode = current_mode;
			current_mode = low_temp;
		}
		else
		{
			current_mode = last_mode;
		}
	}

	switch (current_mode)
	{
	case start:
		break;
	case min:
		if (rpc_out < rpc_min)
			ramp_down(rpc_min);
		break;
	case max:
		if (rpc_out < rpc_max)
			ramp_up(rpc_max);
		break;
	case user:
		break;
	case high_temp:
		if (rpc_out < rpc_max)
			ramp_up(rpc_max);
		break;
	case low_temp:
		if (rpc_out > rpc_min)
			ramp_down(rpc_min);
		break;
	default:
		break;
	}
}

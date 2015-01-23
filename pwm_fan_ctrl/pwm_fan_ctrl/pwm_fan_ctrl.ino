// Change to delayMicroseconds since delay() is affected by the interrupt
// see http://arduino.cc/en/Reference/attachInterrupt

uint16_t rpc_min = 30;
uint16_t rpc_max = 100;
uint16_t rpc_mid = rpc_min + (rpc_max - rpc_min) / 2;
uint16_t rpc_start = 50;
uint16_t rpc_out = rpc_min;
volatile byte half_revolutions;
uint16_t rpm;
unsigned long rpm_time_old;
uint16_t current_temp;
uint16_t max_temp = 40;
uint16_t min_temp = 20;
enum mode {start, min, max, user, high_temp, low_temp};
mode current_mode;

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

void ramp_up(uint32_t ramp_ms, uint16_t rpc_end)
{
	uint16_t steps = rpc_end - rpc_out;
	uint16_t ms_per_step = ramp_ms / steps;

	while (rpc_out < rpc_end)
	{
		set_duty_cycle(++rpc_out);
		delay(ms_per_step);
	}
}

void ramp_down(uint32_t ramp_ms, uint16_t rpc_end)
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
	delay(3000);
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

uint16_t read_temperature()
{
	return 16;
}

void setup() {
	Serial.begin(19200);
	current_mode = start;
	// Setup interrupt 
	attachInterrupt(0, rpm_function, RISING);
	half_revolutions = 0;
	rpm = 0;
	rpm_time_old = 0;
	// Setup fast pwm on digital 3
	pinMode(11, OUTPUT); // OCR2
	pinMode(3, OUTPUT);  // OCR2
	TCCR2A = _BV(COM2A0) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
	TCCR2B = _BV(WGM22) | _BV(CS21);
	OCR2A = 79;
	//OCR2B = 0;
	set_duty_cycle(rpc_mid); // set OCR2B
	delay(10000);
	Serial.println("We are ready for take off");
	test(30000);
}

void loop()
{
	if (half_revolutions > 20)
	{
		rpm = 30 * 1000 / (millis() - rpm_time_old);
		rpm_time_old = millis();
		half_revolutions = 0;
	}

	// Read temperature
	current_temp = read_temperature();
	if (current_temp > max_temp && current_mode != high_temp)
	{
		// Go into full speed mode until temp is down
		current_mode = high_temp;
	}
	else if (current_temp < min_temp && current_mode != low_temp)
	{
		current_mode = low_temp;
	}

	switch (current_mode)
	{
	case start:
		break;
	case min:
		break;
	case max:
		break;
	case user:
		break;
	case high_temp:
		if (rpc_out < rpc_max)
			ramp_up(3000, rpc_max);
		break;
	case low_temp:
		if (rpc_out > rpc_min)
			ramp_down(3000, rpc_min);
		break;
	default:
		break;
	}
}

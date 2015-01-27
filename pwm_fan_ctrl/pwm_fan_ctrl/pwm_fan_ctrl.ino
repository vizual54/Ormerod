// Change to delayMicroseconds since delay() is affected by the interrupt
// see http://arduino.cc/en/Reference/attachInterrupt

#include <PID_v1.h>
#include <dht.h>
#include "globals.h"

#define INTERRUPT_PIN0 0
#define DEBUG true

const byte pushButton2_pin		= A5;
const byte pushButton1_pin		= A4;
const byte pot_pin				= A6;
//const byte power_led_pin		= A4;
//const byte user_mode_led_pin	= A5;
//const byte auto_mode_led_pin	= A6;
//const byte alert_led_pin		= A7;

const byte rpm_pin				= 2;
const byte pwm_pin1				= 3;
const byte dht_pin				= 4;
const byte reserved				= 11;

uint16_t		rpc_min = 0;
uint16_t		rpc_max = 255;
uint16_t		rpc_mid = rpc_min + (rpc_max - rpc_min) / 2;
uint16_t		rpc_start = rpc_mid;
double		rpc_out = rpc_min;

volatile byte	half_revolutions;
uint16_t		rpm;
unsigned long	rpm_time_old;
volatile double	current_temp;
volatile long	last_good_temp;
volatile int	dht_chk;
uint16_t		max_temp = 40;
uint16_t		min_temp = 20;

mode			current_mode;
failsafe_mode   current_failsafe_mode;
mode			last_mode;
dht				DHT;
uint16_t		ms_per_ramp_step = 43;
unsigned long   temp_time_old;
double			pid_setpoint = 23;
double			pid_input, pid_output;
PID				pid_controller(&pid_input, &rpc_out, &pid_setpoint, 120, 0, 0, REVERSE);

byte			button1_state;
byte			last_button1_state = LOW;
long			last_button1_debounce_time = 0;  // the last time the output pin was toggled
long			debounce_delay = 50;    // the debounce time; increase if the output flickers
byte			button2_state;
byte			last_button2_state = LOW;
long			last_button2_debounce_time = 0;

void set_duty_cycle(double cycle)
{
	OCR2B = (int)((float)cycle / 255.0f * ((float)OCR2A - 23.7f) + 23.7f);
}

void ramp_up(double rpc_end)
{
	while (rpc_out < rpc_end)
	{
		set_duty_cycle(++rpc_out);
		delay(ms_per_ramp_step);
	}
}

void ramp_up(uint16_t ramp_ms, double rpc_end)
{
	uint16_t steps = rpc_end - rpc_out;
	uint16_t ms_per_step = ramp_ms / steps;

	while (rpc_out < rpc_end)
	{
		set_duty_cycle(++rpc_out);
		delay(ms_per_step);
	}
}

void ramp_down(double rpc_end)
{
	while (rpc_out > rpc_end)
	{
		set_duty_cycle(--rpc_out);
		delay(ms_per_ramp_step);
	}
}

void ramp_down(uint16_t ramp_ms, double rpc_end)
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
	ramp_up(ms_to_ramp, rpc_max);
	delay(ms_to_ramp);
	ramp_down(ms_to_ramp, rpc_min);
	delay(ms_to_ramp);
	ramp_up(ms_to_ramp, rpc_mid);
}

void rpm_function()
{
	// runs twice per revolution
	half_revolutions++;
}

void switch_mode(mode m)
{
	if (m != current_mode)
	{
		Serial.print("Switching mode to: ");
		Serial.println(m);
		last_mode = current_mode;
		current_mode = m;
		switch (current_mode)
		{
		case user:
		{
			PORTC = user_mode;
			pid_controller.SetMode(MANUAL);
			double temp_rpc = (double)((float)analogRead(pot_pin) / 1023.0f * (rpc_max - rpc_min) + rpc_min);
			//Serial.print("Temp rpc: ");
			//Serial.println(temp_rpc);
			//Serial.print("RPC is: ");
			//Serial.println(rpc_out);
			if (last_mode == pid_ctrl || last_mode == failsafe)
			{
				if (temp_rpc < rpc_out)
				{
					Serial.print("Ramping down to: ");
					Serial.println(temp_rpc);
					ramp_down(temp_rpc);
				}
				else if (temp_rpc > rpc_out)
				{
					Serial.print("Ramping up to: ");
					Serial.println(temp_rpc);
					ramp_up(temp_rpc);
				}
			}
			
			break;
		}
		case pid_ctrl:
			PORTC = auto_mode;
			pid_controller.SetMode(AUTOMATIC);
			break;
		case failsafe:
			PORTC = alert_mode;
			pid_controller.SetMode(MANUAL);
			break;
		default:
			break;
		}
	}
}

void switch_failsafe_mode(failsafe_mode m)
{
	if (m != current_failsafe_mode)
	{
		current_failsafe_mode = m;
	}
}

void check_button_debounced(const byte button, byte &button_state, byte &last_button_state, byte &last_button_debounce_state, long &last_button_debounce_time)
{
	//Serial.print("check button: ");
	//Serial.println(button);
	byte button_reading = digitalRead(button);
	//Serial.print("Button reading is: ");
	//Serial.print(button_reading);
	if (button_reading != last_button_state)
	{
		last_button_debounce_time = millis();
	}
	if ((millis() - last_button_debounce_time) > debounce_delay)
	{
		if (button_reading != button_state)
		{
			//Serial.println("Button is pressed.");
			button_state = button_reading;
		}
	}
	last_button_state = button_reading;
}

// Timer 1 interrupt read temperature and compute pid
ISR(TIMER1_COMPA_vect)
{
	dht_chk = DHT.read21((uint8_t)dht_pin);
	if (dht_chk == DHTLIB_OK)
	{
		Serial.println("Temp ok.");
		current_temp = DHT.temperature;
		pid_input = current_temp;
		last_good_temp = millis();
	}
	else if (dht_chk == DHTLIB_ERROR_CHECKSUM)
	{
		Serial.print("temp sensor not ok. Checksum error. Got temp: ");
		Serial.println(DHT.temperature);
	}
	else if (dht_chk == DHTLIB_ERROR_TIMEOUT)
		Serial.println("temp sensot not ok. Error timeout.");
	else if (dht_chk == DHTLIB_TIMEOUT)
		Serial.println("temp sensot not ok. Timeout.");
	else if (dht_chk == DHTLIB_INVALID_VALUE)
		Serial.println("temp sensot not ok. Invalid value.");

	pid_controller.Compute();
}

void setup_pins()
{
	DDRC = 0x0F; // Set PC0-PC3 as input and PC4-PC7 as output
	PORTC = 0x0F; // light up all status lights
	//pinMode(alert_led_pin, OUTPUT);
	//pinMode(power_led_pin, OUTPUT);
	//digitalWrite(power_led_pin, HIGH);
	//digitalWrite(auto_mode_led_pin, LOW);
	//digitalWrite(alert_led_pin, LOW);
	// A0 and A2 as inputs
	//pinMode(INPUT, pushButton1_pin);
	//pinMode(INPUT, pushButton2_pin);

	pinMode(pwm_pin1, OUTPUT);  // OCR2A
	//	pinMode(pwm_pin2, OUTPUT); // OCR2B
	digitalWrite(LOW, pwm_pin1);
	//	digitalWrite(LOW, pwm_pin2);
	//pinMode(INPUT, pot_pin);
	pinMode(INPUT, dht_pin);
	// Setup interrupt on digital pin 2, enable pull up resistor for digital 2
	digitalWrite(2, HIGH);
}

int freeRAM() {
	extern int __heap_start, *__brkval;
	int v;
	return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

void setup() {
	Serial.begin(19200);
	cli();		// disable all interrupts
	setup_pins();
	attachInterrupt(INTERRUPT_PIN0, rpm_function, RISING);
	half_revolutions = 0;
	rpm = 0;
	rpm_time_old = 0;
	temp_time_old = 0;
	// Setup 0.5 Hz interrupt on timer 1
	TCCR1A = 0;								// set TCCR1A to 0
	TCCR1B = 0;								// set TCCR1B to 0
	TCNT1 = 0;								//initialize counter value to 0
	OCR1A = 31249;							// set compare match register for 1hz increments (16,000,000 / (1024 * 0.5))-1
	TCCR1B |= (1 << WGM12);					// turn on CTC mode
	TCCR1B |= (1 << CS12) | (1 << CS10);	// Set CS10 and CS12 bits for 1024 prescaler
	TIMSK1 |= (1 << OCIE1A);				// enable timer compare interrupt
	
	// Setup fast pwm on timer 2
	TCCR2A = _BV(COM2A0) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
	TCCR2B = _BV(WGM22) | _BV(CS21);
	OCR2A = 79;
	
	pid_controller.SetOutputLimits(0, 255);
	pid_controller.SetMode(MANUAL);
	//pid_controller.SetControllerDirection(0);
	sei();									// enable interrupts
	
	current_mode = user;
	PORTC = user_mode;
	int value = analogRead(pot_pin);
	rpc_out = value / 1023 * (rpc_max - rpc_min) + rpc_min;
	set_duty_cycle(rpc_out);
}

void loop()
{
	if (half_revolutions > 20)
	{
		rpm = 30 * 1000 / (millis() - rpm_time_old);
		rpm_time_old = millis();
		half_revolutions = 0;
	}

	check_button_debounced(pushButton1_pin, button1_state, last_button1_state, last_button1_state, last_button1_debounce_time);
	if (button1_state)
	{
		//Serial.println("button 1 pressed");
		if (current_mode != user)
		{
			switch_mode(user);
		}
	}
	check_button_debounced(pushButton2_pin, button2_state, last_button2_state, last_button2_state, last_button2_debounce_time);
	if (button2_state)
	{
		//Serial.println("button 2 pressed");
		if (current_mode != pid_ctrl)
		{
			switch_mode(pid_ctrl);
		}
	}

#ifdef DEBUG
	// Print temperature every 5 seconds
	if (millis() - temp_time_old > 5000)
	{
		Serial.print("Temperature:");
		Serial.println(current_temp);
		Serial.print("RPC_OUT is: ");
		Serial.println(rpc_out);
		Serial.print("Free ram : ");
		Serial.println(freeRAM());
		temp_time_old = millis();
	}
#endif

	// Make sure we have good temp readings
	if (dht_chk == DHTLIB_OK && millis() - last_good_temp < 20000)
	{
		if (current_mode != failsafe && current_temp > max_temp)
		{
			// Go into full speed mode until temp is down
			Serial.println("Failsafe mode full speed.");
			switch_mode(failsafe);
			switch_failsafe_mode(high_temp);
		}
		else if (current_mode != failsafe && current_temp < min_temp)
		{
			Serial.println("Failsafe mode min speed.");
			switch_mode(failsafe);
			switch_failsafe_mode(low_temp);
		}
		else if (current_mode == failsafe && current_temp < (max_temp - 5) && current_temp > (min_temp + 5))
		{
			switch_mode(last_mode);
		}
	}
	else if (dht_chk != DHTLIB_OK && millis() - last_good_temp > 20000)  // Something wrong the temp sensor, switch to failsafe and high temp mode
	{
		if (current_mode != failsafe && current_failsafe_mode != high_temp)
		{
			Serial.println("Something wrong with temp sensor. Switching failsafe and full speed.");
			last_mode = current_mode;
			switch_mode(failsafe);
			switch_failsafe_mode(high_temp);
		}
	}

	switch (current_mode)
	{
	case user:
	{
		int value = analogRead(pot_pin);
		rpc_out = (double)((float)value / 1023.0f * (rpc_max - rpc_min) + rpc_min);
		set_duty_cycle(rpc_out);
		break;
	}
	case pid_ctrl:
	{
		//rpc_out = (uint16_t)pid_output;
		set_duty_cycle(rpc_out);
		break;
	}
	case failsafe:
		switch (current_failsafe_mode)
		{
		case high_temp:
		{
			if (rpc_out < rpc_max)
				ramp_up(rpc_max);
			break;
		}
		case low_temp:
		{
			if (rpc_out > rpc_min)
				ramp_down(rpc_min);
			break;
		}
		default:
			break;
		}
		break;
	default:
		break;
	}
	/*
	switch (current_mode)
	{
	case user:   // read potentiometer and rescale
		int value = analogRead(pot_pin);
		rpc_out = value / 1023 * (100 - 30);
		set_duty_cycle(rpc_out);
		break;
	case pid_ctrl: 
		break;
	case failsafe:
		switch (current_failsafe_mode)
		{
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
		break;
	default:
		break;
	}
	*/
}

#include <EEPROM.h>
#include <U8glib.h>
#include <OneWire.h>
#include <avr/wdt.h>
#include <PID_v1.h>
#include "globals.h"
#include "tempProbe.h"

#define INTERRUPT_PIN0 0
//#define DEBUG
//#define DEBUG_TEMP
#define HEARTBEATLED 13
#define HEARTBEATFREQ 500

//const byte pushButton2_pin		= A5;
//const byte pushButton1_pin		= A4;
const byte pot_pin				= A6;
//const byte power_led_pin		= A4;
//const byte user_mode_led_pin	= A5;
//const byte auto_mode_led_pin	= A6;
//const byte alert_led_pin		= A7;

const byte rpm_pin				= 2;
const byte pwm_pin1				= 3;
const byte dht_pin				= 4;
const byte pushButton1_pin		= 9;
const byte pushButton2_pin		= 10;
const byte reserved				= 11;

byte	hbState;
unsigned long	hbMillis = 0;
unsigned long   loopTime;
uint16_t		rpc_min = 0;
uint16_t		rpc_max = 255;
uint16_t		rpc_mid = rpc_min + (rpc_max - rpc_min) / 2;
uint16_t		rpc_start = rpc_mid;
double			rpc_out = rpc_min;
uint16_t		temp_probe_timeout = 10000;
volatile byte	half_revolutions;
uint16_t		rpm;
unsigned long	rpm_time_old;
volatile double	current_temp = -273.15;
double			pid_input_temp;
double			setpoint_temp = 22.5;
double			p_term = 120;
double			i_term = 50;
double			d_term = 10;
volatile long	last_good_temp;
double			max_temp = 40;
double			min_temp = 15;
char			serial_data[101];			// Array for incoming serial-data 
unsigned char	serial_index = 0;			// How many bytes have been received?
char			string_started = 0;			// Only saves data if string starts with right byte
mode			current_mode;
failsafe_mode   current_failsafe_mode;
mode			last_mode;
uint16_t		ms_per_ramp_step = 10;
unsigned long   debug_time;
bool			debug_print;

PID				pid_controller(&pid_input_temp, &rpc_out, &setpoint_temp, p_term, i_term, d_term, REVERSE);
OneWire			oneWire(dht_pin);
TempProbe		tempProbe(&oneWire);
bool			temp_probe_initialized;
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_DEV_0 | U8G_I2C_OPT_NO_ACK | U8G_I2C_OPT_FAST);

byte			button1_state;
byte			last_button1_state = LOW;
unsigned long	last_button1_debounce_time = 0;  // the last time the output pin was toggled
unsigned long	debounce_delay = 50;    // the debounce time; increase if the output flickers
byte			button2_state;
byte			last_button2_state = LOW;
unsigned long	last_button2_debounce_time = 0;

void updateHeartbeat()
{
	if (millis() - hbMillis > HEARTBEATFREQ) {
		hbMillis = millis();
		if (hbState == LOW)
		{
			hbState = HIGH;
		}
		else
		{
			hbState = LOW;
		}
		digitalWrite(HEARTBEATLED, hbState);
	}
}

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

void ramp_down(double rpc_end)
{
	while (rpc_out > rpc_end)
	{
		set_duty_cycle(--rpc_out);
		delay(ms_per_ramp_step);
	}
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
#ifdef DEBUG
		Serial.print("Switching mode to: ");
		Serial.println(m);
#endif
		last_mode = current_mode;
		current_mode = m;
		switch (current_mode)
		{
		case user:
		{
			if (last_mode == pid_ctrl)
				PORTC = user_mode | auto_mode;
			else
				PORTC = user_mode | alert_mode;
			pid_controller.SetMode(MANUAL);
			double temp_rpc = (double)((float)analogRead(pot_pin) / 1023.0f * (rpc_max - rpc_min) + rpc_min);

			if (last_mode == pid_ctrl || last_mode == failsafe)
			{
				if (temp_rpc < rpc_out)
				{
#ifdef DEBUG
					Serial.print("Ramping down to: ");
					Serial.println(temp_rpc);
#endif
					ramp_down(temp_rpc);
				}
				else if (temp_rpc > rpc_out)
				{
#ifdef DEBUG
					Serial.print("Ramping up to: ");
					Serial.println(temp_rpc);
#endif
					ramp_up(temp_rpc);
				}
			}
			PORTC = user_mode;
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

// Timer 1 interrupt 1Hz, read temperature and compute pid
ISR(TIMER1_COMPA_vect)
{
	int8_t res = tempProbe.isReady();
	if (res == SENSOR_READY)
	{
		current_temp = tempProbe.getTemp();
		pid_input_temp = current_temp;
		last_good_temp = millis();
		pid_controller.Compute();
		tempProbe.update();
#ifdef DEBUG_TEMP
		Serial.print("Temp sensor ready. Got temp: ");
		Serial.println(current_temp);
#endif
	}
	else if (res == SENSOR_NOT_OK)
	{
#ifdef DEBUG_TEMP
		Serial.println("Temp sensor not ok, try init again.");
#endif
		temp_probe_initialized = tempProbe.init();
	}
	else
	{
#ifdef DEBUG_TEMP
		Serial.println("Temp sensor not ready.");
#endif
	}

}

void draw()
{
	u8g.setFont(u8g_font_unifont);
	u8g.setPrintPos(0, 10);
	u8g.print("Current mode: ");
	u8g.setPrintPos(0, 22);
	switch (current_mode)
	{
	case user:
		u8g.print("USER");
		break;
	case pid_ctrl:
		u8g.print("PID Controlled");
		break;
	case failsafe:
		u8g.print("FAILSAFE");
		break;
	default:
		u8g.print("Unknown");
		break;
	}
	u8g.setPrintPos(0, 34);
	u8g.print("Temp: ");
	u8g.print(current_temp);
	u8g.setPrintPos(0, 46);
	u8g.print("Setpoint: ");
	u8g.print(setpoint_temp);
	u8g.setPrintPos(0, 58);
	u8g.print("RPC out: ");
	u8g.print(rpc_out);
}

void setup_pins()
{
	DDRC = 0x3F; // Set PC0-PC3 as output and PC4-PC7 as input
	PORTC = 0x0F; // light up all status lights

	pinMode(pwm_pin1, OUTPUT);  // OCR2A
	digitalWrite(LOW, pwm_pin1);
	pinMode(INPUT, dht_pin);
	pinMode(INPUT, pushButton1_pin);
	pinMode(INPUT, pushButton2_pin);
	// Setup interrupt on digital pin 2, enable pull up resistor for digital 2
	digitalWrite(2, HIGH);
}

int freeRAM() {
	extern int __heap_start, *__brkval;
	int v;
	return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

void saveSettings()
{
	saveSetpoint();
	savePidTerms();
	saveMinMaxTemps();
}

void saveSetpoint()
{
	// Setpoint temp
	uint8_t temp = (uint8_t)(setpoint_temp * 10);
	EEPROM.write(1, temp);
}

void savePidTerms()
{
	uint16_t temp;
	// P-term
	temp = p_term * 100;
	EEPROM.write(2, (uint8_t)temp);
	EEPROM.write(3, (uint8_t)(temp >> 8));
	// I-term
	temp = i_term * 100;
	EEPROM.write(4, (uint8_t)temp);
	EEPROM.write(5, (uint8_t)(temp >> 8));
	// D-term
	temp = d_term * 100;
	EEPROM.write(6, (uint8_t)temp);
	EEPROM.write(7, (uint8_t)(temp >> 8));
}

void saveMinMaxTemps()
{
	uint16_t temp;
	temp = min_temp * 10;
	EEPROM.write(8, (uint8_t)temp);
	EEPROM.write(9, (uint8_t)(temp >> 8));
	temp = max_temp * 10;
	EEPROM.write(10, (uint8_t)temp);
	EEPROM.write(11, (uint8_t)(temp >> 8));
}

void loadSettings()
{
	setpoint_temp = ((double)EEPROM.read(1) / 10.0);
	p_term = (double)(EEPROM.read(2) + (EEPROM.read(3) << 8)) / 100;
	i_term = (double)(EEPROM.read(4) + (EEPROM.read(5) << 8)) / 100;
	d_term = (double)(EEPROM.read(6) + (EEPROM.read(7) << 8)) / 100;
	min_temp = (double)(EEPROM.read(8) + (EEPROM.read(9) << 8)) / 10;
	max_temp = (double)(EEPROM.read(10) + (EEPROM.read(11) << 8)) / 10;
#ifdef DEBUG
	Serial.println("Settings loaded from EEPROM.");
	Serial.print("Setpoint temperature: ");
	Serial.println(setpoint_temp);
	Serial.print("p_term: ");
	Serial.print(p_term);
	Serial.print(" i_term: ");
	Serial.print(i_term);
	Serial.print(" d_term: ");
	Serial.println(d_term);
#endif
}

float termToDec(uint8_t length)
{
	uint8_t comma = 0;
	long  rl = 0;
	float rr = 0.0f;
	float rb = 0.1f;
	bool dec = false;
	uint8_t i = 0;

	if (serial_data[i] == '-' || serial_data[i] == '+')
		i++;

	while (i < length)
	{
		if (serial_data[i] == '.' || serial_data[i] == ',')
		{
			dec = true;
		}
		else
		{
			if (!dec)
			{
				rl = (10 * rl) + (serial_data[i] - 48);
			}
			else
			{
				rr += rb * (float)(serial_data[i] - 48);
				rb /= 10.0;
			}
		}
		i++;
	}

	rr += (float)rl;

	if (serial_data[0] == '-')
	{
		rr = 0.0f - rr;
	}

	return rr;
}

void handleSerialCommands()
{
	if (Serial.available())
	{
		if (string_started == 1)
		{
			serial_data[serial_index++] = Serial.read();

			// New SetPoint Temp
			if (serial_data[serial_index - 4] == 'N' &&
				serial_data[serial_index - 3] == 'S' &&
				serial_data[serial_index - 2] == 'P' &&
				serial_data[serial_index - 1] == 'T')
			{
				setpoint_temp = (double)termToDec(serial_index - 4);
				Serial.print("Set new setpoint temp: ");
				Serial.println(setpoint_temp);
				serial_index = 0;
				string_started = 0;
			}
			// New KP
			else if (serial_data[serial_index - 3] == 'N' &&
				serial_data[serial_index - 2] == 'K' &&
				serial_data[serial_index - 1] == 'P')
			{
				p_term = (double)termToDec(serial_index - 3);
				pid_controller.SetTunings(p_term, i_term, d_term);
				Serial.print("Set new Kp: ");
				Serial.println(p_term);
				serial_index = 0;
				string_started = 0;
			}
			// New KI
			else if (serial_data[serial_index - 3] == 'N' &&
				serial_data[serial_index - 2] == 'K' &&
				serial_data[serial_index - 1] == 'I')
			{
				i_term = (double)termToDec(serial_index - 3);
				pid_controller.SetTunings(p_term, i_term, d_term);
				Serial.print("Set new Ki: ");
				Serial.println(i_term);
				serial_index = 0;
				string_started = 0;
			}
			// New KD
			else if (serial_data[serial_index - 3] == 'N' &&
				serial_data[serial_index - 2] == 'K' &&
				serial_data[serial_index - 1] == 'D')
			{
				d_term = (double)termToDec(serial_index - 3);
				pid_controller.SetTunings(p_term, i_term, d_term);
				Serial.print("Set new Kd: ");
				Serial.println(i_term);
				serial_index = 0;
				string_started = 0;
			}
			// New MAX temp before failsafe
			else if (serial_data[serial_index - 4] == 'N' &&
				serial_data[serial_index - 3] == 'M' &&
				serial_data[serial_index - 2] == 'A' &&
				serial_data[serial_index - 1] == 'X')
			{
				max_temp = (double)termToDec(serial_index - 4);
				Serial.print("Set new max_temp: ");
				Serial.println(max_temp);
				serial_index = 0;
				string_started = 0;
			}
			// New MIN temp before failsafe
			else if (serial_data[serial_index - 4] == 'N' &&
				serial_data[serial_index - 3] == 'M' &&
				serial_data[serial_index - 2] == 'I' &&
				serial_data[serial_index - 1] == 'N')
			{
				min_temp = (double)termToDec(serial_index - 4);
				Serial.print("Set new min_temp: ");
				Serial.println(min_temp);
				serial_index = 0;
				string_started = 0;
			}
			// Print SetPoint
			else if (serial_data[serial_index - 3] == 'P' &&
				serial_data[serial_index - 2] == 'S' &&
				serial_data[serial_index - 1] == 'P')
			{
				Serial.print("Setpoint temp: ");
				Serial.println(setpoint_temp);
				serial_index = 0;
				string_started = 0;
			}
			// Print Kp, Ki, Kd
			else if (serial_data[serial_index - 4] == 'P' &&
				serial_data[serial_index - 3] == 'P' &&
				serial_data[serial_index - 2] == 'I' &&
				serial_data[serial_index - 1] == 'D')
			{
				Serial.print("PID controller Kp = ");
				Serial.print(pid_controller.GetKp());
				Serial.print(" Ki = ");
				Serial.print(pid_controller.GetKi());
				Serial.print(" Kd = ");
				Serial.println(pid_controller.GetKd());
				serial_index = 0;
				string_started = 0;
			}
			// Print MAX temp
			else if (serial_data[serial_index - 4] == 'P' &&
				serial_data[serial_index - 3] == 'M' &&
				serial_data[serial_index - 2] == 'A' &&
				serial_data[serial_index - 1] == 'X')
			{
				Serial.print("Max temp = ");
				Serial.println(max_temp);
				serial_index = 0;
				string_started = 0;
			}
			// Print MIN temp
			else if (serial_data[serial_index - 4] == 'P' &&
				serial_data[serial_index - 3] == 'M' &&
				serial_data[serial_index - 2] == 'I' &&
				serial_data[serial_index - 1] == 'N')
			{
				Serial.print("Min temp = ");
				Serial.println(min_temp);
				serial_index = 0;
				string_started = 0;
			}
			// Save SetPoint Temp
			else if (serial_data[serial_index - 4] == 'S' &&
				serial_data[serial_index - 3] == 'S' &&
				serial_data[serial_index - 2] == 'P' &&
				serial_data[serial_index - 1] == 'T')
			{
				Serial.println("Saving setpoint temperature.");
				saveSetpoint();
				serial_index = 0;
				string_started = 0;
			}
			// Save PID terms
			else if (serial_data[serial_index - 4] == 'S' &&
				serial_data[serial_index - 3] == 'P' &&
				serial_data[serial_index - 2] == 'I' &&
				serial_data[serial_index - 1] == 'D')
			{
				Serial.println("Saving PID terms.");
				savePidTerms();
				serial_index = 0;
				string_started = 0;
			}
			// Save Max Min Temps
			else if (serial_data[serial_index - 4] == 'S' &&
				serial_data[serial_index - 3] == 'M' &&
				serial_data[serial_index - 2] == 'M' &&
				serial_data[serial_index - 1] == 'T')
			{
				Serial.println("Saving min max temps.");
				saveMinMaxTemps();
				serial_index = 0;
				string_started = 0;
			}
			else if (serial_data[serial_index - 5] == 'D' &&
				serial_data[serial_index - 4] == 'E' &&
				serial_data[serial_index - 3] == 'B' &&
				serial_data[serial_index - 2] == 'U' &&
				serial_data[serial_index - 1] == 'G')
			{
				debug_print = debug_print ? false : true;
				serial_index = 0;
				string_started = 0;
			}
		}
		else if (Serial.read() == '$')
		{
			string_started = 1;
		}
	}
}

void setup() {
	Serial.begin(19200);
	cli();		// disable all interrupts
	setup_pins();
	u8g.setColorIndex(1);		// Setup display
	attachInterrupt(INTERRUPT_PIN0, rpm_function, RISING);
	half_revolutions = 0;
	rpm = 0;
	rpm_time_old = 0;
	debug_time = 0;
	// Setup 1 Hz interrupt on timer 1
	TCCR1A = 0;								// set TCCR1A to 0
	TCCR1B = 0;								// set TCCR1B to 0
	TCNT1 = 0;								// initialize counter value to 0
	OCR1A = 15624;							// set compare match register for 1hz increments (16,000,000 / (1024 * 1))-1
	TCCR1B |= (1 << WGM12);					// turn on CTC mode
	TCCR1B |= (1 << CS12) | (1 << CS10);	// Set CS10 and CS12 bits for 1024 prescaler
	TIMSK1 |= (1 << OCIE1A);				// enable timer compare interrupt
	
	// Setup fast pwm on timer 2
	TCCR2A = _BV(COM2A0) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
	TCCR2B = _BV(WGM22) | _BV(CS21);
	OCR2A = 79;
	// Set fans to max until we got valid temp and can switch to auto
	set_duty_cycle(255);
	temp_probe_initialized = tempProbe.init();

	if (EEPROM.read(0) != 1)
	{
#ifdef DEBUG
		Serial.println("No settings saved in EEPROM.");
#endif
		saveSettings();
		EEPROM.write(0, 1);
	}

	loadSettings();
	pid_controller.SetTunings(p_term, i_term, d_term);
#ifdef DEBUG
	Serial.print("PID controller Kp = ");
	Serial.print(pid_controller.GetKp());
	Serial.print(" Ki = ");
	Serial.print(pid_controller.GetKi());
	Serial.print(" Kd = ");
	Serial.println(pid_controller.GetKd());
	Serial.print("PID controller setpoint = ");
	Serial.println(setpoint_temp);
#endif
	pid_controller.SetOutputLimits(0, 255);
	pid_controller.SetSampleTime(1000);
	pid_controller.SetMode(MANUAL);
	//pid_controller.SetControllerDirection(0);
	sei();									// enable interrupts

	// Make sure we have a reading from temp sensor that is not 0
	do
	{
#ifdef DEBUG
		Serial.println("Waiting for temp reading.");
		Serial.print("Current temp is: ");
		Serial.println(current_temp);
		Serial.print("temp_probe_initialized = ");
		Serial.println(temp_probe_initialized);
#endif
		delay(500);
	} while (current_temp <= 0);

	pid_input_temp = current_temp;
	pid_controller.Compute();
	pid_controller.SetMode(AUTOMATIC);
	current_mode = pid_ctrl;
	PORTC = auto_mode;
	set_duty_cycle(rpc_out);

	//Enable watchdog
	wdt_enable(WDTO_4S);
}

void loop()
{
	unsigned long begin = millis();
	wdt_reset();
	
	updateHeartbeat();

	handleSerialCommands();

	if (half_revolutions > 20)
	{
		rpm = 30 * 1000 / (millis() - rpm_time_old);
		rpm_time_old = millis();
		half_revolutions = 0;
	}

	//check_button_debounced(pushButton1_pin, button1_state, last_button1_state, last_button1_state, last_button1_debounce_time);
	if (digitalRead(pushButton1_pin))
	{
		//Serial.println("button 1 pressed");
		if (current_mode != user)
		{
			switch_mode(user);
		}
	}
	//check_button_debounced(pushButton2_pin, button2_state, last_button2_state, last_button2_state, last_button2_debounce_time);
	if (digitalRead(pushButton2_pin))
	{
		//Serial.println("button 2 pressed");
		if (current_mode != pid_ctrl)
		{
			switch_mode(pid_ctrl);
		}
	}

	// Print debug info every 5 seconds
	if (debug_print && millis() - debug_time > 5000)
	{
		Serial.print("Current mode: ");
		switch (current_mode)
		{
		case user:
			Serial.println("USER");
			break;
		case pid_ctrl:
			Serial.println("PID Controlled");
			break;
		case failsafe:
			Serial.println("FAILSAFE");
			break;
		default:
			Serial.println("Unknown");
			break;
		}
		Serial.print("Setpoint temperature: ");
		Serial.println(setpoint_temp);
		Serial.print("PID controller Kp = ");
		Serial.print(pid_controller.GetKp());
		Serial.print(" Ki = ");
		Serial.print(pid_controller.GetKi());
		Serial.print(" Kd = ");
		Serial.println(pid_controller.GetKd());
		Serial.print("Max_temp: ");
		Serial.print(max_temp);
		Serial.print(" Min_temp: ");
		Serial.println(min_temp);
		Serial.print("Temperature: ");
		Serial.println(current_temp);
		Serial.print("RPC_OUT is: ");
		Serial.println(rpc_out);
		Serial.print("Free ram : ");
		Serial.println(freeRAM());
		Serial.print("Fan RPM is: ");
		Serial.println(rpm);
		Serial.print("Looptime: ");
		Serial.println(loopTime);
		debug_time = millis();
	}
	
	// Make sure temp probe is present and we have good temp readings
	// If temperature is higher or lower than preset values turn on/off fans.
	// If temp probe is not present or time since last know temperature is too long
	// switch to failsafe and turn on fans high
	if (temp_probe_initialized && ((millis() - last_good_temp) < temp_probe_timeout))
	{
		if (current_mode != failsafe && current_temp > max_temp)
		{
			// Go into full speed mode until temp is down
#ifdef DEBUG
			Serial.println("Failsafe mode full speed.");
#endif
			switch_mode(failsafe);
			switch_failsafe_mode(high_temp);
		}
		else if (current_mode != failsafe && current_temp < min_temp)
		{
#ifdef DEBUG
			Serial.println("Failsafe mode min speed.");
#endif
			switch_mode(failsafe);
			switch_failsafe_mode(low_temp);
		}
		else if (current_mode == failsafe && current_temp < (max_temp - 5) && current_temp > (min_temp + 5))
		{
			switch_mode(last_mode);
		}
	}
	else if (temp_probe_initialized == false || (millis() - last_good_temp) > temp_probe_timeout)  // Something wrong the temp sensor, switch to failsafe and high temp mode
	{
		if (current_mode != failsafe && current_failsafe_mode != high_temp)
		{
#ifdef DEBUG
			Serial.println("Something wrong with temp sensor. Switching failsafe and full speed.");
#endif
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

	// Update display
	u8g.firstPage();
	do {
		draw();
	} while (u8g.nextPage());

	loopTime = millis() - begin;
}

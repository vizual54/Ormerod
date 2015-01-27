#ifndef GLOBALS_H
#define GLOBALS_H

enum mode { user, pid_ctrl, failsafe };
enum failsafe_mode { high_temp, low_temp };
const byte power = 0b00000001;
const byte user_mode = 0b00000011;
const byte auto_mode = 0b00000101;
const byte alert_mode = 0b00001001;

#endif
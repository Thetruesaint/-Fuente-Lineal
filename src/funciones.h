#ifndef FUNCIONES_H
#define FUNCIONES_H

#include <Arduino.h>
#include "variables.h"

// Declaración de funciones
void setup();
void loop();
void Read_encoder();
void Read_keypad();
void ResetInputValue();
void Cursor_position();
void ToggleDisplaySettings ();
void Manage_Display();
void V_I_W_Display(float PrintVoltage, float PrintCurrent, String mensaje);
void Temp_check();
void Limits_check();
void Read_Voltage_Current();
void Set_Voltage_Current(bool setVI = false); 
void Mem_selec();
void Calibration();
void LoadCalibration(int address, float &variable);
void Save_EEPROM(int address, float value);
void Save_Calibration();
void ChangeMode(String Modetoset = "V");

#endif
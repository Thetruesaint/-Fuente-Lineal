#ifndef FUNCIONES_H
#define FUNCIONES_H

#include <Arduino.h>
#include "Variables.h"

// Declaración de funciones
void setup();
void loop();
void Read_encoder();
void Read_keypad();
void Reset_Input_Value();
void Cursor_position();
void Show_VI_Settings ();
void Manage_Display();
void V_I_W_Display(float PrintVoltage, float PrintCurrent, String mensaje);
void Temp_check();
void Limits_check();
void Read_Voltage_Current();
void Set_Voltage_Current(bool setVI = false); 
void Mem_selec(char key);
void Calibration();
void Save_EEPROM(int address, float value);
void Load_EEPROM(int address, float &variable);
void Load_Calibration();
void Save_Calibration();
void Change_Mode(char Modetoset = 'V');
void Calibration_Menu(void);
void Start_Calibration(char mode);
void Output_Control(bool enable = false);
bool Calc_Calib_Fact(float x1, float y1, float x2, float y2, float &factor, float &offset);

#endif
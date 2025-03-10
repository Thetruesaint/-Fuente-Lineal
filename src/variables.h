#ifndef VARIABLES_H
#define VARIABLES_H

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#ifndef WOKWI_SIMULATION
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MCP4725.h>
#endif
#include <Keypad.h>                           //http://playground.arduino.cc/Code/Keypad


#ifndef WOKWI_SIMULATION
extern Adafruit_MCP4725 dacV;                        // Objeto dac para el MP4725 de Control de Tension
extern Adafruit_MCP4725 dacI;                        // Objeto dac para el MP4725 de Control de Corriente
extern Adafruit_ADS1115 ads;                         // Objeto ads para el ADS115
#endif
extern LiquidCrystal_I2C lcd;           // Objeto lcd.Sddress to 0x27 for a 16 chars and 2 line display

//----------------------------------------------------I/O Pins---------------------------------------------------------------

const byte ENC_A = 3;                         // Encoder Pin A
const byte ENC_B = 2;                         // Encoder Pin B 
const byte ENC_BTN = 4;                       // Encoder button
const byte CRR_SNSR = 3;                      // Input A3 from ADC
const byte VLTG_SNSR = 1;                     // Input A1 from ADC
const byte MSFT_CTRL = A1;                    // Enable output control
const byte TEMP_SNSR = A2;                    // Sensado de Temperatura
const byte FAN_CTRL = A3;                     // Endendido de Fan

//----------------------------------------------- Variables para Encoder ----------------------------------------------------

const unsigned long PAUSE_LENGTH = 25000;
const unsigned long FAST_INCREMENT  = 10;
const unsigned long ENCODER_MAX = 999000;              // sets maximum Rotary Encoder value allowed CAN BE CHANGED AS REQUIRED (was 50000)

extern unsigned long lastButtonPress;         //Use this to store if the encoder button was pressed or not
extern unsigned long _lastIncReadTime; 
extern unsigned long _lastDecReadTime; 
extern volatile float setvalue;               // Contador del encoder

//----------------------------------------- Variables de operacion y Modos --------------------------------------------------

const float VOLTS_CUTOFF = 30.0;    // Volaje Maximo
const float CURRENT_CUTOFF = 5.0;    // Corriente Maxima

extern int16_t adcv, adci;          // Guarda el valor en binario del ADC
extern float reading;               // Lecturas de Encoder o keypad
extern int CP;                      // Posiciona el cursor en las unidades para Modo V que es el default
extern int CPprev;                  // Posisión anterior del cursor para saltar el punto decimal
extern volatile float factor;       // Factor de escala que cambia las unidades para Modo V que es el default
extern char Mode;                   // Modo "V" es el default
extern char Modetocal;              // Modo seleccionado para calibrar, no puede cambiar.
extern String Mnsg;                 // Para mostrar mensajes

extern float voltage;               // Voltage real
extern float setvoltage;            // Voltaje a setear

extern float current;               // Corriente real
extern float setcurrent;            // Corriente a setear

extern float Sns_Volt_Fact;         // Factor de diseño para el ADC para 30V Max
extern float Sns_Volt_Calib_Fact;   // Factor de calibración para el ADC de V
extern float Sns_Volt_Calib_Offs;   // Offset de calibracion de voltage sensado

extern float Sns_Curr_Fact;         // Factor de diseño para el ADC validar
extern float Sns_Curr_Calib_Fact;   // Factor de calibración para el ADC de I
extern float Sns_Curr_Calib_Offs;   // Offset de calibracion de corriente sensada

extern float Out_Volt_Fact;         // Factor de diseño para el DacV considerando que con 3308 da 4.0389V que setea 30V de voltage de salida
extern float Out_Volt_Calib_Fact;   // Factor de calibración para el DAC de V
extern float Out_Volt_Calib_Offs;   // Offset de calibracion de voltage de salida

extern float Out_Curr_Fact;         // Factor de diseño para el DacI de diseño, para probar.
extern float Out_Curr_Calib_Fact;   // Factor de calibración para el DAC de I
extern float Out_Curr_Calib_Offs;   // Offset de calibracion de corriente máxima de salida

extern bool hlth;                   // Flag de Salud gral.
extern bool mem_st;                 // Flag de seleccion de memorias o presets
extern bool cal_st;                 // Flag para Calibración
extern String Req_info;             // Indica que información es requerida ingresar

//-------------------------------------------Variables para el Keypad---------------------------------------------------------

const int ROWS = 4;          // Teclado de 4 renglones
const int COLS = 5;          // Teclado de 5 columnas

extern char hexaKeys[ROWS][COLS];   // Matriz de simbolos para los botones del teclado


extern byte rowPins[ROWS];          // connect to the row pin outs of the keypad
extern byte colPins[COLS];          // connect to the column pin outs of the keypad

extern Keypad customKeypad;
extern char customKey;
extern char decimalPoint;           // used to test for more than one press of * key (decimal point)
extern char numbers[8];             // keypad number entry - Plenty to store a representation of a float
extern byte index;
extern float x;

//------------------------------------------Variables de coordenadas para el cursor del LCD------------------------------------
extern int y;               // Posición provisoria
extern int r;               // Renglon
extern byte amp_char[8];    // A mayúscula mas chica


//------------------------------------------ Varaible para temporizar la visualizaciòn ----------------------------------------

const unsigned long DSP_SET_DRTN = 2000;    // Tiempo que se mantiene mostrando los valores seteados (ms)
const unsigned long DSP_INFO_RFRSH = 400;   // Tiempo de refresco cuando solo está informando (ms)

extern bool dspset;                         // Flag que indica que se debe mostrar seteo
extern bool dspsetchng;                     // Flag que indica que hubo cambio en algun set para poder mostrarlo
extern unsigned long dspset_time;           // Tiemp que paso mostrando el valor seteado
extern unsigned long dsp_time;              // Tiemp que paso mostrando el valor sensado

//-------------------------------------------Variables para Control de Temperatura---------------------------------------------
const int TMP_CHK_TIME = 800;       // Perdiodo de control de temperatura (miliseg.)
const int FAN_ON_DRTN = 30000;      // Tiempo en miliseg. para mantener los fans encendidos (30 segundos)

extern int temp;                    // Registra temperatura
extern unsigned long Last_tmpchk;   // Tiempo desde el ùltimo chequeo de temperatura
extern unsigned long fan_on_time;   // Tiempo que lleva encendido el Fan
extern bool fans_on;                // Estado de los Coolers
extern bool new_temp;               // flag si hay nuevo valor de temperatura

//----------------------------------------------Direcciónamiento de la EEPRON--------------------------------------------------
const int ADD_SNS_VOLT_FAC_CAL = 0;            // Dirección para Sns_Volt_Calib_Fact
const int ADD_SNS_CURR_FAC_CAL = 4;            // Dirección para Sns_Curr_Calib_Fact
const int ADD_OUT_VOLT_FAC_CAL = 8;            // Dirección para Out_Volt_Calib_Fact
const int ADD_OUT_CURR_FAC_CAL = 12;           // Dirección para Out_Curr_Calib_Fact
const int ADD_SNS_VOLT_OFF_CAL = 16;           // Dirección para Sns_Volt_Calib_Offs
const int ADD_SNS_CURR_OFF_CAL = 20;           // Dirección para Sns_Curr_Calib_Offs
const int ADD_OUT_VOLT_OFF_CAL = 24;           // Dirección para Out_Volt_Calib_Offs
const int ADD_OUT_CURR_OFF_CAL = 28;           // Dirección para Out_Curr_Calib_Offs

#endif
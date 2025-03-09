#include "variables.h"

#ifndef WOKWI_SIMULATION
Adafruit_MCP4725 dacV;                        // Objeto dac para el MP4725 de Control de Tension
Adafruit_MCP4725 dacI;                        // Objeto dac para el MP4725 de Control de Corriente
Adafruit_ADS1115 ads;                         // Objeto ads para el ADS115
#endif
LiquidCrystal_I2C lcd(0x27, 16, 2);           // Objeto lcd.Sddress to 0x27 for a 16 chars and 2 line display

//----------------------------------------------- Variables para Encoder ----------------------------------------------------

unsigned long lastButtonPress = 0;            //Use this to store if the encoder button was pressed or not
unsigned long _lastIncReadTime = micros(); 
unsigned long _lastDecReadTime = micros(); 
volatile float setvalue = 0;                  // Contador del encoder

//----------------------------------------- Variables de operacion y Modos --------------------------------------------------

int16_t adcv, adci;                           // Guarda el valor en binario del ADC
float reading = 0;                            // Lecturas de Encoder o keypad
int CP = 1;                                   // Posiciona el cursor en las unidades para Modo V que es el default
int CPprev = 11;                              // Posisión anterior del cursor para saltar el punto decimal
volatile float factor = 1000;                 // Factor de escala que cambia las unidades para Modo V que es el default
String Mode = "V";                            // Modo "V" es el default
String Mnsg = "          ";                   // Para mostrar mensajes

float voltage = 0;                            // Voltage real
float setvoltage = 0;                         // Voltaje a setear

float current = 0;                            // Corriente real
float setcurrent = 0;                         // Corriente a setear

float Sns_Volt_Fact = 7.423;                  // Factor de diseño para el ADC para 30V Max
float Sns_Volt_Calib_Fact = 1.0;              // Factor de calibración para el ADC de V
float Sns_Volt_Calib_Offs = 0.0;              // Offset de calibracion de voltage sensado

float Sns_Curr_Fact = 1.25;                   // Factor de diseño para el ADC validar
float Sns_Curr_Calib_Fact = 1.0;              // Factor de calibración para el ADC de I
float Sns_Curr_Calib_Offs = 0.0;              // Offset de calibracion de corriente sensada

float Out_Volt_Fact = 108.10;                 // Factor de diseño para el DacV considerando que con 3308 da 4.0389V que setea 30V de voltage de salida
float Out_Volt_Calib_Fact = 1.0;              // Factor de calibración para el DAC de V
float Out_Volt_Calib_Offs = 0.0;              // Offset de calibracion de voltage de salida

float Out_Curr_Fact = 630;                    // Factor de diseño para el DacI de diseño, para probar.
float Out_Curr_Calib_Fact = 1.0;              // Factor de calibración para el DAC de I
float Out_Curr_Calib_Offs = 0.0;              // Offset de calibracion de corriente máxima de salida

bool hlth = true;                             // Flag de Salud gral.
bool mem_st = false;                          // Flag de seleccion de memorias o presets
bool cal_st = false;                          // Flag para Calibración
String Req_info = "      ";                   // Indica que información es requerida ingresar

//-------------------------------------------Variables para el Keypad---------------------------------------------------------

char hexaKeys[ROWS][COLS] = {                 // Matriz de simbolos para los botones del teclado
  {'1','2','3','V','U'},
  {'4','5','6','I','<'},
  {'7','8','9','M','>'},
  {'C','0','.','E','D'}
};

byte rowPins[ROWS] = {5, 6, 7, 8};            // connect to the row pin outs of the keypad
byte colPins[COLS] = {9, 10, 11, 12, 14};     // connect to the column pin outs of the keypad

Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);
char customKey;
char decimalPoint;                            // used to test for more than one press of * key (decimal point)
char numbers[8];                              // keypad number entry - Plenty to store a representation of a float
byte index = 0;
float x = 0;

//------------------------------------------Variables de coordenadas para el cursor del LCD------------------------------------
int y = 0;                                    // Posición provisoria
int r = 0;                                    // Renglon
byte amp_char[8] = {                          // A mayúscula mas chica
  B00000,
  B00000,
  B01110,
  B10001,
  B10001,
  B11111,
  B10001,
  B00000
  };

//------------------------------------------ Varaible para temporizar la visualizaciòn ----------------------------------------

bool dspset = true;                           // Flag que indica que se debe mostrar seteo
bool dspsetchng = false;                      // Flag que indica que hubo cambio en algun set para poder mostrarlo
unsigned long dspset_time = 0;                // Tiemp que paso mostrando el valor seteado
unsigned long dsp_time = 0;                   // Tiemp que paso mostrando el valor sensado

//-------------------------------------------Variables para Control de Temperatura---------------------------------------------

int temp = 0;                                 // Registra temperatura
unsigned long Last_tmpchk = 0;                // Tiempo desde el ùltimo chequeo de temperatura
unsigned long fan_on_time = 0;                // Tiempo que lleva encendido el Fan
bool fans_on = false;                         // Estado de los Coolers
bool new_temp = true;                         // flag si hay nuevo valor de temperatura

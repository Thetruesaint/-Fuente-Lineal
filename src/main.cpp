//#define WOKWI_SIMULATION

#include "Variables.h"
#include "Funciones.h"

namespace {

void PrintCentered(uint8_t row, const char *text) {
  byte len = 0;
  while (text[len] != '\0' && len < 20) len++;
  lcd.setCursor((20 - len) / 2, row);
  lcd.print(text);
}

void PrintTempRight() {
  char tempBuf[6];
  byte len = 0;

  itoa(temp, tempBuf, 10);
  while (tempBuf[len] != '\0' && len < 4) len++;
  tempBuf[len++] = char(0xDF);
  tempBuf[len++] = 'C';
  tempBuf[len] = '\0';

  lcd.setCursor(20 - len, 0);
  lcd.print(tempBuf);
}

}

//---------------- ------------- Set I/O, ADC, DAC, Healt check, Versioning and Calibration load ------------------------------
void setup() {

  //----------------------------------------------------Inicializa I/O---------------------------------------------------------
  pinMode (MSFT_CTRL, OUTPUT);                   // Primer paso, para que deshabilite el Output
  pinMode (ENC_A, INPUT_PULLUP);
  pinMode (ENC_B, INPUT_PULLUP);
  pinMode (ENC_BTN, INPUT_PULLUP);
  pinMode (TEMP_SNSR, INPUT);
  pinMode (FAN_CTRL, OUTPUT);
  attachInterrupt (digitalPinToInterrupt(ENC_A), Read_encoder, CHANGE);
  attachInterrupt (digitalPinToInterrupt(ENC_B), Read_encoder, CHANGE);

  //-------------------------------------Inicializa perifericos-------------------------------------------
  Serial.begin(9600);                           // Para Debugs y Logs
  lcd.begin(20,4);                              // initialize the lcd, default address 0x27
  lcd.backlight();
  lcd.createChar(0, amp_char);                  //Guardo el caracter en la pos. 0 del Lcd para llamarlo luego 

  lcd.clear();
  lcd.setCursor(3,0);
  lcd.print(F("Fuente  Lineal"));
  lcd.setCursor(3,1);
  lcd.print(F("By Guy & Codex"));
  lcd.setCursor(7,2);
  #ifndef WOKWI_SIMULATION
  lcd.print(F("v1.54"));       // Probado Ok en HW                                 
  delay(2000);
  #else
  lcd.print(F("v1.54 - SIM"));  // Test en Simulación
  delay(500);
  #endif

  #ifndef WOKWI_SIMULATION
  lcd.clear();
  bool dacVOk = dacV.begin(0x61);               // initialize dac ads, set address 0x61
  bool dacIOk = dacI.begin(0x60);               // initialize dac, default address 0x60
  bool adsOk = ads.begin();                     // initialize the ads, default address 0x48

  if (!dacVOk) {
      PrintCentered(1, "dacV NDT"); hlth = false;
      }
  if (!dacIOk) {
      PrintCentered(2, "dacI NDT"); hlth = false;
      }
  if (!adsOk) {
      PrintCentered(3, "ads NDT"); hlth = false;
      }
  if (adsOk) {
      ads.setGain(GAIN_TWOTHIRDS);              // 2/3x gain +/- 6.144V  1 bit = 0.1875mV
      }

    temp = analogRead(TEMP_SNSR);              // Tomar temperatura
    temp = temp * 0.48828125;                  // Convertir a Celsius
    PrintTempRight();

  if (temp > 99) {
    PrintCentered(0, "Tmp OoR");
    hlth = false;
  }

  if (hlth == true) {
    PrintCentered(1, "Selfcheck OK");
    Output_Control(true);                      // Habilito MOSFET si se ve todo ok
  } else {
    while (true) {}
  }

  delay(1000);
  #else
  lcd.clear();
  temp = analogRead(TEMP_SNSR);
  temp = temp * 0.09765625;
  PrintTempRight();
  PrintCentered(1, "Selfcheck OK");
  delay(700);
  #endif

  //----------------------------------------Configuraciones iniciales de única vez----------------------------------------------
  //dacV.setVoltage(0,true);                     // reset DAC to zero for no output current set at Switch On, Cambio a "True" para que guarde este valor en la Emprom
  //dacI.setVoltage(0,true);                     // reset DAC to zero for no output current set at Switch On, Cambio a "True" para que guarde este valor en la Emprom
  //-------------------------------------------------Pantalla Inicio------------------------------------------------------------

  lcd.clear();
  Load_Calibration();
}

//--------------------------------------------------- Bucle Principal ---------------------------------------------------------
void loop() {

  reading = setvalue / 1000;                                    //Se registra en la variable reading.
  Read_keypad();                                                //Verifica si hubo entrada por Teclado
  Limits_check();                                               //Chequea valores máximos de V/I y los fija si se superan. Tiene que ser despues de las funciones de ingreso de valores
  Cursor_position();                                            //Recalcula los factores de acuerdo a la posición del cursor
  Read_Voltage_Current();                                       //Muestro V/I sensados
  Set_Voltage_Current();                                        //Seteo V/I de corresponder
  Manage_Display();                                             //Refresco de información en LCD
  Temp_check();                                                 //Chequea Temperatura y acciona en consecuencia

}

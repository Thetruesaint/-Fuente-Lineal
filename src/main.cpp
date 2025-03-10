//#define WOKWI_SIMULATION

#include "Variables.h"
#include "Funciones.h"

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
  lcd.begin(16,2);                              // initialize the lcd, default address 0x27
  lcd.backlight();
  lcd.createChar(0, amp_char);                  //Guardo el caracter en la pos. 0 del Lcd para llamarlo luego 
  #ifndef WOKWI_SIMULATION
  if (dacV.begin(0x61)){
      lcd.setCursor(0,0);
      lcd.print("dacV OK");
      }                                         // initialize dac ads, set address 0x61
      else{
      lcd.setCursor(0,0);
      lcd.print("dacV NDT"); hlth = false;
      Serial.print("dacV NDT");
      }
  if (dacI.begin(0x60)){                        // initialize dac, default address 0x60 
      lcd.setCursor(8,0);
      lcd.print("dacI OK");
      }                            
      else{
      lcd.setCursor(8,0);
      lcd.print("dacI NDT"); hlth = false;
      Serial.print("dacI NDT");
      }
  if (ads.begin()){                             // initialize the ads, default address 0x48
      ads.setGain(GAIN_TWOTHIRDS);              // 2/3x gain +/- 6.144V  1 bit = 0.1875mV
      lcd.setCursor(0,1);
      lcd.print("ads OK");
      }                                         
      else{
      lcd.setCursor(0,1);
      lcd.print("ads NDT"); hlth = false;
      Serial.print("ads NDT");
      }

    temp = analogRead(TEMP_SNSR);              // Tomar temperatura
    temp = temp * 0.48828125;                  // Convertir a Celsius

    lcd.setCursor(11,1);                       // Mostrar Sensado de Temperatura
    lcd.print(temp);
    lcd.print((char)0xDF);
    lcd.print("C");

  if (hlth == true && temp <= 99) {
    Output_Control(true);                      // Habilito MOSFET si se ve todo ok
    }

  delay(1000);
  #endif

  //----------------------------------------Configuraciones iniciales de única vez----------------------------------------------
  //dacV.setVoltage(0,true);                     // reset DAC to zero for no output current set at Switch On, Cambio a "True" para que guarde este valor en la Emprom
  //dacI.setVoltage(0,true);                     // reset DAC to zero for no output current set at Switch On, Cambio a "True" para que guarde este valor en la Emprom
  //-------------------------------------------------Pantalla Inicio------------------------------------------------------------

  lcd.clear();
  lcd.setCursor(1,0);
  lcd.print("Fuente Lineal");
  lcd.setCursor(0,1);
  #ifndef WOKWI_SIMULATION
  lcd.print("v1.53b");        // No probado en HW                                 
  delay(1000);
  #else
  lcd.print("v1.53 - SIM");   // Test en Simulación
  delay(500);
  #endif
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

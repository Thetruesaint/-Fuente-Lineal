#include "Variables.h"
#include "Funciones.h"
#include <EEPROM.h>                           //include EEPROM library used for storing setup data

//---------------------------------------------------- Encoder Decoder --------------------------------------------------------
void Read_encoder() {
  // Encoder interrupt routine for both pins. Updates setvalue
  // if they are valid and have rotated a full indent
  
  static uint8_t old_AB = 3;                                    // Lookup table index
  static int8_t encval = 0;                                     // Encoder value  
  static const int8_t enc_states[]  = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};    // Lookup table

  old_AB <<=2;  // Remember previous state

  if (digitalRead(ENC_A)) old_AB |= 0x02;                       // Add current state of pin A
  if (digitalRead(ENC_B)) old_AB |= 0x01;                       // Add current state of pin B
  
  encval += enc_states[( old_AB & 0x0f )];

  // Update setvalue if encoder has rotated a full indent, that is at least 4 steps
  if( encval > 3 ) {                                            // Four steps forward
    
    setvalue = setvalue + factor;                              // Update setvalue Up
    encval = 0;
  }
  else if( encval < -3 ) {                                     // Four steps backward
    
    setvalue = setvalue - factor;                             // Update setvalue Down
    encval = 0;
  }
  setvalue = min(ENCODER_MAX, max(0, setvalue));
  r = 0; y = CP;                                              // Muevo el Cursor en renglòn de V e I y en la posición de CP
} 

//--------------------------------------------------- Read Keypad Input -------------------------------------------------------
void Read_keypad (void) {
  char customKey = customKeypad.getKey();
  byte index_max = 4;
  
  if (customKey == NO_KEY) return;  // Solo procesar si se presionó una tecla

  Show_VI_Settings();  // Indica que hubo un cambio en el seteo

  if (customKey == 'V') { Change_Mode(); return; }    // Cambiar a Modo CV
  if (customKey == 'I') { Change_Mode('I'); return; } // Cambiar a Modo CI
        
  if (customKey == 'M') { // Selección de memorias
    mem_st = !mem_st;
    if (cal_st) mem_st = false; // Si está en modo calibración, no entra en modo Preset en Memoria
    if (mem_st) { Req_info = "Preset"; r = 1; y = index; }
    else if (!cal_st) { Req_info = "      "; }
    return;
  }

  if (customKey >= '0' && customKey <= '9') {  // Teclas numéricas
    if (mem_st && !cal_st) { Mem_selec(customKey); return;}
    else if (cal_st) index_max = 5;
    index = min(index_max, max(0, index)); 
    numbers[index++] = customKey;
    numbers[index] = '\0';  
    Mnsg = numbers;
    r = 1; y = index;
    return;
  }

  if (customKey == '.') {  // Punto decimal
    if (decimalPoint != ('*')) {
      numbers[index++] = '.';
      numbers[index] = '\0';
      Mnsg = numbers;
      r = 1; y = index;
      decimalPoint = ('*');
    }
    return;
  }

  if (customKey == 'E') { // Enter
    x = atof(numbers);
    Reset_Input_Value();
    if (cal_st) {
      Calibration();
    } else {
      reading = x;
      setvalue = reading * 1000;
      Mnsg = String(Mode) + F(" set!");  // Convierte Mode en string y concatena
    }
    r = 0; y = 8;
    Output_Control(true);
    return;
  }

  if (customKey == 'C' && index > 0) { // Borrar valores ingresados
    index--;  
    if (numbers[index] == '.') decimalPoint = ' ';  
    numbers[index] = '\0';  
    Mnsg = numbers;
    lcd.setCursor(index, r);
    lcd.print(" ");
    r = 1; y = index;
    return;
  }

  if (customKey == 'U') { // Aumentar setpoint
    setvalue = min(ENCODER_MAX, max(0, setvalue + factor));
    r = 0; y = CP;
    return;
  }

  if (customKey == 'D') { // Disminuir setpoint
    setvalue = min(ENCODER_MAX, max(0, setvalue - factor));
    r = 0; y = CP;
    return;
  }
  
  if (customKey == '<') { CP--; return; } // Cursor a la izquierda
  if (customKey == '>') { CP++; return; } // Cursor a la derecha
}

//------------------------------------------------------ Clear Input ----------------------------------------------------------
void Reset_Input_Value() {
    index = 0;                                              // Resetear index de entrada de valores
    numbers[index] = '\0';                                  // Resetar entrada de valores
    decimalPoint = (' ');                                   // Clear decimal point test character reset
    Mnsg = "          ";                                    // Limpiar zona de mensajes
}

//---------------------------------------------------- Cursor Position --------------------------------------------------------
void Cursor_position(void) {

  if (digitalRead(ENC_BTN) == LOW) {
    delay(200);                                               // Simple key bounce delay
    CP++;                                                     // Avanzo el cursor con el botón del encoder
    Show_VI_Settings();
  }

  if (CP != CPprev) {
    if (Mode == 'V') {                                        // Si esta en Modo V:
      int unitPosition = 0;
      if (CP < unitPosition) {CP = unitPosition+4;}           // Vuelve a las centesimas
      if (CP > unitPosition +4) {CP = unitPosition;}          // Vuelve a las decenas
      if (CP == unitPosition + 2) {                           // Si esta en el pto. decimal...
      if (CP > CPprev) {CP++;}                                // y viene de la izq. salta a la der.
            else {CP--;}                                      // sino es porque viende de la der. y salta a la izq.
          }
      if (CP == unitPosition) {factor = 10000;}               // factor para decenas de V
      if (CP == unitPosition +1) { factor = 1000; }           // factor para unidades de V
      if (CP == unitPosition +3) { factor = 100; }            // factor para decimas de V
      if (CP == unitPosition +4)  {factor = 10; }             // Factor para centesimas de V
    }
    
    if (Mode == 'I') {
      int unitPosition = 10;
      if (CP < unitPosition) {CP = unitPosition+4;}           // Vuelve a las centesimas
      if (CP > unitPosition +4) {CP = unitPosition;}          // Vuelve a las decenas
      if (CP == unitPosition + 1) {                           // Si esta en el pto. decimal...
      if (CP > CPprev) {CP++;}                                // y viene de la izq. salta a la der.
            else {CP--;}                                      // sino es porque viende de la der. y salta a la izq.
          }
      if (CP == unitPosition) {factor = 1000;}               // factor para unidades de I
      if (CP == unitPosition +2) { factor = 100; }           // factor para decimas de I
      if (CP == unitPosition +3) { factor = 10; }            // factor para centesimas de I
      if (CP == unitPosition +4)  {factor = 1; }             // Factor para milésimas de I
    }
    r = 0; y = CP;                                           // Muevo el Cursor en renglòn de V e I y en la posición de CP
    CPprev = CP;
  }
}

//------------------------------------------------- Toggle Display Settings ---------------------------------------------------
void Show_VI_Settings (void) {                              
  dspset = true;                                                  // Flag de que hubo cambio de set o se presiono una tecla
  dspsetchng = true;                                              // Flag de que hubo cambio de set
  dspset_time = millis();                                         // Se resetea el tiempo que mostrara el cambio
}

//---------------------------------------------------- Display Managment ------------------------------------------------------
void Manage_Display(void) {
  unsigned long current_time = millis();

  if (dspset && dspsetchng) {
      V_I_W_Display(setvoltage, setcurrent, Mnsg);                // Muestra los valores seteados
      dspsetchng = false;                                         // Se saca el flag de cambio de set
      dspset_time = current_time;                                 // Toma nota del momento en que se mostro el set
      lcd.blink();                                                // Muestra el cursor  
  }
  else if(!dspset) {
    if ((current_time - dsp_time) >= DSP_INFO_RFRSH) {            // Si paso el tiempo de refresco de info
      V_I_W_Display(voltage, current, Mnsg);                      // Muestra los valores sensados
      dsp_time = current_time;                                    // Reset crono de tempo de Display
    }
  }

  // Manejo del temporizador para alternar mostrar valores seteados y sensados
  if (dspset && (current_time - dspset_time) >= DSP_SET_DRTN) {   // Si se vencio el tiempo volver a mostrar V/I
    dspset = false;                                               // Se vencio el tiempo y se saca el flag
    if (!cal_st) Reset_Input_Value();                             // Reseteo valor ingresado
    lcd.noBlink();                                                // Desactiva el cursor, solo se muesrtra en seteo
  }
}

//------------------------------------------------ Display variables on LCD ---------------------------------------------------
void V_I_W_Display(float PrintVoltage, float PrintCurrent, String mensaje) {
  
  PrintVoltage = max(PrintVoltage, 0);                            // Asegurarse de que no se impriman valores negativos
  PrintCurrent = max(PrintCurrent, 0);                            // Asegurarse de que no se impriman valores negativos
  float PrintPower = PrintCurrent * PrintVoltage;

  lcd.setCursor(0,0);
  if (PrintVoltage <= 9.99) {lcd.print(" ");}
  lcd.print(PrintVoltage, 2);
  lcd.print("v");

  lcd.setCursor(10,0);  
  lcd.print(PrintCurrent, 3);
  lcd.write((uint8_t)0);                                          // Llamo al caracter del LCD en la posición 0

  lcd.setCursor(10,1);
  if (!mem_st && !cal_st) {
    if (PrintPower < 10.00) {lcd.print(" ");}
    lcd.print(PrintPower, ((PrintPower >= 100.0) ? 1 : 2));       // Pone la potencia con uno o dos decimales
    lcd.print("w");
    } 
    else if (mem_st || cal_st){
    lcd.print(Req_info);                                          // Muestra el mensaje de requerimiento
    }

  lcd.setCursor(6,0);
  if (Mode == 'V'){lcd.print((char)0x7F);} else{lcd.print(" ");}  // Flecha hacia la V
  lcd.print("C");
  lcd.print(Mode);
  if (Mode == 'I'){lcd.print((char)0x7E);} else{lcd.print(" ");}  // Flecha hacia la I

  lcd.setCursor(0,1);

  if (mensaje == "          " && !cal_st){
    lcd.print("       ");           // Borro restos de cualquier otro mensaje
    if (temp < 10){lcd.print(" ");} // Pone un espacio si es menor un digito.
    lcd.print(temp);
    lcd.print(char(0xDF));          // Signo de Grados
  }
    else {
      lcd.print(mensaje);                                         // Muestro el mensaje
      for (unsigned int i = 1; i<= (10 - mensaje.length()); i++){ //Rellena el espacio de mensaje que queda con espacios para borrar los caranteres de mensajes anteriores
      lcd.print(" ");
      }
    }
  lcd.setCursor(y,r);
}

//--------------------------------------------------- Temperature Check -------------------------------------------------------
void Temp_check(void) {
  static unsigned long Last_tmpchk = 0;                // Tiempo desde el ùltimo chequeo de temperatura
  static unsigned long fan_on_time = 0;                // Tiempo que lleva encendido el Fan
  static bool fans_on = false;                         // Estado de los Coolers
  unsigned long hldtmp_time = millis();
  
  if ((hldtmp_time - Last_tmpchk) >= TMP_CHK_TIME) {                           // Si pasaron TMP_CHK_TIME milisegundos, tomará la temperatura
    int act_temp = analogRead(TEMP_SNSR);                                      // Tomar temperatura del disipador
    #ifndef WOKWI_SIMULATION
    act_temp = act_temp * 0.48828125; // Convertir a Celsius
    #else
    act_temp = act_temp * 0.09765625; // Hasta 100°C con el pote de 0 a 5V que simula sensor de temperatura
    #endif

    if (temp != act_temp) {                                                   // Recalcular solo si cambió la temperatura
      temp = act_temp;
      new_temp = true;                                                        // Flag de que cambio la temperatura
    }
    else {new_temp = false;}                                                  // Si no hubo cambio deja en falso el flag

    Last_tmpchk = hldtmp_time;                                                // Actualiza el momento de chequeo de temperatura

    if (temp >= 40) {                                                         // Si la temperatura es igual o mayor a 40°C enciende el cooler
      digitalWrite(FAN_CTRL, HIGH);                                           // Encender el cooler si la temperatura es mayor o igual a 35°C
      fans_on = true;                                                         // Flag de cooler encendido
      fan_on_time = hldtmp_time;                                              // Actualizo el tiempo de encendido del cooler
    } else if (fans_on && (hldtmp_time - fan_on_time) >= FAN_ON_DRTN) {       // Si temp es < 40 va a dejar prendido el fan por FAN_ON_DRTN
        digitalWrite(FAN_CTRL, LOW);                                          // Apaga el cooler si paso el tiempo
        fans_on = false;                                                      // Flag de cooler encendido
      }
  }
}

//----------------------------------- Limit Maximum Setting & PWR Transistors protection --------------------------------------
void Limits_check (void) {

  float actpwrdis;
  static float maxpwrdis;                             // Potencia máxima de disipación calculada para el TIP3055, 90W para temp. ambiente
  
  if (temp >= 99) {                                   // Controlar el encendido de los fans y el temporizador
    Output_Control(false);                            // Deshabilito MOSFET y este a los TIP3055
    Reset_Input_Value();
    Mnsg = F("OFF Max T ");
  }

  if (new_temp){                                      // Si hay un nuevo valor de temperatura...
    maxpwrdis = max(0, min(90, 108 - 0.72 * temp));   // Recalculo la potencia máxima que se podrá con un TIP3055, limitada entre 90 y 0
    new_temp = false;                                 // para no volver a recalcular maxpwrdis
  }

  actpwrdis = max(0,(38 - voltage) * current / 2);    // Con corriente de mas de 2A baja la tensión de 41 a 38. La corriente se divide por dos TIP3055 y no puede ser menor que 0

  if (actpwrdis >= maxpwrdis) {
    Output_Control(false);                            // Deshabilito MOSFET y este a los TIP3055
    Reset_Input_Value();
    Mnsg = F("OFF Max WD");
  }

  if (Mode =='V' && reading > VOLTS_CUTOFF) {         // Si en Modo V y la lectura supera el máximo
    reading = VOLTS_CUTOFF;                           // Mantengo el valor a mostrar en máximo
    setvalue = VOLTS_CUTOFF * 1000;                   // Lo mismo para el setvalue
    Reset_Input_Value();
    Mnsg = F("Max V");
  }

  if (Mode =='I' && reading > CURRENT_CUTOFF) {       // Si en Modo I y la lectura supera el máximo
    reading = CURRENT_CUTOFF;                         // Mantengo el valor a mostrar en máximo
    setvalue = CURRENT_CUTOFF * 1000;                 // Lo mismo para el setvalue
    Reset_Input_Value();
    Mnsg = F("Max I");
  }
}

//---------------------------------------------- Reading Voltage and current --------------------------------------------------
void Read_Voltage_Current() {
  #ifndef WOKWI_SIMULATION
  float raw_voltage = 0;
  float raw_current = 0;

                                                      //static float multiplier = 0.1875F; /* ADS1115  @ +/- 6.144V gain (16-bit results) */
                                                      //ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 0.1875mV
                                                      //ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 0.125mV
                                                      //ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.03125mV
                                                      //ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.015625mV
                                                      //ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.0078125mV

  ads.setGain(GAIN_ONE);                             // 1x gain   +/- 4.096V  1 bit = 0.125mV
  adcv = ads.readADC_SingleEnded(VLTG_SNSR);
  raw_voltage = ads.computeVolts(adcv) * Sns_Volt_Fact;                   // Por ampl. dif. para sensado remoto de (Max. 30V).
  voltage = raw_voltage * Sns_Volt_Calib_Fact + Sns_Volt_Calib_Offs;        // Calibracion fina de voltaje


  adci = ads.readADC_SingleEnded(CRR_SNSR);
  raw_current = ads.computeVolts(adci) * Sns_Curr_Fact;                   // Por ampl. dif. para sensado remoto de (Max. 5A).
  current = raw_current  * Sns_Curr_Calib_Fact + Sns_Curr_Calib_Offs;       // Calibracion fina de corriente
  #else

  int potValue = analogRead(A3);
  static float sim_Resistance = 1000;
  sim_Resistance = map(potValue, 0, 1023, 10, 1000) / 10;  // Convertir el rango 0-1023 a 1000-0.1 ohms
  current = (voltage / sim_Resistance) * Out_Curr_Calib_Fact + Out_Curr_Calib_Offs;                                // Asigna el valor de v simulado
  if (current >= setcurrent){
    current = setcurrent;
    voltage = (sim_Resistance * current) * Out_Volt_Calib_Fact + Out_Volt_Calib_Offs;
  } else {
    voltage = setvoltage * Out_Volt_Calib_Fact + Out_Volt_Calib_Offs;
  }

  #endif
}

//------------------------------------------- Set output voltage and max current ----------------------------------------------
void Set_Voltage_Current(bool setVI) {

  #ifndef WOKWI_SIMULATION
  float Ctrl_Volt = 0;
  float Ctrl_Curr = 0;

  if (Mode == 'V' && reading != setvoltage && !setVI){
    setvoltage = reading;
    Ctrl_Volt = setvoltage * Out_Volt_Fact * Out_Volt_Calib_Fact + Out_Volt_Calib_Offs; // Calcula valor de salida para el DacV con los factores y offset

    dacV.setVoltage(Ctrl_Volt, false);                                                  // Setea el voltage de salida por el factor y POR EL MOMENTO, no lo graba en la Eprom del DacV.
    Show_VI_Settings(); // Ver si es necesario.
    return;
  }

  if (Mode == 'I' && reading != setcurrent && !setVI){
    setcurrent = reading;
    Ctrl_Curr = setcurrent * Out_Curr_Fact * Out_Curr_Calib_Fact + Out_Curr_Calib_Offs; // Calcula valor de salida para el DacI con los factores y offset
    dacI.setVoltage(Ctrl_Curr, false);                                                  // Setea corriente máxima de salida por el factor y POR EL MOMENTO, no lo graba en la Eprom del DacI.
    Show_VI_Settings();  // Ver si es necesario.
    return;
  }

  if (setVI) {
    Ctrl_Volt = setvoltage * Out_Volt_Fact * Out_Volt_Calib_Fact + Out_Volt_Calib_Offs; // Calcula valor de salida para el DacV con los factores y offset
    Ctrl_Curr = setcurrent * Out_Curr_Fact * Out_Curr_Calib_Fact + Out_Curr_Calib_Offs; // Calcula valor de salida para el DacI con los factores y offset
    dacV.setVoltage(Ctrl_Volt, false);                                                  // Setea el voltage de salida por el factor y POR EL MOMENTO, no lo graba en la Eprom del DacV.
    dacI.setVoltage(Ctrl_Curr, false);                                                  // Setea corriente máxima de salida por el factor y POR EL MOMENTO, no lo graba en la Eprom del DacI.
    return;
  }
  #else
  if (Mode == 'V' && reading != setvoltage && !setVI){
    setvoltage = setvalue / 1000;
    Show_VI_Settings();}
  if (Mode == 'I' && reading != setcurrent && !setVI){
    setcurrent = setvalue / 1000;
    Show_VI_Settings();}
  if (setVI) {
    voltage = setvoltage * Out_Volt_Calib_Fact + Out_Volt_Calib_Offs;
    //current = setcurrent * Out_Curr_Calib_Fact + Out_Curr_Calib_Offs;
  }
  #endif
}

//------------------------------------------------- Memory Selection ----------------------------------------------------------
void Mem_selec(char key) {
  bool setVI = true;

  if (key == '1') {
    setvoltage = 3.3;
    setcurrent = 1.0;
  }
  else if (key == '2') {
    setvoltage = 5;
    setcurrent = 2.0;
  }
  else if (key == '3') {
    setvoltage = 9;
    setcurrent = 3.0;
  }
  else if (key == '4') {
    setvoltage = 12;
    setcurrent = 3.0;
  }
  else if (key == '5') {
    setvoltage = 16.8;
    setcurrent = 4.0;
  }
  else if (key == '6') {
    setvoltage = 24;
    setcurrent = 5.0;
  }
  else if (key == '0') {
    Calibration_Menu();
    setVI = false;
    return;
  }
  else {
    setVI = false;
    Mnsg = "Empty";
  }

  if (setVI == true) {
    Change_Mode('B');
  }
  mem_st = false;
}

//--------------------------------------- Graba en la EEPROM factores de Calibration ------------------------------------------
void Save_EEPROM(int address, float value) {
  EEPROM.put(address, value);                       // Guardar el valor en la EEPROM
  Serial.print(F("Saved in "));
  Serial.print(address);
  Serial.print(F(": "));
  Serial.println(value, 4);
}

//--------------------------------------- Leer desde la EEPROM un valor de Calibration ------------------------------------------
void Load_EEPROM(int address, float &variable) {
  EEPROM.get(address, variable);

  if (isnan(variable)) {  // Si el valor es NaN, significa que la EEPROM no estaba inicializada
      variable = (address == ADD_SNS_VOLT_OFF_CAL || address == ADD_SNS_CURR_OFF_CAL || 
                  address == ADD_OUT_VOLT_OFF_CAL || address == ADD_OUT_CURR_OFF_CAL) ? 0.0 : 1.0;

      EEPROM.put(address, variable);  // Guardar el valor por defecto en la EEPROM

      Serial.print(F("EEPROM nan at "));
      Serial.print(address);
      Serial.println(F(" fixed"));
    }

  Serial.print(F("Read from "));
  Serial.print(address);
  Serial.print(F(": "));
  Serial.println(variable, 4);
}

//----------------------------------------- Validar Calibración y grabar en EEPROM --------------------------------------------
void Save_Calibration() {
  bool cng_flag = false;
  float eeprom_read_Cal;

  // Lista de direcciones y variables a verificar
  struct CalibrationData {
      int address;
      float &variable;
      const char *name;
  };

  CalibrationData data[] = {
      {ADD_SNS_VOLT_FAC_CAL, Sns_Volt_Calib_Fact, "Sns_Volt_Calib_Fact"},
      {ADD_SNS_CURR_FAC_CAL, Sns_Curr_Calib_Fact, "Sns_Curr_Calib_Fact"},
      {ADD_OUT_VOLT_FAC_CAL, Out_Volt_Calib_Fact, "Out_Volt_Calib_Fact"},
      {ADD_OUT_CURR_FAC_CAL, Out_Curr_Calib_Fact, "Out_Curr_Calib_Fact"},
      {ADD_SNS_VOLT_OFF_CAL, Sns_Volt_Calib_Offs, "Sns_Volt_Calib_Offs"},
      {ADD_SNS_CURR_OFF_CAL, Sns_Curr_Calib_Offs, "Sns_Curr_Calib_Offs"},
      {ADD_OUT_VOLT_OFF_CAL, Out_Volt_Calib_Offs, "Out_Volt_Calib_Offs"},
      {ADD_OUT_CURR_OFF_CAL, Out_Curr_Calib_Offs, "Out_Curr_Calib_Offs"},
  };

  // Verificar y guardar solo si el valor ha cambiado
  for (CalibrationData &d : data) {
      Load_EEPROM(d.address, eeprom_read_Cal);
      if (abs(d.variable - eeprom_read_Cal) > 0.0001) {
          Save_EEPROM(d.address, d.variable);
          Serial.print(d.name);
          Serial.println(F(" saved"));
          cng_flag = true;
      }
  }
  Mnsg = cng_flag ? "Cal saved!" : "Cal is Ok";
}

//--------------------------------------- Cargar todos los valores de Calibration desde EEPROM ------------------------------------------
void Load_Calibration() {
  struct CalibrationData {
      int address;
      float &variable;
      const char *name;
      bool isOffset;  // Indica si es un OFF_CAL (true) o un FAC_CAL (false)
  };

  CalibrationData data[] = {
      {ADD_SNS_VOLT_FAC_CAL, Sns_Volt_Calib_Fact, "Sns_Volt_Calib_Fact", false},
      {ADD_SNS_CURR_FAC_CAL, Sns_Curr_Calib_Fact, "Sns_Curr_Calib_Fact", false},
      {ADD_OUT_VOLT_FAC_CAL, Out_Volt_Calib_Fact, "Out_Volt_Calib_Fact", false},
      {ADD_OUT_CURR_FAC_CAL, Out_Curr_Calib_Fact, "Out_Curr_Calib_Fact", false},
      {ADD_SNS_VOLT_OFF_CAL, Sns_Volt_Calib_Offs, "Sns_Volt_Calib_Offs", true},
      {ADD_SNS_CURR_OFF_CAL, Sns_Curr_Calib_Offs, "Sns_Curr_Calib_Offs", true},
      {ADD_OUT_VOLT_OFF_CAL, Out_Volt_Calib_Offs, "Out_Volt_Calib_Offs", true},
      {ADD_OUT_CURR_OFF_CAL, Out_Curr_Calib_Offs, "Out_Curr_Calib_Offs", true},
  };

  for (CalibrationData &d : data) {
      Load_EEPROM(d.address, d.variable);

      if (!d.isOffset) {  // Es un FAC_CAL (factor de calibración)
          if (d.variable < 0.9 || d.variable > 1.1) {
              Serial.print(F("Fact_out_of_range in "));
              Serial.print(d.address);
              Serial.println(F(" Using default (1.0)"));
              d.variable = 1.0;  // Restaurar al valor por defecto
              Mnsg = "Df Fac Cal";
          } else {
              Serial.print(F("Fact__Cal in "));
              Serial.print(d.address); Serial.print(F(": "));
              Serial.println(d.variable, 4);
          }
      } else {  // Es un OFF_CAL (offset de calibración)
          if (d.variable < -0.1 || d.variable > 0.1) {
              Serial.print(F("Off_out_of_range in "));
              Serial.print(d.address);
              Serial.println(F(" Using default (0.0)"));
              d.variable = 0.0;  // Restaurar al valor por defecto
              Mnsg = "Df Off Cal";
          } else {
              Serial.print(F("Off_Cal in "));
              Serial.print(d.address); Serial.print(F(": "));
              Serial.println(d.variable, 4);
          }
      }
  }
}

// ------------------------------------- Cambiar de modos y presets correspondientes ------------------------------------------
void Change_Mode(char Modetoset) {
  if (Modetoset == 'I') {
    Mode = 'I';
    r = 0; y = 10; CP = 10; CPprev = 10;  factor = 1000;  // Cambio de modo, reubicación de factores y pos. de cursor
    reading = setcurrent;
    setvalue = setcurrent * 1000;
  }
  else if(Modetoset == 'B'){
    Set_Voltage_Current(true);
    if (Mode == 'V') {
      reading = setvoltage;
      setvalue = setvoltage * 1000;
    }
    else{
      reading = setcurrent;
      setvalue = setcurrent * 1000;
    }
  }
  else {
    Mode = 'V';
    r = 0; y = 1; CP = 1; CPprev = 1;  factor = 1000;     // Cambio de modo, reubicación de factores y pos. de cursor
    reading = setvoltage;
    setvalue = setvoltage * 1000;
  }
  Reset_Input_Value(); // Reseto cualquier valor que se haya estado ingresando
}

// ------------------------------------------------ Menu de Calibración  ------------------------------------------------------
void Calibration_Menu(void) {
  Output_Control(false);
  Reset_Input_Value();
  mem_st = false;
  int menuIndex = 0;
  const String menuItems[] = {"Cal V", "Cal I", "Load", "Save", "Exit"};
  const int menuSize = sizeof(menuItems) / sizeof(menuItems[0]);

  lcd.noBlink();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Calibration Menu"));

  while (true) {
      // Mostrar la opción seleccionada
      lcd.setCursor(0, 1);
      lcd.print("> ");
      lcd.print(menuItems[menuIndex]);
      lcd.print("   "); // Espacios para borrar caracteres anteriores

      // Leer entrada del teclado
      char key = customKeypad.getKey();

      if (key == 'U' || key == '<') {         // Flecha arriba o Izquierda
          menuIndex = (menuIndex > 0) ? menuIndex - 1 : menuSize - 1;
          delay(200); // Pequeña pausa para evitar rebotes
      } 
      else if (key == 'D' || key == '>') {    // Flecha abajo o derecha
          menuIndex = (menuIndex < menuSize - 1) ? menuIndex + 1 : 0;
          delay(200); // Pequeña pausa para evitar rebotes
      } 
      else if (key == 'E') { // Enter
        delay(200); // Pequeña pausa para evitar rebotes
        switch (menuIndex) {
          case 0: Start_Calibration('V'); return;
          case 1: Start_Calibration('I'); return;
          case 2: // Cargar calibración de EEPROM
                  Load_Calibration();
                  lcd.clear();
                  lcd.setCursor(0, 0);
                  lcd.print("Load OK");
                  delay(1000);
                  return;
          case 3: // Guardar calibración en EEPROM
                  Save_Calibration();
                  lcd.clear();
                  lcd.setCursor(0, 0);
                  lcd.print("Save OK");
                  delay(1000);
                  return;
          case 4: // Salir del menú
                  return;
        }
      }
  }
}

// ------------------------------------------------ Menu de Calibración  ------------------------------------------------------
void Start_Calibration(char mode) {
  cal_st = true;
  Mode = mode;      // Definimos si calibramos Voltaje o Corriente antes de P1
  Modetocal = mode; // Registra el modo a Calibrar

  setvoltage = 2;
  setcurrent = 0.1;
  Change_Mode('B');
  Output_Control(true);

  Mnsg = F("     <Set ");
  Req_info = (Modetocal == 'V') ? F("v1 Cal") : F("i1 Cal"); 

  // Inicializar factores de calibración SOLO para el parámetro que vamos a calibrar
  if (Modetocal == 'V') {
      Sns_Volt_Calib_Fact = 1.0;
      Sns_Volt_Calib_Offs = 0.0;
      Out_Volt_Calib_Fact = 1.0;
      Out_Volt_Calib_Offs = 0.0;
  } else {
      Sns_Curr_Calib_Fact = 1.0;
      Sns_Curr_Calib_Offs = 0.0;
      Out_Curr_Calib_Fact = 1.0;
      Out_Curr_Calib_Offs = 0.0;
  }
}

// ------------------------------------------------ Calibración Nuevo --------------------------------------------------------
void Calibration(void) {
  static bool firstPoint = true;
  static float x1 = 0, y1 = 0, x2 = 0, y2 = 0;
  bool sns_ok, out_ok;

  if (x <= 0 || Mode != Modetocal) {
    Mnsg = "Cal cancel";
    Req_info = "      ";
    cal_st = false;
    firstPoint = true;
    Load_Calibration(); // Restaurar valores desde EEPROM
    return;
  }

  if (firstPoint) {
    x1 = x;
    y1 = (Modetocal == 'V') ? voltage : current;
    firstPoint = false;
    Serial.print(x1);
    Serial.println(y1);
    // Configurar el segundo punto de calibración
    setvoltage = (Modetocal == 'V') ? 28 : 20;
    setcurrent = (Modetocal == 'I') ? 4 : 0.1;
    Change_Mode('B');
    Mnsg = F("     <Set ");
    Req_info = (Modetocal == 'V') ? F("v2 Cal") : F("i2 Cal"); 
  } else {
    x2 = x;
    y2 = (Modetocal == 'V') ? voltage : current;
    Serial.print(x2);
    Serial.println(y2);
    if (Modetocal == 'V') {
      sns_ok = Calc_Calib_Fact(x1, y1, x2, y2, Sns_Volt_Calib_Fact, Sns_Volt_Calib_Offs);
      out_ok = Calc_Calib_Fact(y1, x1, y2, x2, Out_Volt_Calib_Fact, Out_Volt_Calib_Offs);
    } else {
      sns_ok = Calc_Calib_Fact(x1, y1, x2, y2, Sns_Curr_Calib_Fact, Sns_Curr_Calib_Offs);
      out_ok = Calc_Calib_Fact(y1, x1, y2, x2, Out_Curr_Calib_Fact, Out_Curr_Calib_Offs);
    }

      Serial.print(F("Cal for Mode ")); 
      Serial.println(Modetocal);
      Serial.print(F("Sns Fact: ")); Serial.println((Modetocal == 'V') ? Sns_Volt_Calib_Fact : Sns_Curr_Calib_Fact, 4);
      Serial.print(F("Sns Offs: ")); Serial.println((Modetocal == 'V') ? Sns_Volt_Calib_Offs : Sns_Curr_Calib_Offs, 4);
      Serial.print(F("Out Fact: ")); Serial.println((Modetocal == 'V') ? Out_Volt_Calib_Fact : Out_Curr_Calib_Fact, 4);
      Serial.print(F("Out Offs: ")); Serial.println((Modetocal == 'V') ? Out_Volt_Calib_Offs : Out_Curr_Calib_Offs, 4);

  // Verificar si la calibración quedó fuera de rango
  if (!sns_ok || !out_ok) {
    Mnsg = (Modetocal == 'V') ? "V Cal Fail" : "I Cal Fail";
  } else {
    Mnsg = (Modetocal == 'V') ? "V Cal Done" : "I Cal Done";
  }
  Req_info = "      ";
  // Reiniciar calibración
  cal_st = false;
  firstPoint = true;
  Modetocal = 'U';
  }
}

// ------------------------------------------------ Calculo de factores --------------------------------------------------------
bool Calc_Calib_Fact(float x1, float y1, float x2, float y2, float &factor, float &offset) {
  factor = max(0.9f, min(1.1f, (x2 - x1) / (y2 - y1))); // Calcula y limita factor
  offset = max(-0.1f, min(0.1f, x1 - factor * y1));     // Calcula y limita offset

  // Retorna false si el factor o el offset quedaron en el límite
  return !(factor == 0.9f || factor == 1.1f || offset == -0.1f || offset == 0.1f);
}

// ----------------- Output Control ---------------------------
void Output_Control(bool enable) {

  if (enable) digitalWrite(MSFT_CTRL, HIGH);
  else {digitalWrite(MSFT_CTRL, LOW);}

}
#include "variables.h"
#include "funciones.h"
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
  
  //--------------------------------------------- Leer boton de encoder y acciónar ----------------------------------------------
  void Read_encoder_btn(void) {
    if (digitalRead(ENC_BTN) == LOW) {
      delay(200);                                               // Simple key bounce delay
      CP++;                                                     // Avanzo el cursor con el botón del encoder
      ToggleDisplaySettings();
    }
  }
  
  //--------------------------------------------------- Read Keypad Input -------------------------------------------------------
  void Read_keypad (void) {
    customKey = customKeypad.getKey();
    byte index_max = 5;
    
    if (customKey != NO_KEY){
  
      ToggleDisplaySettings();                                // Indica que hubo un cambio en el seteo
  
      if(customKey == 'V'){                                   // Pasar a Modo CV
        Mode = "V";
        r = 0; y = 1; CP = 1; CPprev = 1;  factor = 1000;     // Cambio de modo, reubicación de factores y pos. de cursor
        reading = setvoltage;
        setvalue = setvoltage * 1000;
        decimalPoint = (' ');                                 // clear decimal point text character reset
        }
              
      if(customKey == 'I'){                                   // Pasar a Modo CI
        Mode = "I";
        r = 0; y = 10; CP = 10; CPprev = 10;  factor = 1000;  // Cambio de modo, reubicación de factores y pos. de cursor
        reading = setcurrent;
        setvalue = setcurrent * 1000;
        decimalPoint = (' ');                                 // Clear decimal point text character reset
        }
            
      if(customKey == 'M'){                                   // Seleccion de memorias
        mem_st = !mem_st;
        if (cal_st){mem_st = false;}                          // Si esta en modo calibración, no entre a modo Preset en Memoria
        if (mem_st){Req_info = "Preset"; r = 1; y = index;}   // Avisa que espera el preset de memoria
        if (!mem_st && !cal_st){Req_info = "      ";}         // Limpia el mensaje de espera
        }
  
      if(customKey >= '0' && customKey <= '9'){               // Para teclas del 0 al 9....
        if (cal_st){index_max = 6;}                           // Si esta en modo calibración, extiendo el index para valores mas prescisos 
          else {index_max = 4;}
        index = min(index_max, max(0, index));                // El valor de index puede estar entre 0 y 5 o 7 si esta en modo Calibración
        numbers[index++] = customKey;                         
        numbers[index] = '\0';                                // Agrega caracter nulo para indicar fin de la cadena de caranteres.
        Mnsg = numbers;                                       // Muestro carga
        r = 1; y = index;                                     // Resposicion del cursor
        }
  
      if(customKey == '.'){                                   // Check if decimal button key pressed
        if (decimalPoint != ('*')){                           // Test if decimal point entered twice - if so skip 
          numbers[index++] = '.';
          numbers[index] = '\0';
          Mnsg = numbers;
          r = 1; y = index;                                   // Resposicion del cursor
          decimalPoint = ('*');                               // Used to indicate decimal point has been input
          }
        }
  
      if(customKey == 'E') {                                  // Tecla "Enter" carga el valor en el Set que corresponda
        x = atof(numbers);
        if (mem_st && !cal_st){
          Mem_selec();
          }
        else if (cal_st) {Calibration();}                     // Calibra factores
        else{
          reading = x;
          setvalue = reading * 1000;
          Mnsg = Mode + " set!";
          }
        index = 0;
        numbers[index] = '\0';
        decimalPoint = (' ');                                 // Clear decimal point test character reset
        r = 0; y = 8;                                         // Reseteo posición del cursor reseltando el modo seteado
        digitalWrite(MSFT_CTRL, HIGH);                        // Habilito MOSFET si estaba deshabilitado  
        }
  
      if(customKey == 'C'){                                   // Borra valores ingresados
        ResetInputValue();
      }
  
      if(customKey == 'U'){
        setvalue = setvalue + factor;                        // Update setvalue Up
        setvalue = min(ENCODER_MAX, max(0, setvalue));
        r = 0; y = CP;                                       // Muevo el Cursor en renglòn de V e I y en la posición de CP  
        }
                  
      if(customKey == 'D'){
        setvalue = setvalue - factor;
        setvalue = min(ENCODER_MAX, max(0, setvalue));       // Update setvalue Down
        r = 0; y = CP;                                       // Muevo el Cursor en renglòn de V e I y en la posición de CP
        }
        
      if(customKey == '<'){CP--;}                            // Cursor a la Izquierda
      
      if(customKey == '>'){CP++;}                            // Cursor a la derecha
    }
  }
  
  //------------------------------------------------------ Clear Input ----------------------------------------------------------
  void ResetInputValue() {
      index = 0;                                              // Resetear index de entrada de valores
      numbers[index] = '\0';                                  // Resetar entrada de valores
      decimalPoint = (' ');                                   // Clear decimal point test character reset
      Mnsg = "          ";                                    // Limpiar zona de mensajes
      r = 1; y = 0;                                           // Reseteo posición del cursor
  }
  
  //---------------------------------------------------- Cursor Position --------------------------------------------------------
  void Cursor_position(void) {
  
    if (CP != CPprev) {
      if (Mode == "V") {                                        // Si esta en Modo V:
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
      
      if (Mode == "I") {
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
  void ToggleDisplaySettings (void) {                              
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
      ResetInputValue();                                            // Reseteo valor ingresado
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
    if (Mode == "V"){lcd.print((char)0x7F);} else{lcd.print(" ");}  // Flecha hacia la V
    lcd.print("C");
    lcd.print(Mode);
    if (Mode == "I"){lcd.print((char)0x7E);} else{lcd.print(" ");}  // Flecha hacia la I
  
    lcd.setCursor(0,1);
  
    if (mensaje == "          "){
      lcd.print("       ");                                         // Borro restos de cualquier otro mensaje
      lcd.print(temp);                                              // Mostrar temperatura si no hubo mensajes
      lcd.print((char)0xDF);                                        // Signo de Grados
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
    static float maxpwrdis;                               // Potencia máxima de disipación calculada para el TIP3055, 90W para temp. ambiente
    
    if (temp >= 100) {                                    // Controlar el encendido de los fans y el temporizador
      digitalWrite(MSFT_CTRL, LOW);                       // Deshabilito MOSFET y este a los TIP3055
      Mnsg = "OFF Max T ";
    }
  
    if (new_temp){                                        // Si hay un nuevo valor de temperatura...
      maxpwrdis = max(0, min(90, 108 - 0.72 * temp));    // Recalculo la potencia máxima que se podrá con un TIP3055, limitada entre 90 y 0
      new_temp = false;                                   // para no volver a recalcular maxpwrdis
    }
  
    actpwrdis = max(0,(38 - voltage) * current / 2);      // Con corriente de mas de 2A baja la tensión de 41 a 38. La corriente se divide por dos TIP3055 y no puede ser menor que 0
  
    if (actpwrdis >= maxpwrdis) {
      digitalWrite(MSFT_CTRL, LOW);                       // Deshabilito MOSFET y este a los TIP3055
      Mnsg = "OFF Max WD";
    }
  
    if (Mode =="V" && reading > VOLTS_CUTOFF) {          // Si en Modo V y la lectura supera el máximo
      reading = VOLTS_CUTOFF;                            // Mantengo el valor a mostrar en máximo
      setvalue = VOLTS_CUTOFF * 1000;                    // Lo mismo para el setvalue
      Mnsg = "Max V";                                     // Notifico
    }
  
    if (Mode =="I" && reading > CURRENT_CUTOFF) {          // Si en Modo I y la lectura supera el máximo
      reading = CURRENT_CUTOFF;                            // Mantengo el valor a mostrar en máximo
      setvalue = CURRENT_CUTOFF * 1000;                    // Lo mismo para el setvalue
      Mnsg = "Max I";                                     // Notifico
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

    if (cal_st){Sns_Volt_Calib_Fact = 1.0; Sns_Volt_Calib_Offs = 0.0;}        // Si estoy en modo Calibración, reseteo a 1 el factor para poder leer el valor sin calibrar
    voltage = raw_voltage * Sns_Volt_Calib_Fact + Sns_Volt_Calib_Offs;        // Calibracion fina de voltaje


    adci = ads.readADC_SingleEnded(CRR_SNSR);
    raw_current = ads.computeVolts(adci) * Sns_Curr_Fact;                   // Por ampl. dif. para sensado remoto de (Max. 5A).
    if (cal_st){Sns_Curr_Calib_Fact = 1.0; Sns_Curr_Calib_Offs = 0.0;}        // Si estoy en modo Calibración, reseteo a 1 el factor para poder leer el valor sin calibrar
    current = raw_current  * Sns_Curr_Calib_Fact + Sns_Curr_Calib_Offs;       // Calibracion fina de corriente
    #else
    voltage = setvoltage * Out_Volt_Calib_Fact + Out_Volt_Calib_Offs;
    current = setcurrent * Out_Curr_Calib_Fact + Out_Curr_Calib_Offs;

    if (Mode == "B") {
      voltage = setvoltage * Out_Volt_Calib_Fact + Out_Volt_Calib_Offs;
      current = setcurrent * Out_Curr_Calib_Fact + Out_Curr_Calib_Offs;
  }

    #endif
  }
  
  //------------------------------------------- Set output voltage and max current ----------------------------------------------
  void Set_Voltage_Current() {
  
    #ifndef WOKWI_SIMULATION
    float Ctrl_Volt = 0;
    float Ctrl_Curr = 0;
  
    if (Mode == "V" && reading != setvoltage){
      setvoltage = reading;
      if (cal_st){Out_Volt_Calib_Fact = 1.0; Out_Volt_Calib_Offs = 0.0;}                  // Si estoy en modo Calibración, factor = 1 y Offset = 0 para poder leer el Voltage sin calibrar
      Ctrl_Volt = setvoltage * Out_Volt_Fact * Out_Volt_Calib_Fact + Out_Volt_Calib_Offs; // Calcula valor de salida para el DacV con los factores y offset

      dacV.setVoltage(Ctrl_Volt, false);                                                // Setea el voltage de salida por el factor y POR EL MOMENTO, no lo graba en la Eprom del DacV.
      ToggleDisplaySettings();                                                            // Indica que hubo un cambio de seteo
    }
  
    if (Mode == "I" && reading != setcurrent){
      setcurrent = reading;
      if (cal_st){Out_Curr_Calib_Fact = 1.0; Out_Curr_Calib_Offs = 0.0;}                  // Si estoy en modo Calibración, reseteo factor para poder ver el valor sin calibrar
      Ctrl_Curr = setcurrent * Out_Curr_Fact * Out_Curr_Calib_Fact + Out_Curr_Calib_Offs; // Calcula valor de salida para el DacI con los factores y offset
      dacI.setVoltage(Ctrl_Curr, false);                                                // Setea corriente máxima de salida por el factor y POR EL MOMENTO, no lo graba en la Eprom del DacI.
      ToggleDisplaySettings();
    }
  
    if (Mode == "B") {
      digitalWrite(MSFT_CTRL, LOW);                                                       // Deshabilito MOSFET, porque voy a setear tanto V como I. Se debe habililitar luego
      if (cal_st){
        Out_Volt_Calib_Fact = 1.0; Out_Volt_Calib_Offs = 0.0;                             // Si estoy en modo Calibración, factor = 1 y Offset = 0 para poder setear el Voltage sin calibrar
        Out_Curr_Calib_Fact = 1.0; Out_Curr_Calib_Offs = 0.0;                             // Si estoy en modo Calibración, factor = 1 y Offset = 0 para poder setear la Correinte sin calibrar
      }   
      Ctrl_Volt = setvoltage * Out_Volt_Fact * Out_Volt_Calib_Fact + Out_Volt_Calib_Offs; // Calcula valor de salida para el DacV con los factores y offset
      Ctrl_Curr = setcurrent * Out_Curr_Fact * Out_Curr_Calib_Fact + Out_Curr_Calib_Offs; // Calcula valor de salida para el DacI con los factores y offset
      dacV.setVoltage(Ctrl_Volt, false);                                                  // Setea el voltage de salida por el factor y POR EL MOMENTO, no lo graba en la Eprom del DacV.
      dacI.setVoltage(Ctrl_Curr, false);                                                  // Setea corriente máxima de salida por el factor y POR EL MOMENTO, no lo graba en la Eprom del DacI.
    }
    #else
    if (Mode == "V" && reading != setvoltage){
        setvoltage = setvalue / 1000;
        ToggleDisplaySettings();}
    if (Mode == "I" && reading != setcurrent){
        setcurrent = setvalue / 1000;
        ToggleDisplaySettings();}
    #endif
  }
  
  //------------------------------------------------- Memory Selection ----------------------------------------------------------
  void Mem_selec(void) {
    bool setVI = true;
    String lst_Mode = Mode;
    if (x == 1.0) {
      setvoltage = 3.3;
      setcurrent = 1.0;
    }
    else if (x == 2.0) {
      setvoltage = 5;
      setcurrent = 2.0;
    }
    else if (x == 3.0) {
      setvoltage = 9;
      setcurrent = 3.0;
    }
    else if (x == 4.0) {
      setvoltage = 12;
      setcurrent = 3.0;
    }
    else if (x == 5.0) {
      setvoltage = 16.8;
      setcurrent = 4.0;
    }
    else if (x == 6.0) {
      setvoltage = 24;
      setcurrent = 5.0;
    }
    else if (x == 99.0) {
      Calibration();
      setVI = false;
    }
    else if (x == 100.0) {
      Save_Calibration();  
      setVI = false;
    }
    else {
      setVI = false;
      Mnsg = "Error";
    }
  
    if (setVI == true) {
      Mode = "B";
      Set_Voltage_Current();
      Mode = lst_Mode;
      if (Mode == "V"){
        reading = setvoltage;
        setvalue = setvoltage * 1000;
        }
      else {
        reading = setcurrent;
        setvalue = setcurrent * 1000;
        }
      if (!cal_st) {Mnsg = "Mem Set!  ";}
    }
    mem_st = false;
  }
  
  //----------------------------------------------------- Calibration -----------------------------------------------------------
  void Calibration() {
    
    float Ctrl_Volt = 0;
    float Ctrl_Curr = 0;

    static float x1 = 0, ys1 = 0, yo1 = 0, x2 = 0, ys2 = 0, yo2 = 0;
    static bool firstPoint = true;
    static bool cal_call = true;
  
  
    if(cal_call) {
        cal_st = true;
        x1 = 0; ys1 = 0; yo1 = 0; x2 = 0; ys2 = 0; yo2 = 0;         //Resetero por si se llamo antes a la función.
        setvoltage = 2;
        setcurrent = 0.1;
        Mode = "B";
        Set_Voltage_Current();
        Mode = "V";
        reading = setvoltage;
        setvalue = setvoltage * 1000;
        cal_call = false;
        firstPoint = true;
        Mnsg = "      Set ";
        Req_info = "P1 Cal";
        return;
    }
  
    if (x <= 0) {                                                   // Validar que 'x' sea mayor que 0
      Serial.println(F("null: Cal canceled"));
      Mnsg = "Cal cancel";                                          // Notifico
      firstPoint = true;
      cal_st = false;
      cal_call = true;
      x1 = 0; ys1 = 0; yo1 = 0; x2 = 0; ys2 = 0; yo2 = 0;           //Resetero por si se llamo antes a la función.
      // Como cancelo la calibración, vuelco a cargar los valores grabados en EEPROM
      LoadCalibration(ADD_SNS_VOLT_FAC_CAL, Sns_Volt_Calib_Fact);    // Carga Factor de Calibración de la EEPROM para sensado de Voltaje
      LoadCalibration(ADD_SNS_CURR_FAC_CAL, Sns_Curr_Calib_Fact);    // Carga Factor de Calibración de la EEPROM para sensado de Corriente
      LoadCalibration(ADD_OUT_VOLT_FAC_CAL, Out_Volt_Calib_Fact);    // Carga Factor de Calibración de la EEPROM para seteo de Voltaje
      LoadCalibration(ADD_OUT_CURR_FAC_CAL, Out_Curr_Calib_Fact);    // Carga Factor de Calibración de la EEPROM para seteo de Corriente
      LoadCalibration(ADD_SNS_VOLT_OFF_CAL, Sns_Volt_Calib_Offs);    // Carga Offset de Calibración de la EEPROM para seteo de Voltaje
      LoadCalibration(ADD_SNS_CURR_OFF_CAL, Sns_Curr_Calib_Offs);    // Carga Offset de Calibración de la EEPROM para seteo de Corriente
      LoadCalibration(ADD_OUT_VOLT_OFF_CAL, Out_Volt_Calib_Offs);    // Carga Offset de Calibración de la EEPROM para seteo de Voltaje
      LoadCalibration(ADD_OUT_CURR_OFF_CAL, Out_Curr_Calib_Offs);    // Carga Offset de Calibración de la EEPROM para seteo de Corriente
      return;
    }
  
    // Calibración para modo de Voltaje
    else if (Mode == "V") {
      if (firstPoint){
        x1 = x;                // Valor de referencia ingresado (usando multímetro)
        ys1 = voltage;         // Valor sensado
        yo1 = setvoltage;      // Valor seteado
        firstPoint = false;
        setvoltage = 28;
        setcurrent = 0.1;
        Mode = "B";
        Set_Voltage_Current();
        Mode = "V";
        reading = setvoltage;
        setvalue = setvoltage * 1000;
        Mnsg = "    Set V ";
        Req_info = "V2 Cal";
      }
      else{
        x2 = x;
        ys2 = voltage;
        yo2 = setvoltage;
        Sns_Volt_Calib_Fact = max(0.9, min(1.1, (x2 - x1) / (ys2 - ys1)));                  // Calcula factor de corrección de sensado usando el valorres de referencia ingresado (usando instrumento externo)
        Sns_Volt_Calib_Offs = max(-0.1, min(0.1, x1 - Sns_Volt_Calib_Fact * ys1));          // Calcula Offset de calibracion de voltage sensado
        Out_Volt_Calib_Fact = max(0.9, min(1.1, (yo2 - yo1) / (x2 - x1)));                  // Calcula factor de corrección de salida usando el valor de referencia ingresado (usando instrumento externo)
        Out_Volt_Calib_Offs = max(-0.1, min(0.1, x1 - Out_Volt_Calib_Fact * yo1));
        Ctrl_Volt = setvoltage * Out_Volt_Fact * Out_Volt_Calib_Fact + Out_Volt_Calib_Offs;
        #ifndef WOKWI_SIMULATION
        dacV.setVoltage(Ctrl_Volt, false);                                                  // Setea el voltage de salida por el nuevo factor para poder medirlo
        #endif
        // Informo resultados de Calibnación:
        Serial.print(F("Sns_Volt_Calib_Fact: ")); Serial.println(Sns_Volt_Calib_Fact, 4);
        Serial.print(F("Sns_Volt_Calib_Offs: ")); Serial.println(Sns_Volt_Calib_Offs, 4);
        Serial.print(F("Out_Volt_Calib_Fact: ")); Serial.println(Out_Volt_Calib_Fact, 4);
        Serial.print(F("Out_Volt_Calib_Offs: ")); Serial.println(Out_Volt_Calib_Offs, 4);
        cal_call = true;
        cal_st = false;
        firstPoint = true;                                                              // Reiniciar para futuras calibraciones
        Mnsg = "V Cal set";
        Req_info = "      ";
        LoadCalibration(ADD_SNS_CURR_FAC_CAL, Sns_Curr_Calib_Fact);                     // Carga Factor de Calibración de la EEPROM para sensado de Corriente, porque se reseteo en modo calibracion
        LoadCalibration(ADD_OUT_CURR_FAC_CAL, Out_Curr_Calib_Fact);                     // Carga Factor de Calibración de la EEPROM para seteo de Corriente, porque se reseteo en modo calibracion
        LoadCalibration(ADD_SNS_CURR_OFF_CAL, Sns_Curr_Calib_Offs);                     // Carga Offset de Calibración de la EEPROM para seteo de Corriente, porque se reseteo en modo calibracion
        LoadCalibration(ADD_OUT_CURR_OFF_CAL, Out_Curr_Calib_Offs);                     // Carga Offset de Calibración de la EEPROM para seteo de Voltaje
        return;
      }
    }
  
    // Calibración para modo de Corriente
    else if (Mode == "I") {
      if (firstPoint){
        x1 = x;                                                                         // Valor de referencia ingresado (usando multímetro)
        ys1 = current;                                                                  // Valor sensado
        yo1 = setcurrent;                                                               // Valor seteado
        firstPoint = false;
        setvoltage = 20;                                                                // Probado con los nuevos TIP3055 
        setcurrent = 4;                                                                 // Probado con los nuevos TIP3055 
        Mode = "B";
        Set_Voltage_Current();
        Mode = "I";
        reading = setcurrent;
        setvalue = setcurrent * 1000;
        Mnsg = "    Set I ";
        Req_info = "I2 Cal";
      }
      else{
        x2 = x;
        ys2 = current;
        yo2 = setcurrent;
        Sns_Curr_Calib_Fact = max(0.9, min(1.1, (x2 - x1) / (ys2 - ys1)));               // Calcula factor de corrección de sensado usando el valorres de referencia ingresado (usando instrumento externo)
        Sns_Curr_Calib_Offs = max(-0.1, min(0.1, x1 - Sns_Curr_Calib_Fact * ys1));       // Calcula Offset
        Out_Curr_Calib_Fact = max(0.9, min(1.1, (yo2 - yo1) / (x2 - x1)));               // Calcula factor de corrección de salida usando el valor de referencia ingresado (usando instrumento externo)
        Out_Curr_Calib_Offs = max(-0.1, min(0.1, x1 - Out_Curr_Calib_Fact * yo1));       // Calcula Offset
        Ctrl_Curr = setcurrent * Out_Curr_Fact * Out_Curr_Calib_Fact;
        #ifndef WOKWI_SIMULATION
        dacI.setVoltage(Ctrl_Curr, false);                                               // Setea la corriente de salida por el nuevo factor para poder medirlo
        #endif
        // Informo resultados de Calibnación:
        Serial.print(F("Sns_Curr_Calib_Fact: ")); Serial.println(Sns_Curr_Calib_Fact, 4);
        Serial.print(F("Sns_Curr_Calib_Offs: ")); Serial.println(Sns_Curr_Calib_Offs, 4);
        Serial.print(F("Out_Curr_Calib_Fact: ")); Serial.println(Out_Curr_Calib_Fact, 4);
        Serial.print(F("Out_Curr_Calib_Offs: ")); Serial.println(Out_Curr_Calib_Offs, 4);
        cal_call = true;
        cal_st = false;
        firstPoint = true;                                                               // Reiniciar para futuras calibraciones
        Mnsg = "I Cal set";
        Req_info = "      ";
        LoadCalibration(ADD_SNS_VOLT_FAC_CAL, Sns_Volt_Calib_Fact);                      // Carga Factor de Calibración de la EEPROM para sensado de Voltaje, porque se reseteo en modo calibracion
        LoadCalibration(ADD_OUT_VOLT_FAC_CAL, Out_Volt_Calib_Fact);                      // Carga Factor de Calibración de la EEPROM para seteo de Voltaje, porque se reseteo en modo calibracion
        LoadCalibration(ADD_SNS_VOLT_OFF_CAL, Sns_Volt_Calib_Offs);                      // Carga Offset de Calibración de la EEPROM para seteo de Voltaje, porque se reseteo en modo calibracion
        LoadCalibration(ADD_OUT_VOLT_OFF_CAL, Out_Volt_Calib_Offs);                      // Carga Offset de Calibración de la EEPROM para seteo de Voltaje
        return;
      }
    } 
  
    // Si el modo no es válido
    else {
      //Serial.println("Error: Modo no válido. Use 'V' o 'I'.");
      Mnsg = "No valid  ";                               // Notifico
    }
  }
  
  //--------------------------------------- Carga de la EEPRON factores de Calibration ------------------------------------------
  void LoadCalibration(int address, float &variable) {
    EEPROM.get(address, variable);                                          // Leer desde la EEPROM
  
    if (address != ADD_SNS_VOLT_OFF_CAL && address != ADD_SNS_CURR_OFF_CAL && address != ADD_OUT_VOLT_OFF_CAL && address != ADD_OUT_CURR_OFF_CAL) {   
      if (variable < 0.9 || variable > 1.1) {                                 // Validar que el valor esté dentro del rango válido (0.9 a 1.1)
        Serial.print(F("Fact_out_of_range in "));
        Serial.print(address);
        Serial.println(F("Using_default"));
        variable = 1.0;  // Restaurar el valor por defecto
        Mnsg = "Df Fac Cal";                                                // Notifico en Display
      }
      else {
          Serial.print(F("Fact__Cal in "));
          Serial.print(address); Serial.print(F(": "));
          Serial.println(variable, 4);
      }
    }
    else {
      if (variable < -0.1 || variable > 0.1) {                              // Validar que el valor esté dentro del rango válido (-0.1 a 0.1)
        Serial.print("Off_out_of_range in ");
        Serial.print(address);
        Serial.println(F("Using_default"));
        variable = 0.0;  // Restaurar el valor por defecto
        Mnsg = "Df Off Cal";                                                // Notifico en Display
      }
      else{
          Serial.print(F("Off_Cal in "));
          Serial.print(address); Serial.print(F(": "));
          Serial.println(variable, 4);
      }
    }
  }
  
  //--------------------------------------- Graba en la EEPROM factores de Calibration ------------------------------------------
  void SaveCalibration(int address, float value) {
    EEPROM.put(address, value);                       // Guardar el valor en la EEPROM
    Serial.print(F("Saved in "));
    Serial.print(address);
    Serial.print(F(": "));
    Serial.println(value, 4);
  }
  
  //----------------------------------------- Validar Calibración y grabar en EEPROM --------------------------------------------
  void Save_Calibration() {
    bool cng_flag = false;
    float eeprom_read_Cal;
  
    EEPROM.get(ADD_SNS_VOLT_FAC_CAL, eeprom_read_Cal);                // Leer el valor de calibración guardado en EEPROM para Sns_Volt_Calib_Fact
    if (abs(Sns_Volt_Calib_Fact - eeprom_read_Cal) > 0.0001) {        // Verifica con tolerancia
      SaveCalibration(ADD_SNS_VOLT_FAC_CAL, Sns_Volt_Calib_Fact);     // Guardar nuevo valor
      Serial.println(F("Sns_Volt_Calib_Fact saved"));
      cng_flag = true;                                                // Flag de que se grabo un valor
    }
   
    EEPROM.get(ADD_SNS_CURR_FAC_CAL, eeprom_read_Cal);                // Leer el valor de calibración guardado en EEPROM para Sns_Curr_Calib_Fact
    if (abs(Sns_Curr_Calib_Fact - eeprom_read_Cal) > 0.0001) {        // Verifica con tolerancia
      SaveCalibration(ADD_SNS_CURR_FAC_CAL, Sns_Curr_Calib_Fact);     // Guardar nuevo valor
      Serial.println(F("Sns_Curr_Calib_Fact saved"));
      cng_flag = true;                                                // Flag de que se grabo un valor
    }
  
    // Comprobar y guardar Out_Volt_Calib_Fact
    EEPROM.get(ADD_OUT_VOLT_FAC_CAL, eeprom_read_Cal);                // Leer el valor de calibración guardado en EEPROM para Out_Volt_Calib_Fact
    if (abs(Out_Volt_Calib_Fact - eeprom_read_Cal) > 0.0001) {        // Verifica con tolerancia
      SaveCalibration(ADD_OUT_VOLT_FAC_CAL, Out_Volt_Calib_Fact);     // Guardar nuevo valor
      Serial.println(F("Out_Volt_Calib_Fact saved"));
      cng_flag = true;                                                // Flag de que se grabo un valor
    }
  
    // Comprobar y guardar Out_Curr_Calib_Fact
    EEPROM.get(ADD_OUT_CURR_FAC_CAL, eeprom_read_Cal);                // Leer el valor de calibración guardado en EEPROM para Out_Curr_Calib_Fact
    if (abs(Out_Curr_Calib_Fact - eeprom_read_Cal) > 0.0001) {        // Verifica con tolerancia
      SaveCalibration(ADD_OUT_CURR_FAC_CAL, Out_Curr_Calib_Fact);
      Serial.println(F("Out_Curr_Calib_Fact saved"));
      cng_flag = true;                                                // Flag de que se grabo un valor
    }
  
    // Comprobar y guardar Sns_Volt_Calib_Offs
    EEPROM.get(ADD_SNS_VOLT_OFF_CAL, eeprom_read_Cal);                // Leer el valor de calibración guardado en EEPROM para Sns_Volt_Calib_Offs
    if (abs(Sns_Volt_Calib_Offs - eeprom_read_Cal) > 0.0001) {        // Verifica con tolerancia
      SaveCalibration(ADD_SNS_VOLT_OFF_CAL, Sns_Volt_Calib_Offs);
      Serial.println(F("Sns_Volt_Calib_Offs saved"));
      cng_flag = true;                                                // Flag de que se grabo un valor
    }
  
    // Comprobar y guardar Sns_Curr_Calib_Offs
    EEPROM.get(ADD_SNS_CURR_OFF_CAL, eeprom_read_Cal);                // Leer el valor de calibración guardado en EEPROM para Sns_Curr_Calib_Offs
    if (abs(Sns_Curr_Calib_Offs - eeprom_read_Cal) > 0.0001) {        // Verifica con tolerancia
      SaveCalibration(ADD_SNS_CURR_OFF_CAL, Sns_Curr_Calib_Offs);
      Serial.println("Sns_Curr_Calib_Offs saved");
      cng_flag = true;                                                // Flag de que se grabo un valor
    }
  
    // Comprobar y guardar Out_Volt_Calib_Offs
    EEPROM.get(ADD_OUT_VOLT_OFF_CAL, eeprom_read_Cal);                // Leer el valor de calibración guardado en EEPROM para Sns_Volt_Calib_Offs
    if (abs(Out_Volt_Calib_Offs - eeprom_read_Cal) > 0.0001) {        // Verifica con tolerancia
      SaveCalibration(ADD_OUT_VOLT_OFF_CAL, Out_Volt_Calib_Offs);
      Serial.println("Out_Volt_Calib_Offs saved");
      cng_flag = true;                                                // Flag de que se grabo un valor
    }
  
    // Comprobar y guardar Out_Curr_Calib_Offs
    EEPROM.get(ADD_OUT_CURR_OFF_CAL, eeprom_read_Cal);                // Leer el valor de calibración guardado en EEPROM para Sns_Curr_Calib_Offs
    if (abs(Out_Curr_Calib_Offs - eeprom_read_Cal) > 0.0001) {        // Verifica con tolerancia
      SaveCalibration(ADD_OUT_CURR_OFF_CAL, Out_Curr_Calib_Offs);
      Serial.println("Out_Curr_Calib_Offs saved");
      cng_flag = true;                                                // Flag de que se grabo un valor
    }
  
    if (!cng_flag) {Mnsg = "Cal is Ok ";} else {Mnsg = "Cal saved!";} // Aviso por Display si se grabó alguna Calibración.
  }
#include "Variables.h"
#include "Funciones.h"
#include <EEPROM.h>
#include <math.h>
#include <string.h>

namespace {

uint32_t g_displayHash[4] = {0};

struct CalibrationData {
  int address;
  float *variable;
  bool isOffset;
};

struct CalibrationPoint {
  float external;
  float sensed;
  float command;
};

void CopyUiText(char *dest, size_t destLen, const char *src) {
  if (src == nullptr) src = "";
  strncpy(dest, src, destLen - 1);
  dest[destLen - 1] = '\0';
}

void ClearUiText(char *dest, size_t destLen) {
  CopyUiText(dest, destLen, "");
}

void SetModeMessage() {
  Mnsg[0] = Mode;
  Mnsg[1] = ' ';
  Mnsg[2] = 's';
  Mnsg[3] = 'e';
  Mnsg[4] = 't';
  Mnsg[5] = '!';
  Mnsg[6] = '\0';
}

void PrintFixedLine(uint8_t row, const char *text) {
  char line[21];
  byte idx = 0;

  while (idx < 20 && text[idx] != '\0') {
    line[idx] = text[idx];
    idx++;
  }
  while (idx < 20) {
    line[idx++] = ' ';
  }
  line[20] = '\0';

  lcd.setCursor(0, row);
  lcd.print(line);
}

void ClearLineBuffer(char *line) {
  memset(line, ' ', 20);
  line[20] = '\0';
}

byte CopyTextLimited(char *dest, const char *src, byte maxLen) {
  byte idx = 0;
  while (idx < maxLen && src[idx] != '\0') {
    dest[idx] = src[idx];
    idx++;
  }
  return idx;
}

byte BuildTemperatureText(char *dest) {
  char digits[5];
  itoa(temp, digits, 10);
  byte idx = CopyTextLimited(dest, digits, 4);
  dest[idx++] = char(0xDF);
  dest[idx++] = 'C';
  dest[idx] = '\0';
  return idx;
}

void BuildMenuLine(char *dest, bool selected, const char *item) {
  memset(dest, ' ', 12);
  dest[0] = selected ? '>' : ' ';
  CopyTextLimited(dest + 1, item, 11);
  dest[12] = '\0';
}

float ParseKeypadValue(const char *text) {
  unsigned long integerPart = 0;
  unsigned long fractionalPart = 0;
  unsigned long fractionalScale = 1;
  bool decimalSeen = false;

  for (byte i = 0; text[i] != '\0'; i++) {
    char ch = text[i];
    if (ch == '.') {
      if (decimalSeen) break;
      decimalSeen = true;
      continue;
    }

    if (ch < '0' || ch > '9') break;

    if (!decimalSeen) {
      integerPart = integerPart * 10UL + (ch - '0');
    } else if (fractionalScale < 1000UL) {
      fractionalPart = fractionalPart * 10UL + (ch - '0');
      fractionalScale *= 10UL;
    }
  }

  return integerPart + (fractionalPart / static_cast<float>(fractionalScale));
}

uint32_t HashDisplayLine(const char *text) {
  uint32_t hash = 2166136261UL;
  for (byte i = 0; i < 20; i++) {
    hash ^= static_cast<uint8_t>(text[i]);
    hash *= 16777619UL;
  }
  return hash;
}

void WriteTextAt(char *line, byte column, const char *text, byte maxLen = 20) {
  byte idx = 0;
  while ((column + idx) < 20 && idx < maxLen && text[idx] != '\0') {
    line[column + idx] = text[idx];
    idx++;
  }
}

void CommitLineIfChanged(uint8_t row, const char *text) {
  uint32_t newHash = HashDisplayLine(text);
  if (g_displayHash[row] != newHash) {
    lcd.setCursor(0, row);
    lcd.print(text);
    g_displayHash[row] = newHash;
  }
}

void ResetDisplayCache() {
  for (byte row = 0; row < 4; row++) {
    g_displayHash[row] = 0;
  }
}

void DrawMenuStatusPanel() {
  char field[6];
  char panel[8];
  float power = max(voltage * current, 0.0f);

  memset(panel, ' ', 7);
  panel[7] = '\0';
  byte fieldLen = BuildTemperatureText(field);
  memcpy(panel + (7 - min((byte)7, fieldLen)), field, min((byte)7, fieldLen));
  lcd.setCursor(13, 0);
  lcd.print(panel);

  memset(panel, ' ', 7);
  panel[7] = '\0';
  dtostrf(max(voltage, 0.0f), 6, 2, field);
  memcpy(panel, field, 6);
  panel[6] = 'v';
  lcd.setCursor(13, 1);
  lcd.print(panel);

  memset(panel, ' ', 7);
  panel[7] = '\0';
  dtostrf(max(current, 0.0f), 6, 3, field);
  memcpy(panel, field, 6);
  panel[6] = 'a';
  lcd.setCursor(13, 2);
  lcd.print(panel);

  memset(panel, ' ', 7);
  panel[7] = '\0';
  dtostrf(power, 6, (power >= 100.0f) ? 1 : 2, field);
  memcpy(panel, field, 6);
  panel[6] = 'w';
  lcd.setCursor(13, 3);
  lcd.print(panel);
}

float ApplyLinearCalibration(float raw, float factorValue, float offsetValue) {
  return raw * factorValue + offsetValue;
}

#ifndef WOKWI_SIMULATION
float InvertLinearCalibration(float desired, float factorValue, float offsetValue) {
  if (fabs(factorValue) < 0.0001f) return desired;
  return (desired - offsetValue) / factorValue;
}

uint16_t BuildDacCode(float desired, float baseFactor, float calibFactor, float calibOffset) {
  float correctedSetpoint = InvertLinearCalibration(desired, calibFactor, calibOffset);
  long ctrl = lroundf(correctedSetpoint * baseFactor);
  return constrain(ctrl, 0L, 4095L);
}
#endif

const CalibrationData *GetCalibrationTable(byte &count) {
  static const CalibrationData data[] = {
      {ADD_SNS_VOLT_FAC_CAL, &Sns_Volt_Calib_Fact, false},
      {ADD_SNS_CURR_FAC_CAL, &Sns_Curr_Calib_Fact, false},
      {ADD_OUT_VOLT_FAC_CAL, &Out_Volt_Calib_Fact, false},
      {ADD_OUT_CURR_FAC_CAL, &Out_Curr_Calib_Fact, false},
      {ADD_SNS_VOLT_OFF_CAL, &Sns_Volt_Calib_Offs, true},
      {ADD_SNS_CURR_OFF_CAL, &Sns_Curr_Calib_Offs, true},
      {ADD_OUT_VOLT_OFF_CAL, &Out_Volt_Calib_Offs, true},
      {ADD_OUT_CURR_OFF_CAL, &Out_Curr_Calib_Offs, true},
  };

  count = sizeof(data) / sizeof(data[0]);
  return data;
}

bool IsOffsetAddress(int address) {
  byte count = 0;
  const CalibrationData *data = GetCalibrationTable(count);
  for (byte i = 0; i < count; i++) {
    if (data[i].address == address) return data[i].isOffset;
  }
  return false;
}

float GetDefaultCalibrationValue(bool isOffset) {
  return isOffset ? 0.0f : 1.0f;
}

bool IsCalibrationValueValid(float value, bool isOffset) {
  return isOffset ? (value >= -0.1f && value <= 0.1f) : (value >= 0.9f && value <= 1.1f);
}

void ResetModeCalibration(char mode) {
  if (mode == 'V') {
    Sns_Volt_Calib_Fact = 1.0f;
    Sns_Volt_Calib_Offs = 0.0f;
    Out_Volt_Calib_Fact = 1.0f;
    Out_Volt_Calib_Offs = 0.0f;
  } else if (mode == 'I') {
    Sns_Curr_Calib_Fact = 1.0f;
    Sns_Curr_Calib_Offs = 0.0f;
    Out_Curr_Calib_Fact = 1.0f;
    Out_Curr_Calib_Offs = 0.0f;
  }
}

void CancelCalibrationSession(const char *message) {
  CopyUiText(Mnsg, MESSAGE_LEN, message);
  ClearUiText(Req_info, REQUEST_LEN);
  cal_st = false;
  mem_st = false;
  Modetocal = 'U';
  Load_Calibration();
}

void ApplyCalibrationSetpoints(float voltageValue, float currentValue) {
  setvoltage = voltageValue;
  setcurrent = currentValue;
  Change_Mode('B');
  Output_Control(true);
}

int RunSimpleMenu(const char *title, const char *const items[], int itemCount) {
  int selected = 0;
  int top = 0;
  bool redraw = true;
  unsigned long lastStatusRefresh = 0;

  while (true) {
    if (redraw) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("            ");
      lcd.setCursor(0, 0);
      lcd.print(title);
      for (int row = 0; row < 3; row++) {
        char line[13];
        int itemIndex = top + row;
        if (itemIndex < itemCount) {
          BuildMenuLine(line, itemIndex == selected, items[itemIndex]);
        } else {
          line[0] = '\0';
        }
        lcd.setCursor(0, row + 1);
        lcd.print("            ");
        lcd.setCursor(0, row + 1);
        lcd.print(line);
      }
      DrawMenuStatusPanel();
      lastStatusRefresh = millis();
      redraw = false;
    }

    if ((millis() - lastStatusRefresh) >= 250) {
      Read_Voltage_Current();
      Temp_check();
      DrawMenuStatusPanel();
      lastStatusRefresh = millis();
    }

    char key = customKeypad.getKey();
    if (key == NO_KEY) continue;

    if (key == 'U' || key == '<') {
      selected = (selected > 0) ? selected - 1 : itemCount - 1;
      if (selected < top) top = selected;
      if (selected >= top + 3) top = selected - 2;
      redraw = true;
      delay(180);
    } else if (key == 'D' || key == '>') {
      selected = (selected < itemCount - 1) ? selected + 1 : 0;
      if (selected < top) top = selected;
      if (selected >= top + 3) top = selected - 2;
      redraw = true;
      delay(180);
    } else if (key == 'C') {
      delay(180);
      lcd.clear();
      return -1;
    } else if (key == 'E') {
      delay(180);
      lcd.clear();
      return selected;
    }
  }
}

}  // namespace

void Read_encoder() {
  static uint8_t old_AB = 3;
  static int8_t encval = 0;
  static const int8_t enc_states[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

  old_AB <<= 2;

  if (digitalRead(ENC_A)) old_AB |= 0x02;
  if (digitalRead(ENC_B)) old_AB |= 0x01;

  encval += enc_states[(old_AB & 0x0f)];

  if (encval > 3) {
    setvalue = setvalue + factor;
    encval = 0;
  } else if (encval < -3) {
    setvalue = setvalue - factor;
    encval = 0;
  }

  setvalue = min(ENCODER_MAX, max(0, setvalue));
  r = (Mode == 'V') ? 1 : 2;
  y = CP;
}

void Read_keypad(void) {
  char customKey = customKeypad.getKey();
  byte index_max = 4;

  if (customKey == NO_KEY) return;

  Show_VI_Settings();

  if (customKey == 'V') { Change_Mode(); return; }
  if (customKey == 'I') { Change_Mode('I'); return; }

  if (customKey == 'S') {
    mem_st = !mem_st;
    if (cal_st) {
      if (mem_st) {
        CopyUiText(Req_info, REQUEST_LEN, "CLR: Cancel");
      } else {
        ClearUiText(Req_info, REQUEST_LEN);
      }
    } else {
      ClearUiText(Req_info, REQUEST_LEN);
    }
    return;
  }

  if (customKey >= '0' && customKey <= '9') {
    if (mem_st && !cal_st) {
      Mem_selec(customKey);
      return;
    } else if (cal_st) {
      index_max = 5;
    }

    index = min(index_max, max(0, index));
    numbers[index++] = customKey;
    numbers[index] = '\0';
    r = 3;
    y = (cal_st ? 5 : 4) + index;
    return;
  }

  if (customKey == '.') {
    if (decimalPoint != '*') {
      numbers[index++] = '.';
      numbers[index] = '\0';
      r = 3;
      y = (cal_st ? 5 : 4) + index;
      decimalPoint = '*';
    }
    return;
  }

  if (customKey == 'E') {
    x = ParseKeypadValue(numbers);
    Reset_Input_Value();
    if (cal_st) {
      Calibration();
    } else {
      reading = x;
      setvalue = reading * 1000;
      SetModeMessage();
    }
    r = (Mode == 'V') ? 1 : 2;
    y = CP;
    Output_Control(true);
    return;
  }

  if (customKey == 'C' && cal_st && mem_st) {
    CancelCalibrationSession("Cal cancel");
    return;
  }

  if (customKey == 'C' && index > 0) {
    mem_st = false;
    if (cal_st) ClearUiText(Req_info, REQUEST_LEN);
    index--;
    if (numbers[index] == '.') decimalPoint = ' ';
    numbers[index] = '\0';
    lcd.setCursor((cal_st ? 5 : 4) + index, r);
    lcd.print(" ");
    r = 3;
    y = (cal_st ? 5 : 4) + index;
    return;
  }

  if (customKey == 'U') {
    setvalue = min(ENCODER_MAX, max(0, setvalue + factor));
    r = (Mode == 'V') ? 1 : 2;
    y = CP;
    return;
  }

  if (customKey == 'D') {
    setvalue = min(ENCODER_MAX, max(0, setvalue - factor));
    r = (Mode == 'V') ? 1 : 2;
    y = CP;
    return;
  }

  if (customKey == '<') { CP--; return; }
  if (customKey == '>') { CP++; return; }
}

void Reset_Input_Value() {
  index = 0;
  numbers[index] = '\0';
  decimalPoint = ' ';
  ClearUiText(Mnsg, MESSAGE_LEN);
}

void Cursor_position(void) {
  if (digitalRead(ENC_BTN) == LOW) {
    delay(200);
    CP++;
    Show_VI_Settings();
  }

  if (CP != CPprev) {
    if (Mode == 'V') {
      int unitPosition = 14;
      if (CP < unitPosition) CP = unitPosition + 4;
      if (CP > unitPosition + 4) CP = unitPosition;
      if (CP == unitPosition + 2) {
        if (CP > CPprev) CP++;
        else CP--;
      }
      if (CP == unitPosition) factor = 10000;
      if (CP == unitPosition + 1) factor = 1000;
      if (CP == unitPosition + 3) factor = 100;
      if (CP == unitPosition + 4) factor = 10;
    }

    if (Mode == 'I') {
      int unitPosition = 14;
      if (CP < unitPosition) CP = unitPosition + 4;
      if (CP > unitPosition + 4) CP = unitPosition;
      if (CP == unitPosition + 1) {
        if (CP > CPprev) CP++;
        else CP--;
      }
      if (CP == unitPosition) factor = 1000;
      if (CP == unitPosition + 2) factor = 100;
      if (CP == unitPosition + 3) factor = 10;
      if (CP == unitPosition + 4) factor = 1;
    }
    r = (Mode == 'V') ? 1 : 2;
    y = CP;
    CPprev = CP;
  }
}

void Show_VI_Settings(void) {
  dspset = true;
  dspsetchng = true;
  dspset_time = millis();
}

void Manage_Display(void) {
  unsigned long current_time = millis();
  float displayVoltage = (dspset && Mode == 'V') ? setvoltage : voltage;
  float displayCurrent = (dspset && Mode == 'I') ? setcurrent : current;

  if (dspset && dspsetchng) {
    V_I_W_Display(displayVoltage, displayCurrent, Mnsg);
    dspsetchng = false;
    dspset_time = current_time;
    lcd.blink();
  } else if (!dspset) {
    if ((current_time - dsp_time) >= DSP_INFO_RFRSH) {
      V_I_W_Display(displayVoltage, displayCurrent, Mnsg);
      dsp_time = current_time;
    }
  }

  if (dspset && (current_time - dspset_time) >= DSP_SET_DRTN) {
    dspset = false;
    if (!cal_st) Reset_Input_Value();
    lcd.noBlink();
  }
}

void V_I_W_Display(float PrintVoltage, float PrintCurrent, const char *mensaje) {
  PrintVoltage = max(PrintVoltage, 0.0f);
  PrintCurrent = max(PrintCurrent, 0.0f);
  float PrintPower = max(voltage, 0.0f) * max(current, 0.0f);
  char row0[21];
  char row1[21];
  char row2[21];
  char row3[21];
  char valueBuf[8];
  char tempBuf[6];

  ClearLineBuffer(row0);
  ClearLineBuffer(row1);
  ClearLineBuffer(row2);
  ClearLineBuffer(row3);

  WriteTextAt(row0, 0, mensaje, 14);
  BuildTemperatureText(tempBuf);
  WriteTextAt(row0, 15, tempBuf, 5);

  if (mem_st && !cal_st) {
    WriteTextAt(row1, 0, "1..6 Mem", 8);
    WriteTextAt(row2, 0, "0 Config.", 9);
  } else if (mem_st && cal_st) {
    WriteTextAt(row1, 0, "CLR: Cancel", 11);
  } else if (cal_st) {
    WriteTextAt(row1, 0, Req_info, 10);
  }
  row1[13] = (Mode == 'V') ? '>' : ' ';
  dtostrf(PrintVoltage, 5, 2, valueBuf);
  WriteTextAt(row1, 14, valueBuf, 5);
  row1[19] = 'v';

  row2[13] = (Mode == 'I') ? '>' : ' ';
  dtostrf(PrintCurrent, 5, 3, valueBuf);
  WriteTextAt(row2, 14, valueBuf, 5);
  row2[19] = 'a';

  if (cal_st) {
    WriteTextAt(row3, 0, "Real:", 5);
  } else {
    WriteTextAt(row3, 0, "Set:", 4);
  }
  WriteTextAt(row3, cal_st ? 5 : 4, numbers, 8);
  dtostrf(PrintPower, 6, (PrintPower >= 100.0f) ? 1 : 2, valueBuf);
  WriteTextAt(row3, 13, valueBuf, 6);
  row3[19] = 'w';

  CommitLineIfChanged(0, row0);
  CommitLineIfChanged(1, row1);
  CommitLineIfChanged(2, row2);
  CommitLineIfChanged(3, row3);

  lcd.setCursor(y, r);
}

void Temp_check(void) {
  static unsigned long Last_tmpchk = 0;
  static unsigned long fan_on_time = 0;
  static bool fans_on = false;
  unsigned long hldtmp_time = millis();

  if ((hldtmp_time - Last_tmpchk) >= TMP_CHK_TIME) {
    int act_temp = analogRead(TEMP_SNSR);
#ifndef WOKWI_SIMULATION
    act_temp = act_temp * 0.48828125;
#else
    act_temp = act_temp * 0.09765625;
#endif

    if (temp != act_temp) {
      temp = act_temp;
      new_temp = true;
    } else {
      new_temp = false;
    }

    Last_tmpchk = hldtmp_time;

    if (temp >= 40) {
      digitalWrite(FAN_CTRL, HIGH);
      fans_on = true;
      fan_on_time = hldtmp_time;
    } else if (fans_on && (hldtmp_time - fan_on_time) >= FAN_ON_DRTN) {
      digitalWrite(FAN_CTRL, LOW);
      fans_on = false;
    }
  }
}

void Limits_check(void) {
  float actpwrdis;
  static float maxpwrdis;

  if (temp >= 99) {
    Output_Control(false);
    Reset_Input_Value();
    CopyUiText(Mnsg, MESSAGE_LEN, "OFF Max T");
  }

  if (new_temp) {
    maxpwrdis = max(0.0f, min(90.0f, 108.0f - 0.72f * temp));
    new_temp = false;
  }

  actpwrdis = max(0.0f, (38.0f - voltage) * current / 2.0f);

  if (actpwrdis >= maxpwrdis) {
    Output_Control(false);
    Reset_Input_Value();
    CopyUiText(Mnsg, MESSAGE_LEN, "OFF Max WD");
  }

  if (Mode == 'V' && reading > VOLTS_CUTOFF) {
    reading = VOLTS_CUTOFF;
    setvalue = VOLTS_CUTOFF * 1000;
    Reset_Input_Value();
    CopyUiText(Mnsg, MESSAGE_LEN, "Max V");
  }

  if (Mode == 'I' && reading > CURRENT_CUTOFF) {
    reading = CURRENT_CUTOFF;
    setvalue = CURRENT_CUTOFF * 1000;
    Reset_Input_Value();
    CopyUiText(Mnsg, MESSAGE_LEN, "Max I");
  }
}

void Read_Voltage_Current() {
#ifndef WOKWI_SIMULATION
  float raw_voltage = 0;
  float raw_current = 0;

  ads.setGain(GAIN_ONE);
  adcv = ads.readADC_SingleEnded(VLTG_SNSR);
  raw_voltage = ads.computeVolts(adcv) * Sns_Volt_Fact;
  voltage = ApplyLinearCalibration(raw_voltage, Sns_Volt_Calib_Fact, Sns_Volt_Calib_Offs);

  adci = ads.readADC_SingleEnded(CRR_SNSR);
  raw_current = ads.computeVolts(adci) * Sns_Curr_Fact;
  current = ApplyLinearCalibration(raw_current, Sns_Curr_Calib_Fact, Sns_Curr_Calib_Offs);
#else
  int potValue = analogRead(A3);
  static float sim_Resistance = 1000;
  float effectiveVoltage = ApplyLinearCalibration(setvoltage, Out_Volt_Calib_Fact, Out_Volt_Calib_Offs);
  float currentLimit = ApplyLinearCalibration(setcurrent, Out_Curr_Calib_Fact, Out_Curr_Calib_Offs);

  sim_Resistance = map(potValue, 0, 1023, 1000, 10) / 10.0f;
  current = effectiveVoltage / sim_Resistance;

  if (current >= currentLimit) {
    current = currentLimit;
    voltage = sim_Resistance * current;
  } else {
    voltage = effectiveVoltage;
  }
#endif
}

void Set_Voltage_Current(bool setVI) {
#ifndef WOKWI_SIMULATION
  uint16_t Ctrl_Volt = 0;
  uint16_t Ctrl_Curr = 0;

  if (Mode == 'V' && reading != setvoltage && !setVI) {
    setvoltage = reading;
    Ctrl_Volt = BuildDacCode(setvoltage, Out_Volt_Fact, Out_Volt_Calib_Fact, Out_Volt_Calib_Offs);
    dacV.setVoltage(Ctrl_Volt, false);
    Show_VI_Settings();
    return;
  }

  if (Mode == 'I' && reading != setcurrent && !setVI) {
    setcurrent = reading;
    Ctrl_Curr = BuildDacCode(setcurrent, Out_Curr_Fact, Out_Curr_Calib_Fact, Out_Curr_Calib_Offs);
    dacI.setVoltage(Ctrl_Curr, false);
    Show_VI_Settings();
    return;
  }

  if (setVI) {
    Ctrl_Volt = BuildDacCode(setvoltage, Out_Volt_Fact, Out_Volt_Calib_Fact, Out_Volt_Calib_Offs);
    Ctrl_Curr = BuildDacCode(setcurrent, Out_Curr_Fact, Out_Curr_Calib_Fact, Out_Curr_Calib_Offs);
    dacV.setVoltage(Ctrl_Volt, false);
    dacI.setVoltage(Ctrl_Curr, false);
  }
#else
  if (Mode == 'V' && reading != setvoltage && !setVI) {
    setvoltage = setvalue / 1000;
    Show_VI_Settings();
  }
  if (Mode == 'I' && reading != setcurrent && !setVI) {
    setcurrent = setvalue / 1000;
    Show_VI_Settings();
  }
  if (setVI) {
    voltage = ApplyLinearCalibration(setvoltage, Out_Volt_Calib_Fact, Out_Volt_Calib_Offs);
    current = ApplyLinearCalibration(setcurrent, Out_Curr_Calib_Fact, Out_Curr_Calib_Offs);
  }
#endif
}

void Mem_selec(char key) {
  bool setVI = true;

  if (key == '1') {
    setvoltage = 3.3f;
    setcurrent = 1.0f;
  } else if (key == '2') {
    setvoltage = 5.0f;
    setcurrent = 2.0f;
  } else if (key == '3') {
    setvoltage = 9.0f;
    setcurrent = 3.0f;
  } else if (key == '4') {
    setvoltage = 12.0f;
    setcurrent = 3.0f;
  } else if (key == '5') {
    setvoltage = 16.8f;
    setcurrent = 4.0f;
  } else if (key == '6') {
    setvoltage = 24.0f;
    setcurrent = 5.0f;
  } else if (key == '0') {
    Configuration_Menu();
    return;
  } else {
    setVI = false;
    CopyUiText(Mnsg, MESSAGE_LEN, "Empty");
  }

  if (setVI) {
    Change_Mode('B');
  }
  mem_st = false;
}

void Save_EEPROM(int address, float value) {
  EEPROM.put(address, value);
}

void Load_EEPROM(int address, float &variable) {
  bool isOffset = IsOffsetAddress(address);
  EEPROM.get(address, variable);

  if (isnan(variable)) {
    variable = GetDefaultCalibrationValue(isOffset);
    EEPROM.put(address, variable);
  }
}

void Save_Calibration() {
  bool cng_flag = false;
  float eeprom_read_Cal = 0.0f;
  byte count = 0;
  const CalibrationData *data = GetCalibrationTable(count);

  for (byte i = 0; i < count; i++) {
    Load_EEPROM(data[i].address, eeprom_read_Cal);
    if (fabs(*data[i].variable - eeprom_read_Cal) > 0.0001f) {
      Save_EEPROM(data[i].address, *data[i].variable);
      cng_flag = true;
    }
  }

  CopyUiText(Mnsg, MESSAGE_LEN, cng_flag ? "Cal saved!" : "Cal is Ok");
}

void Load_Calibration() {
  bool corrected = false;
  byte count = 0;
  const CalibrationData *data = GetCalibrationTable(count);

  for (byte i = 0; i < count; i++) {
    Load_EEPROM(data[i].address, *data[i].variable);

    if (!IsCalibrationValueValid(*data[i].variable, data[i].isOffset)) {
      float defaultValue = GetDefaultCalibrationValue(data[i].isOffset);
      *data[i].variable = defaultValue;
      Save_EEPROM(data[i].address, *data[i].variable);
      corrected = true;
    }
  }

  if (corrected) CopyUiText(Mnsg, MESSAGE_LEN, "Cal defaulted");
}

void Change_Mode(char Modetoset) {
  char savedMessage[MESSAGE_LEN];
  char savedRequest[REQUEST_LEN];

  if (cal_st) {
    CopyUiText(savedMessage, sizeof(savedMessage), Mnsg);
    CopyUiText(savedRequest, sizeof(savedRequest), Req_info);
  }

  if (Modetoset == 'I') {
    Mode = 'I';
    r = 2;
    y = 14;
    CP = 14;
    CPprev = 14;
    factor = 1000;
    reading = setcurrent;
    setvalue = setcurrent * 1000;
  } else if (Modetoset == 'B') {
    Set_Voltage_Current(true);
    if (Mode == 'V') {
      reading = setvoltage;
      setvalue = setvoltage * 1000;
    } else {
      reading = setcurrent;
      setvalue = setcurrent * 1000;
    }
  } else {
    Mode = 'V';
    r = 1;
    y = 15;
    CP = 15;
    CPprev = 15;
    factor = 1000;
    reading = setvoltage;
    setvalue = setvoltage * 1000;
  }

  if (cal_st) {
    index = 0;
    numbers[index] = '\0';
    decimalPoint = ' ';
    CopyUiText(Mnsg, MESSAGE_LEN, savedMessage);
    CopyUiText(Req_info, REQUEST_LEN, savedRequest);
  } else {
    Reset_Input_Value();
  }
}

void Configuration_Menu(void) {
  static const char *const menuItems[] = {"Limits", "Calibration", "Exit"};

  Output_Control(false);
  Reset_Input_Value();
  mem_st = false;
  lcd.noBlink();

  while (true) {
    int selection = RunSimpleMenu("Config.", menuItems, sizeof(menuItems) / sizeof(menuItems[0]));

    if (selection == 0) {
      Limits_Menu();
    } else if (selection == 1) {
      Calibration_Menu();
      if (cal_st) {
        return;
      }
    } else {
      break;
    }
  }

  lcd.clear();
  ResetDisplayCache();
  dspsetchng = true;
  dspset = false;
  V_I_W_Display(voltage, current, Mnsg);
}

void Limits_Menu(void) {
  static const char *const menuItems[] = {"Back"};
  Output_Control(false);
  lcd.noBlink();
  RunSimpleMenu("Limits", menuItems, sizeof(menuItems) / sizeof(menuItems[0]));
  lcd.clear();
  ResetDisplayCache();
}

void Calibration_Menu(void) {
  static const char *const menuItems[] = {"Cal V", "Cal I", "Load Cal", "Save Cal", "Back"};
  int selection;

  Output_Control(false);
  Reset_Input_Value();
  mem_st = false;
  lcd.noBlink();

  selection = RunSimpleMenu("Calibration", menuItems, sizeof(menuItems) / sizeof(menuItems[0]));

  switch (selection) {
    case 0:
      Start_Calibration('V');
      return;
    case 1:
      Start_Calibration('I');
      return;
    case 2:
      Load_Calibration();
      lcd.clear();
      PrintFixedLine(0, "Calibration loaded");
      delay(1000);
      break;
    case 3:
      Save_Calibration();
      lcd.clear();
      PrintFixedLine(0, "Calibration saved");
      delay(1000);
      break;
    default:
      break;
  }

  lcd.clear();
  ResetDisplayCache();
  dspsetchng = true;
  dspset = false;
  V_I_W_Display(voltage, current, Mnsg);
}

void Start_Calibration(char mode) {
  cal_st = true;
  mem_st = false;
  Mode = mode;
  Modetocal = mode;

  ResetModeCalibration(mode);
  ApplyCalibrationSetpoints(2.0f, 0.1f);

  CopyUiText(Mnsg, MESSAGE_LEN, (Modetocal == 'V') ? "Enter V1" : "Enter I1");
  ClearUiText(Req_info, REQUEST_LEN);
}

void Calibration(void) {
  static bool firstPoint = true;
  static CalibrationPoint point1 = {0, 0, 0};
  static CalibrationPoint point2 = {0, 0, 0};
  CalibrationPoint *currentPoint = firstPoint ? &point1 : &point2;
  bool sns_ok = false;
  bool out_ok = false;

  if (x <= 0) {
    CancelCalibrationSession("Cal cancel");
    firstPoint = true;
    return;
  }

  currentPoint->external = x;
  currentPoint->sensed = (Modetocal == 'V') ? voltage : current;
  currentPoint->command = (Modetocal == 'V') ? setvoltage : setcurrent;

  if (firstPoint) {
    firstPoint = false;
    if (Modetocal == 'V') {
      ApplyCalibrationSetpoints(28.0f, 0.1f);
      CopyUiText(Mnsg, MESSAGE_LEN, "Enter V2");
    } else {
      ApplyCalibrationSetpoints(20.0f, 4.0f);
      CopyUiText(Mnsg, MESSAGE_LEN, "Enter I2");
    }
    ClearUiText(Req_info, REQUEST_LEN);
    return;
  }

  if (Modetocal == 'V') {
    sns_ok = Calc_Calib_Fact(point1.external, point1.sensed, point2.external, point2.sensed, Sns_Volt_Calib_Fact, Sns_Volt_Calib_Offs);
    out_ok = Calc_Calib_Fact(point1.external, point1.command, point2.external, point2.command, Out_Volt_Calib_Fact, Out_Volt_Calib_Offs);
  } else {
    sns_ok = Calc_Calib_Fact(point1.external, point1.sensed, point2.external, point2.sensed, Sns_Curr_Calib_Fact, Sns_Curr_Calib_Offs);
    out_ok = Calc_Calib_Fact(point1.external, point1.command, point2.external, point2.command, Out_Curr_Calib_Fact, Out_Curr_Calib_Offs);
  }

  if (!sns_ok || !out_ok) {
    CopyUiText(Mnsg, MESSAGE_LEN, (Modetocal == 'V') ? "V Cal Fail" : "I Cal Fail");
  } else {
    CopyUiText(Mnsg, MESSAGE_LEN, (Modetocal == 'V') ? "V Cal Done" : "I Cal Done");
  }

  ClearUiText(Req_info, REQUEST_LEN);
  cal_st = false;
  mem_st = false;
  firstPoint = true;
  Modetocal = 'U';
}

bool Calc_Calib_Fact(float x1, float y1, float x2, float y2, float &factorValue, float &offsetValue) {
  float deltaX = x2 - x1;
  float deltaY = y2 - y1;

  if (fabs(deltaX) < 0.001f || fabs(deltaY) < 0.001f) {
    factorValue = 1.0f;
    offsetValue = 0.0f;
    return false;
  }

  float rawFactor = deltaX / deltaY;
  float rawOffset = x1 - rawFactor * y1;
  bool inRange = IsCalibrationValueValid(rawFactor, false) && IsCalibrationValueValid(rawOffset, true);

  factorValue = constrain(rawFactor, 0.9f, 1.1f);
  offsetValue = constrain(rawOffset, -0.1f, 0.1f);
  return inRange;
}

void Output_Control(bool enable) {
  if (enable) digitalWrite(MSFT_CTRL, HIGH);
  else digitalWrite(MSFT_CTRL, LOW);
}

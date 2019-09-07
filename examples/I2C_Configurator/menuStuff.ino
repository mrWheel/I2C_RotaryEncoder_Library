/*
***************************************************************************  
**
**  Program     : menuStuff (part of I2C_Configurator)
**    
**  Copyright (c) 2019 Willem Aandewiel
**
**  TERMS OF USE: MIT License. See bottom of file.                                                            
***************************************************************************      
*/

//===========================================================================================
bool valueInRange(int32_t val, int32_t minVal, int32_t maxVal)
{
  if (val < minVal) return false;
  if (val > maxVal) return false;
  return true;
} // valueInRange()

//===========================================================================================
int32_t readNumber(char *prompt, int32_t minVal, int32_t maxVal)
{
  int32_t inVal, outVal;
  String in;
  do {
    Serial.println();
    Serial.print(prompt);
    Serial.print(F("("));
    Serial.print(minVal);
    Serial.print(F(" t/m "));
    Serial.print(maxVal);
    Serial.print(F(") : "));
    while(Serial.available()) Serial.read();
    Serial.setTimeout(30000); // dertig seconden
    in = Serial.readStringUntil('\n'); 
    inVal = String(in).toInt();
  } while (!valueInRange(inVal, minVal, maxVal));

  Serial.println(inVal);

  return inVal;

} // readNumber()


//===========================================================================================
void animateLeds()
{
  static uint8_t ledMode = 0;
  static uint32_t animateTimer;
  static uint8_t W, R, G, B;

  if (!doAnimate || (millis() - animateTimer) < 10) {
    return;
  }
  animateTimer = millis();

  switch(ledMode) {
  case 0:
    R = Encoder1.getLedRed();
    G = Encoder1.getLedGreen();
    B = Encoder1.getLedBlue();
    Encoder1.setRGBcolor(R, G, B);
    ledMode = 1;
    break;

  case 1:
    if (R < 255) {  // 16711680
      Encoder1.setRGBcolor(++R, G, B);
    } else {
      ledMode++;
    }
    break;

  case 2:
    if (G < 0xFF) {
      if (R >= 4) R-= 4;
      else R = 0;
      Encoder1.setRGBcolor(R, ++G, B);
    } else {
      ledMode++;
    }
    break;

  case 3:
    if (B < 0xFF) {
      if (G >= 4) G -= 4;
      else G = 0;
      Encoder1.setRGBcolor(R, G, ++B);
    } else {
      ledMode++;
    }
    break;

  case 4:
    if (R < 0xFF) {  // 16711680
      if (B >= 4) B -= 4;
      else B = 0;
      Encoder1.setRGBcolor(++R, G, B);
    } else {
      W = 0;
      R = 255;
      ledMode++;
    }
    break;

  case 5:
    if (W < 200) {
      if (R >= 4) R -= 4;
      else R = 0;
      Encoder1.setRGBcolor(R, G, B);
      W++;
    } else {
      ledMode = 0;
    }
  } // switch()

} // animateLeds()


//===========================================================================================
void handleKeyInput()
{
  char    inChar;
  int32_t val;
  uint8_t command = 0;
  uint8_t modeReg;

  while (Serial.available() > 0) {
    delay(1);
    inChar = (char)Serial.read();

    switch(inChar) {
    case 'A':
      I2C_newAddress = readNumber("Change I2C address (dec)", 1, 127);
      if (!Encoder1.setNewAddress(I2C_newAddress & 0x7F)) {
        Serial.println(F("Error setting new I2C address .."));
      } else {
        Serial.print(F("New I2C address set to [0x"));
        Serial.print(I2C_newAddress, HEX);
        Serial.println(F("] -> needs reBoot!!"));
      }
      break;
    case 'b':
    case 'B':
      val = readNumber("Blue Led PWM values", 0, 255);
      if (Encoder1.setLedBlue(val)) {
        Serial.print(F("Blue set to ["));
        Serial.print(Encoder1.getLedBlue());
        Serial.println(F("]"));
      } else {
        Serial.println(F("Error setting Blue PWM .."));
      }
      break;
    case 'c':
    case 'C':
      val = readNumber("RGB color", 0, 0xFFFFFF);
      if (!Encoder1.setRGBcolor(val)) {
        Serial.println(F("Error setting Color value .."));
      }
      Serial.print(F("Color is dec["));
      Serial.print(val);
      Serial.print(F("]/[0x"));
      Serial.print(val, HEX);
      Serial.print(F("] -> "));
      printRegister(sizeof(val), &val);
      Serial.println();
      break;
    case '-':
      doAnimate = false;
      break;

    case '+':
      doAnimate = true;
      break;
    case 'd':
    case 'D':
      val = readNumber("Rotary value", 0, 255);
      if (!Encoder1.setRotVal(val)) {
        Serial.println(F("Error setting Rotary value .."));
      }
      break;
    case 'e':
    case 'E':
      val = readNumber("Min. Rotary value", -1024, 1024);
      if (!Encoder1.setRotMin((int16_t)val)) {
        Serial.println(F("Error setting Min. Rotary value.."));
      }
      if (rotVal < val) {
        Encoder1.setRotVal(val);
        rotVal = val;
      }
      break;
    case 'f':
    case 'F':
      val = readNumber("Max. Rotary value", 0, 1024);
      if (!Encoder1.setRotMax(val)) {
        Serial.println(F("Error setting Max. Rotary value.."));
      }
      if (rotVal > val) {
        Encoder1.setRotVal(val);
        rotVal = val;
      }
      break;
    case 'g':
    case 'G':
      val = readNumber("Green Led PWM value", 0, 255);
      if (Encoder1.setLedGreen(val)) {
        Serial.print(F("Green set to ["));
        Serial.print(Encoder1.getLedGreen());
        Serial.println(F("]"));
      } else {
        Serial.println(F("Error setting Green PWM .."));
      }
      break;
    case 'h':
    case 'H':
      val = readNumber("Rotary Spin Time (millisec.)", 0, 250);
      if (!Encoder1.setRotSpinTime(val)) {
        Serial.println(F("Error setting Rotary Spin Time value.."));
      }
      break;
    case 'i':
    case 'I':
      val = readNumber("Steps per click", -100, 100);
      if (!Encoder1.setRotStep(val)) {
        Serial.println(F("Error setting Steps per click .."));
      }
      break;
    case 'j':
    case 'J':
      val = readNumber("Debounce time in micro seconds", 0, 255);
      if (!Encoder1.setDebounceTime(val & 0xFF)) {
        Serial.println(F("Error setting DebounceTime.."));
      }
      break;
    case 'k':
    case 'K':
      val = readNumber("Mid Press Time (milli sec.)", 0, 1000);
      if (!Encoder1.setMidPressTime(val)) {
        Serial.println(F("Error setting Mid Press Time .."));
      }
      break;
    case 'l':
    case 'L':
      val = readNumber("Long Press Time (milli sec.)", 0, 5000);
      if (!Encoder1.setLongPressTime(val)) {
        Serial.println(F("Error setting Long Press Time .."));
      }
      break;
    case 'r':
    case 'R':
      val = readNumber("Red PWM value", 0, 255);
      if (Encoder1.setLedRed(val)) {
        Serial.print(F("Red set to ["));
        Serial.print(Encoder1.getLedRed());
        Serial.println(F("]"));
      } else {
        Serial.println(F("Error setting Red PWM .."));
      }
      break;
    case '1':
      val = readNumber("setMode HW Rot Dir (0=CC/1=C)", 0, 1);
      modeReg = Encoder1.getModeSettings();
      Serial.print(F("modeSettings "));
      printRegister(sizeof(modeReg), &modeReg);
      Serial.println();
      if (val == 1) {
        if (!Encoder1.setModeSetBit(STNG_HWROTDIR)) {
          Serial.println(F("Error setting HW Rot.Dir. to 1"));
        }
      } else {
        if (!Encoder1.setModeClearBit(STNG_HWROTDIR)) {
          Serial.println(F("Error setting HW Rot.Dir. to 0"));
        }
      }

      modeReg = Encoder1.getModeSettings();
      Serial.print(F("modeSettings "));
      printRegister(sizeof(modeReg), &modeReg);
      Serial.println();
      break;
    case '2':
      val = readNumber("setMode flipMode", 0, 1);
      modeReg = Encoder1.getModeSettings();
      Serial.print(F("modeSettings "));
      printRegister(sizeof(modeReg), &modeReg);
      Serial.println();
      if (val == 1) {
        if (!Encoder1.setModeSetBit(STNG_FLIPMODE)) {
          Serial.println(F("Error setting flipMode to 1"));
        }
        Encoder1.setModeClearBit(STNG_TURNMODE);
      } else {
        if (!Encoder1.setModeClearBit(STNG_FLIPMODE)) {
          Serial.println(F("Error setting flipMode to 0"));
        }
      }
      modeReg = Encoder1.getModeSettings();
      Serial.print(F("modeSettings "));
      printRegister(sizeof(modeReg), &modeReg);
      Serial.println();
      break;

    case '3':
      val = readNumber("setMode turnMode", 0, 1);
      modeReg = Encoder1.getModeSettings();
      Serial.print(F("modeSettings "));
      printRegister(sizeof(modeReg), &modeReg);
      Serial.println();
      if (val == 1) {
        if (!Encoder1.setModeSetBit(STNG_TURNMODE)) {
          Serial.println(F("Error setting tunMode to 1"));
        }
        Encoder1.setModeClearBit(STNG_FLIPMODE);
      } else {
        if (!Encoder1.setModeClearBit(STNG_TURNMODE)) {
          Serial.println(F("Error setting turnMode to 0"));
        }
      }
      modeReg = Encoder1.getModeSettings();
      Serial.print(F("modeSettings "));
      printRegister(sizeof(modeReg), &modeReg);
      Serial.println();
      break;

    case 's':
    case 'S':
      if (Encoder1.writeCommand(_BV(CMD_WRITECONF))) {
        Serial.println(F("Registers saved to EEPROM"));
      } else {
        Serial.println(F("Error saving to EEPROM"));
      }
      delay(250); // <-- give Slve some time to save to EEPROM
      if (Encoder1.writeCommand(_BV(CMD_REBOOT))) {
        Serial.println(F("Slave is reBooting .."));
        inConfigureMode = false;
      } else {
        Serial.println(F("Slave is not reBooting ??"));
      }
      break;
    case '0':
      if (Encoder1.writeCommand(_BV(CMD_READCONF))) {
        Serial.println(F("registers read from EEPROM .."));
      } else {
        Serial.println(F("Error reading from EEPROM"));
      }
      break;
    case 'X':
      if (Encoder1.writeCommand(_BV(CMD_REBOOT))) {
        Serial.println(F("Slave is reBooting .."));
        inConfigureMode = false;
      } else {
        Serial.println(F("Slave is not reBooting ??"));
      }
      break;
    case 'z':
    case 'Z':
      Serial.println(F("Z (exit config mode)"));
      inConfigureMode = false;
      break;

    default:
      Serial.print(F("\n==== Firmware version I2C_RotaryEncoder========[v"));
      Serial.print(majorRelease);
      Serial.print(F("."));
      Serial.print(minorRelease);
      Serial.println(F("]============================"));
      Serial.print(F(" A.  Change I2C address .............. (is now [0x"));
      whoAmI = Encoder1.getWhoAmI();
      Serial.print(whoAmI, HEX);
      Serial.print(F(", dec"));
      Serial.print(whoAmI);
      Serial.print(F("])"));
      if (whoAmI == 0x00) {
        inConfigureMode = false;
        return;
      }
      if (I2C_newAddress != Encoder1.getWhoAmI()) {
        Serial.print(F(" *! ["));
        Serial.print(I2C_newAddress, HEX);
        Serial.println(F("]"));
      } else  Serial.println();

      Serial.print(F(" R.  Set Red led PWM value ........... (is now ["));
      Serial.print(Encoder1.getLedRed());
      Serial.println(F("])"));
      Serial.print(F(" G.  Set Green led PWM value ......... (is now ["));
      Serial.print(Encoder1.getLedGreen());
      Serial.println(F("])"));
      Serial.print(F(" B.  Set Blue led PWM value .......... (is now ["));
      Serial.print(Encoder1.getLedBlue());
      Serial.println(F("])"));
      Serial.println(F(" C.  Set RGB color"));
      Serial.println(F(" +   Animate RGB (switch off with '-')"));

      Serial.print(F(" D.  Set Rotary value ................ (is now ["));
      Serial.print(Encoder1.getRotVal());
      Serial.println(F("])"));
      Serial.print(F(" E.  Set Minimum Rotary value ........ (is now ["));
      Serial.print(Encoder1.getRotMin());
      Serial.println(F("])"));
      Serial.print(F(" F.  Set Maximum Rotary value ........ (is now ["));
      Serial.print(Encoder1.getRotMax());
      Serial.println(F("])"));
      Serial.print(F(" H.  Set Rotary Spin Time in milliSec. (is now ["));
      Serial.print(Encoder1.getRotSpinTime());
      Serial.println(F("])"));
      Serial.print(F(" I.  Set Rotary Step per click ....... (is now ["));
      Serial.print(Encoder1.getRotStep());
      Serial.println(F("])"));
      Serial.print(F(" J.  Set debounceTime in microSeconds  (is now ["));
      Serial.print(Encoder1.getDebounceTime());
      Serial.println(F("])"));
      Serial.print(F(" K.  Set Mid Press Time in milliSec.   (is now ["));
      Serial.print(Encoder1.getMidPressTime());
      Serial.println(F("])"));
      Serial.print(F(" L.  Set Long Press Time in milliSec.  (is now ["));
      Serial.print(Encoder1.getLongPressTime());
      Serial.println(F("])"));

      Serial.println(F(" "));
      Serial.print(F(" 1.  setMode HW Rot. Dir. (Settings is "));
      modeReg = Encoder1.getModeSettings();
      printRegister(sizeof(modeReg), &modeReg);
      if (modeReg & (1<<STNG_HWROTDIR)) {
        Serial.println(F(")/ C --> +1"));
      } else {
        Serial.println(F(")/ CC -> -1"));
      }
      Serial.print(F(" 2.  setMode flipMode ... (Settings is "));
      modeReg = Encoder1.getModeSettings();
      printRegister(sizeof(modeReg), &modeReg);
      if (modeReg & (1<<STNG_FLIPMODE))   Serial.println(F(")/ On"));
      else                                Serial.println(F(")/ Off"));
      Serial.print(F(" 3.  setMode turnMode ... (Settings is "));
      modeReg = Encoder1.getModeSettings();
      printRegister(sizeof(modeReg), &modeReg);
      if (modeReg & (1<<STNG_TURNMODE))   Serial.println(F(")/ On"));
      else                                Serial.println(F(")/ Off"));
      Serial.println(F(" "));
      Serial.println(F(" S.  Save registers to EEPROM"));
      Serial.println(F(" 0.  Read registers from EEPROM"));
      Serial.println(F(" "));
      Serial.println(F("*X.  Reboot Slave"));
      Serial.println(F(" Z.  Exit"));
      Serial.println(F(" "));
    } // switch()
    while (Serial.available() > 0) {
      delay(0);
      (char)Serial.read();
    }
  }

}  // handleKeyInput()


/***************************************************************************
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the
* "Software"), to deal in the Software without restriction, including
* without limitation the rights to use, copy, modify, merge, publish,
* distribute, sublicense, and/or sell copies of the Software, and to permit
* persons to whom the Software is furnished to do so, subject to the
* following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
* OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR
* THE USE OR OTHER DEALINGS IN THE SOFTWARE.
* 
***************************************************************************/

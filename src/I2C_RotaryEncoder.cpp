/*
***************************************************************************  
**
**  File    : I2C_RotaryEncoder.cpp
**  Version : v0.1
**
**  Copyright (c) 2019 Willem Aandewiel
**
**  TERMS OF USE: MIT License. See bottom of file.                                                            
***************************************************************************      
*/

#include "./I2C_RotaryEncoder.h"
#include "Arduino.h"

// Constructor
I2CRE::I2CRE() { }

// Initializes the I2C_RotaryEncoder
// Returns false if I2C_RotaryEncoder is not detected
//-------------------------------------------------------------------------------------
bool I2CRE::begin(TwoWire &wireBus, uint8_t deviceAddress)
{
  _I2Cbus = &wireBus;
  _I2Cbus->begin(); 
  _I2Cbus->setClock(100000);

  _I2Caddress = deviceAddress;

  if (isConnected() == false)
    return (false); // Check for I2C_RotaryEncoder presence
  
  return (true); // Everything is OK!

} // begin()

// Change the I2C address of this I2C Slave address to newAddress
//-------------------------------------------------------------------------------------
bool I2CRE::setI2Caddress(uint8_t newAddress)
{
  if (writeReg1Byte(I2CRE_ADDRESS, newAddress)) {
    // Once the address is changed, we need to change it in the library
    _I2Caddress = newAddress;
    // -->> writeReg1Byte(I2CRE_COMMAND, I2CRE_ADDRESS);
    return true;
  }
  return false;

} // newAddress()

//-------------------------------------------------------------------------------------
//-------------------------- SETTERS --------------------------------------------------
//-------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------
bool I2CRE::setRotVal(int16_t val)
{
  return (writeReg2Byte(I2CRE_ROTVAL, val));
}
//-------------------------------------------------------------------------------------
bool I2CRE::setRotMin(int16_t val)
{
  return (writeReg2Byte(I2CRE_ROTMIN, val));
}
//-------------------------------------------------------------------------------------
bool I2CRE::setRotMax(int16_t val)
{
  return (writeReg2Byte(I2CRE_ROTMAX, val));
}
//-------------------------------------------------------------------------------------
bool I2CRE::setRotStep(int16_t val)
{
  return (writeReg2Byte(I2CRE_ROTSTEP, val));
}
//-------------------------------------------------------------------------------------
bool I2CRE::setRotSpinTime(uint8_t millisecs)
{
  return (writeReg1Byte(I2CRE_ROTSPINTIME, millisecs));
}
// Sets the color of the encoder LEDs
//-------------------------------------------------------------------------------------
bool I2CRE::setRGBcolor(uint8_t red, uint8_t green, uint8_t blue)
{
  return (writeReg4Byte( I2CRE_RED, (uint32_t)blue << 16
                         | (uint32_t)green << 8
                         | red)      );
} // setColor()
//-------------------------------------------------------------------------------------
bool I2CRE::setRGBcolor(uint32_t RGB)
{
  return (writeRegRGB( I2CRE_RED, RGB ));
}
// Sets the color of a specific Led
//-------------------------------------------------------------------------------------
bool I2CRE::setLedRed(uint8_t red)
{
//-------------------------------------------------------------------------------------
  return (writeReg1Byte(I2CRE_RED, red));
}
//-------------------------------------------------------------------------------------
bool I2CRE::setLedGreen(uint8_t green)
{
//-------------------------------------------------------------------------------------
  return (writeReg1Byte(I2CRE_GREEN, green));
}
//-------------------------------------------------------------------------------------
bool I2CRE::setLedBlue(uint8_t blue)
{
  return (writeReg1Byte(I2CRE_BLUE, blue));
}
//-------------------------------------------------------------------------------------
bool I2CRE::setDebounceTime(uint8_t microsecs)
{
  return (writeReg1Byte(I2CRE_DEBOUNCETIME, microsecs));
}
//-------------------------------------------------------------------------------------
bool I2CRE::setMidPressTime(uint16_t millisecs)
{
  return (writeReg2Byte(I2CRE_MIDPRESSTIME, millisecs));
}
//-------------------------------------------------------------------------------------
bool I2CRE::setLongPressTime(uint16_t millisecs)
{
  return (writeReg2Byte(I2CRE_LONGPRESSTIME, millisecs));
}
//-------------------------------------------------------------------------------------
bool I2CRE::setModeSetBit(uint8_t bit)
{
  byte modeReg = getModeSettings();
  modeReg |= (1 << bit);
  return (writeReg1Byte(I2CRE_MODESETTINGS, modeReg));
}
//-------------------------------------------------------------------------------------
bool I2CRE::setModeClearBit(uint8_t bit)
{
  byte modeReg = getModeSettings();
  modeReg &= ~(1 << bit);
  return (writeReg1Byte(I2CRE_MODESETTINGS, modeReg));
}
//-------------------------------------------------------------------------------------
bool I2CRE::writeCommand(byte command)
{
  return (writeReg1Byte(I2CRE_COMMAND, command));
}

//-------------------------------------------------------------------------------------
//-------------------------- GETTERS --------------------------------------------------
//-------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------
uint8_t I2CRE::getStatus()
{
  while ((millis() - _statusTimer) < _READDELAY) {
    delay(1);
  }
  _statusTimer = millis();
  uint8_t tmpStatus = (uint8_t)readReg1Byte(I2CRE_STATUS);
  _status |= (uint8_t)tmpStatus;
  _status &= ~(1 << INTERRUPT_BIT);
  return (tmpStatus);
}

//-------------------------------------------------------------------------------------
int8_t I2CRE::getWhoAmI()
{
  return (readReg1Byte(I2CRE_ADDRESS));
}

//-------------------------------------------------------------------------------------
int16_t I2CRE::getRotVal()
{
  return (readReg2Byte(I2CRE_ROTVAL));
}

//-------------------------------------------------------------------------------------
int16_t I2CRE::getRotMin()
{
  return ((int16_t)readReg2Byte(I2CRE_ROTMIN));
}

//-------------------------------------------------------------------------------------
int16_t I2CRE::getRotMax()
{
  return (readReg2Byte(I2CRE_ROTMAX));
}

//-------------------------------------------------------------------------------------
int16_t I2CRE::getRotStep()
{
  return (readReg2Byte(I2CRE_ROTSTEP));
}

//-------------------------------------------------------------------------------------
uint8_t I2CRE::getRotSpinTime()
{
  return (readReg1Byte(I2CRE_ROTSPINTIME));
}

//-------------------------------------------------------------------------------------
uint8_t I2CRE::getLedRed()
{
  return ((uint8_t)readReg1Byte(I2CRE_RED));
}
//-------------------------------------------------------------------------------------
uint8_t I2CRE::getLedGreen()
{
  return ((uint8_t)readReg1Byte(I2CRE_GREEN));
}
//-------------------------------------------------------------------------------------
uint8_t I2CRE::getLedBlue()
{
  return ((uint8_t)readReg1Byte(I2CRE_BLUE));
}

//-------------------------------------------------------------------------------------
uint8_t I2CRE::getDebounceTime()
{
  return (readReg1Byte(I2CRE_DEBOUNCETIME));
}
//-------------------------------------------------------------------------------------
uint16_t I2CRE::getMidPressTime()
{
  return (readReg2Byte(I2CRE_MIDPRESSTIME));
}
//-------------------------------------------------------------------------------------
uint16_t I2CRE::getLongPressTime()
{
  return (readReg2Byte(I2CRE_LONGPRESSTIME));
}
//-------------------------------------------------------------------------------------
uint8_t I2CRE::getModeSettings()
{
  return (readReg1Byte(I2CRE_MODESETTINGS));
}

//-------------------------------------------------------------------------------------
uint8_t I2CRE::getMajorRelease()
{
  return (readReg1Byte(I2CRE_MAJORRELEASE));
}
//-------------------------------------------------------------------------------------
uint8_t I2CRE::getMinorRelease()
{
  return (readReg1Byte(I2CRE_MINORRELEASE));
}


//-------------------------------------------------------------------------------------
//-------------------------- READ FROM REGISTERS --------------------------------------
//-------------------------------------------------------------------------------------

// Reads a uint8_t from a register @addr
//-------------------------------------------------------------------------------------
uint8_t I2CRE::readReg1Byte(uint8_t addr)
{  
  while ((millis() - _statusTimer) < _READDELAY) {
    delay(1);
  }
  _statusTimer = millis();

  _I2Cbus->beginTransmission((uint8_t)_I2Caddress);
  _I2Cbus->write(addr);
  if (_I2Cbus->endTransmission() != 0) {
    return (0); // Slave did not ack
  }

  _I2Cbus->requestFrom((uint8_t)_I2Caddress, (uint8_t) 1);
  if (_I2Cbus->available()) {
    return (_I2Cbus->read());
  }

  return (0); // Slave did not respond
}

// Reads an int16_t from a register @addr
//-------------------------------------------------------------------------------------
int16_t I2CRE::readReg2Byte(uint8_t addr)
{
  while ((millis() - _statusTimer) < _READDELAY) {
    delay(1);
  }
  _statusTimer = millis();

  _I2Cbus->beginTransmission((uint8_t)_I2Caddress);
  _I2Cbus->write(addr);
  if (_I2Cbus->endTransmission() != 0) {
    return (0); // Slave did not ack
  }

  _I2Cbus->requestFrom((uint8_t)_I2Caddress, (uint8_t) 2);
  if (_I2Cbus->available()) {
    uint8_t LSB = _I2Cbus->read();
    uint8_t MSB = _I2Cbus->read();
    return ((int16_t)MSB << 8 | LSB);
  }

  return (0); // Slave did not respond
}

// Reads an int32_t from a register @addr
//-------------------------------------------------------------------------------------
int32_t I2CRE::readReg4Byte(uint8_t addr)
{
  while ((millis() - _statusTimer) < _READDELAY) {
    delay(1);
  }
  _statusTimer = millis();

  _I2Cbus->beginTransmission((uint8_t)_I2Caddress);
  _I2Cbus->write(addr);
  if (_I2Cbus->endTransmission() != 0) {
    return (0); // Slave did not ack
  }

  _I2Cbus->requestFrom((uint8_t)_I2Caddress, (uint8_t) 4);
  delay(10);
  if (_I2Cbus->available()) {
    uint8_t LSB   = _I2Cbus->read();
    uint8_t mLSB  = _I2Cbus->read();
    uint8_t mMSB  = _I2Cbus->read();
    uint8_t MSB   = _I2Cbus->read();
    uint32_t comb = MSB << 24 | mMSB << 16 | mLSB << 8 | LSB;
    return ((int32_t)MSB << 24 | mMSB << 16 | mLSB << 8 | LSB);
  }

  return (0); // Slave did not respond
}

//-------------------------------------------------------------------------------------
//-------------------------- WRITE TO REGISTERS ---------------------------------------
//-------------------------------------------------------------------------------------

// Write a 1 byte value to a register
//-------------------------------------------------------------------------------------
bool I2CRE::writeReg1Byte(uint8_t addr, uint8_t val)
{
  while ((millis() - _statusTimer) < _WRITEDELAY) {
    delay(1);
  }
  _statusTimer = millis();

  _I2Cbus->beginTransmission((uint8_t)_I2Caddress);
  _I2Cbus->write(addr);
  _I2Cbus->write(val);
  if (_I2Cbus->endTransmission() != 0) {
    return (false); // Slave did not ack
  }

  return (true);
}

// Write a 2 byte value to a register
//-------------------------------------------------------------------------------------
bool I2CRE::writeReg2Byte(uint8_t addr, int16_t val)
{
  while ((millis() - _statusTimer) < _WRITEDELAY) {
    delay(1);
  }
  _statusTimer = millis();

  _I2Cbus->beginTransmission((uint8_t)_I2Caddress);
  _I2Cbus->write(addr);
  _I2Cbus->write(val & 0xFF); // LSB
  _I2Cbus->write(val >> 8);   // MSB
  if (_I2Cbus->endTransmission() != 0) {
    return (false); // Slave did not ack
  }

  return (true);
}

// Write a 3 byte value to a register
//-------------------------------------------------------------------------------------
bool I2CRE::writeRegRGB(uint8_t addr, uint32_t RGB)
{
  while ((millis() - _statusTimer) < _WRITEDELAY) {
    delay(1);
  }
  _statusTimer = millis();

  //Serial.print("\nwriteReg3Bytes: ");
  //showRegister(sizeof(val), &val);
  _I2Cbus->beginTransmission((uint8_t)_I2Caddress);
  _I2Cbus->write(addr);
  // val is [-------- rrrrrrrr gggggggg bbbbbbbb]
  _I2Cbus->write(RGB >> 16);     // Red
  _I2Cbus->write(RGB >> 8);       // Green
  _I2Cbus->write(RGB &0xFF);      // Blue
  if (_I2Cbus->endTransmission() != 0) {
    return (false); // Slave did not ack
  }

  return (true);
}

// Write a 3 byte value to a register
//-------------------------------------------------------------------------------------
bool I2CRE::writeReg3Byte(uint8_t addr, int32_t val)
{
  while ((millis() - _statusTimer) < _WRITEDELAY) {
    delay(1);
  }
  _statusTimer = millis();

  _I2Cbus->beginTransmission((uint8_t)_I2Caddress);
  _I2Cbus->write(addr);
  _I2Cbus->write(val &0xFF);     // LSB
  _I2Cbus->write(val >> 8);       // mLSB
  _I2Cbus->write(val >> 16);      // mMSB
  //_I2Cbus->write(val >> 24);    // MSB
  if (_I2Cbus->endTransmission() != 0) {
    return (false); // Slave did not ack
  }

  return (true);
}

// Write a 4 byte value to a register
//-------------------------------------------------------------------------------------
bool I2CRE::writeReg4Byte(uint8_t addr, int32_t val)
{
  while ((millis() - _statusTimer) < _WRITEDELAY) {
    delay(1);
  }
  _statusTimer = millis();

  _I2Cbus->beginTransmission((uint8_t)_I2Caddress);
  _I2Cbus->write(addr);
  _I2Cbus->write(val & 0xFF); // LSB
  _I2Cbus->write(val >> 8);   // mLSB
  _I2Cbus->write(val >> 16);  // mMSB
  _I2Cbus->write(val >> 24);  // MSB
  if (_I2Cbus->endTransmission() != 0) {
    return (false); // Slave did not ack
  }

  return (true);
}

//-------------------------------------------------------------------------------------
//-------------------------- HELPERS --------------------------------------------------
//-------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------
bool I2CRE::isConnected()
{
  _I2Cbus->beginTransmission((uint8_t)_I2Caddress);
  if (_I2Cbus->endTransmission() != 0)
    return (false); // I2C Slave did not ACK
  return (true);
} // isConnected()

//-------------------------------------------------------------------------------------
bool I2CRE::isRotValChanged()
{
  if (_status & (1 << COUNTUP_BIT) || _status & (1 << COUNTDOWN_BIT)) {
    _status &= ~(1 << COUNTUP_BIT);
    _status &= ~(1 << COUNTDOWN_BIT);
    return true;
  } 
  return false;
}
//-------------------------------------------------------------------------------------
bool I2CRE::isRotValChangedUp()
{
  if (_status & (1 << COUNTUP_BIT)) {
    _status &= ~(1 << COUNTUP_BIT);
    return true;
  } 
  return false;
}
//-------------------------------------------------------------------------------------
bool I2CRE::isRotValChangedDown()
{
  if (_status & (1 << COUNTDOWN_BIT)) {
    _status &= ~(1 << COUNTDOWN_BIT);
    return true;
  } 
  return false;
}

//-------------------------------------------------------------------------------------
bool I2CRE::isButtonPressed() 
{
  if (_status & (1 << PRESSED_BIT)) {
    _status &= ~(1 << PRESSED_BIT);
    return true;
  }
  return false;
}

//-------------------------------------------------------------------------------------
bool I2CRE::isButtonQuickReleased()
{
  if (_status & (1 << QUICKRELEASE_BIT)) {
    _status &= ~(1 << QUICKRELEASE_BIT);
    return true;
  }
  return false;
}

//-------------------------------------------------------------------------------------
bool I2CRE::isButtonMidReleased()
{
  if (_status & (1 << MIDRELEASE_BIT)) {
    _status &= ~(1 << MIDRELEASE_BIT);
    return true;
  }
  return false;
}

//-------------------------------------------------------------------------------------
bool I2CRE::isButtonLongReleased()
{
  if (_status & (1 << LONGRELEASE_BIT)) {
    _status &= ~(1 << LONGRELEASE_BIT);
    return true;
  }
  return false;
}

//===========================================================================================
//assumes little endian
void I2CRE::showRegister(size_t const size, void const * const ptr)
{
  unsigned char *b = (unsigned char*) ptr;
  unsigned char byte;
  int i, j;
  Serial.print("[");
  for (i=size-1; i>=0; i--) {
    if (i != (size-1)) Serial.print(" ");
    for (j=7; j>=0; j--) {
      byte = (b[i] >> j) & 1;
      Serial.print(byte);
    }
  }
  Serial.print("] ");
} // showRegister()


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

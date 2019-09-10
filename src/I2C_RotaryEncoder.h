/*
***************************************************************************  
**
**  File    : I2C_RotaryEncoder.h
**  Version : v1.2
**
**  Copyright (c) 2019 Willem Aandewiel
**
**  TERMS OF USE: MIT License. See bottom of file.                                                            
***************************************************************************      
*/


#ifndef _I2C_ROTARYENCODER_H
#define _I2C_ROTARYENCODER_H
#include "Arduino.h"
#include "Wire.h"

#define I2C_SLAVE_ADDRESS 0x28

// status bits
enum  {  INTERRUPT_BIT, COUNTUP_BIT, COUNTDOWN_BIT, PRESSED_BIT
       , QUICKRELEASE_BIT, MIDRELEASE_BIT, LONGRELEASE_BIT };
enum  {  CMD_READCONF, CMD_WRITECONF, CMD_DUM2, CMD_DUM3, CMD_DUM4
       , CMD_DUM5, CMD_DUM6,  CMD_REBOOT };
enum  {  STNG_HWROTDIR, STNG_FLIPMODE, STNG_TURNMODE, STNG_SPARE3
       , STNG_SPARE4, STNG_SPARE5, STNG_SPARE_6, STNG_SPARE7 };


// Map to the various registers on the I2C Rotary Encoder
enum encoderRegisters {
  I2CRE_STATUS          = 0x00,
  I2CRE_ADDRESS         = 0x01,
  I2CRE_MAJORRELEASE    = 0x02,
  I2CRE_MINORRELEASE    = 0x03,
  I2CRE_ROTVAL          = 0x04, // 2
  I2CRE_ROTSTEP         = 0x06, // 2
  I2CRE_ROTMIN          = 0x08, // 2
  I2CRE_ROTMAX          = 0x0A, // 2
  I2CRE_ROTSPINTIME     = 0x0C,
  I2CRE_RED             = 0x0D,
  I2CRE_GREEN           = 0x0E,
  I2CRE_BLUE            = 0x0F,
  I2CRE_DEBOUNCETIME    = 0x10,  // microSeconds
  I2CRE_MIDPRESSTIME    = 0x11,  // 2
  I2CRE_LONGPRESSTIME   = 0x13,  // 2
  I2CRE_MODESETTINGS    = 0x15,
  //----
  I2CRE_COMMAND         = 0xF0  // -> this is NOT a "real" register!!
};

#define _WRITEDELAY 10
#define _READDELAY  5

class I2CRE
{

public:
  I2CRE();

  bool       begin(TwoWire &wireBus = Wire, uint8_t deviceAddress = I2C_SLAVE_ADDRESS);
  bool       isConnected();

//-------------------------------------------------------------------------------------
//-------------------------- GETTERS --------------------------------------------------
//-------------------------------------------------------------------------------------
  uint8_t   getStatus();                  // reads the status byte
  int16_t   getRotVal();                  // read the value of the rotary (-5000 .. +5000)
  int16_t   getRotStep();                 // read the rotary Step (1 .. 50)
  int16_t   getRotMin();                  // read the minimum rotary value (-5000 .. +5000)
  int16_t   getRotMax();                  // read the maximum rotary value (-5000 .. +5000)
  uint8_t   getRotSpinTime();             // read the rotary spin time (2 .. 100 milli seconds)
  int8_t    getWhoAmI();                  // read the Address Register
  uint8_t   getLedRed();                  // read the current Red led PWM value (0 .. 255)
  uint8_t   getLedGreen();                // read the current Green led PWM value (0 .. 255)
  uint8_t   getLedBlue();                 // read the current Blue led PWM value (0 .. 255)
  uint8_t   getDebounceTime();            // read the Debounce Time of the switch (5 .. 250 micro seconds)
  uint16_t  getMidPressTime();            // read the Mid Press Time of the switch (100 .. 5000 milli seconds)
  uint16_t  getLongPressTime();           // read the Long Press Time of the switch (300 .. 10000 milli seconds)
  uint8_t   getMajorRelease();            // read the Major Firmware Release byte (0 .. 255)
  uint8_t   getMinorRelease();            // read the Minor Firmware Release byte (0 .. 255)
  uint8_t   getModeSettings();            // read the Mode register byte (0 .. 255)
  bool      getModeSettings(uint8_t);     // read a Mode register Bit (true if set/1 otherwise false/0)

//-------------------------------------------------------------------------------------
//-------------------------- SETTERS --------------------------------------------------
//-------------------------------------------------------------------------------------
  bool      setRotVal(int16_t val);       // set the value of the Rotary Encoder (-5000 .. + 5000)
  bool      setRotStep(int16_t val);      // set the rotary Step (1 .. 50)
  bool      setRotMin(int16_t val);       // set the Minimum rotary value (-5000 .. +5000)
  bool      setRotMax(int16_t val);       // set the Maximum rotary value (-5000 .. +5000)
  bool      setRotSpinTime(uint8_t val);  // set the Rotary Spin thime value (2 .. 100 milli seconds)
                                          // set the color of all 3 leds  Red, Green, Blue
  bool      setRGBcolor(uint8_t red, uint8_t green, uint8_t blue);  // (0 .. 255, 0 .. 255, 0 .. 255)
  bool      setRGBcolor(uint32_t RGB);    // set the RGB color of all 3 leds (0x000000 .. 0xFFFFFF)
  bool      setLedRed(uint8_t);           // set the PWM value of the Red led (0 .. 255)
  bool      setLedGreen(uint8_t);         // set the PWM value of the Green led (0 .. 255)
  bool      setLedBlue(uint8_t);          // set the PWM value of the Blue led (0 .. 255)
  bool      setDebounceTime(uint8_t);     // set the Debounce Time of the switch (5 .. 250 micro seconds)
  bool      setMidPressTime(uint16_t);    // set the Mid Press Time of the switch (100 .. 5000 milli seconds)
  bool      setLongPressTime(uint16_t);   // set the Long Press Time of the switch (300 .. 10000 milli seconds)
  bool      setModeSetBit(uint8_t);       // set the Mode Bit (STNG_HWROTDIR | STNG_FLIPMODE | STNG_TURNMODE)
  bool      setModeClearBit(uint8_t);     // clears the Mode Bit (STNG_HWROTDIR | STNG_FLIPMODE | STNG_TURNMODE)
  bool      writeCommand(uint8_t);        // write a command to the Slave (CMD_READCONF | CMD_WRITECONF | CMD_REBOOT)

  bool      setI2Caddress(uint8_t newAddress);   // set a new I2C address for this Slave (1 .. 127)        
  
//-------------------------------------------------------------------------------------
//-------------------------- HELPERS --------------------------------------------------
//-------------------------------------------------------------------------------------
  bool      isRotValChanged();            // true if the Rotary Value has changed
  bool      isRotValChangedUp();          // true if the Rotary Value > previous value
  bool      isRotValChangedDown();        // true if the Rotary Value < previous value
  bool      isButtonPressed();            // true if the Button is pressed
  bool      isButtonQuickReleased();      // true if the Button is released before midPressTime
  bool      isButtonMidReleased();        // true if the Button is released between midPressTime and longPressTime
  bool      isButtonLongReleased();        // true if the Button is released after longPressTime

private:
  TwoWire           *_I2Cbus;
  uint8_t           _I2Caddress;
  volatile uint8_t  _status;
  uint32_t          _statusTimer;

  uint8_t   readReg1Byte(uint8_t reg);
  int16_t   readReg2Byte(uint8_t reg);
  int32_t   readReg4Byte(uint8_t reg);

  bool      writeReg1Byte(uint8_t reg, uint8_t val);
  bool      writeReg2Byte(uint8_t reg, int16_t val);
  bool      writeRegRGB(uint8_t reg, uint32_t RGB);
  bool      writeReg3Byte(uint8_t reg, int32_t val);
  bool      writeReg4Byte(uint8_t reg, int32_t val);

//static void onReceiveCallback(int numbytes);

  void showRegister(size_t const size, void const * const ptr);
};

#endif

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

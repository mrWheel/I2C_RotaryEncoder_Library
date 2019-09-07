/*
***************************************************************************  
**
**  Program     : I2C_Configurator
*/
#define _FW_VERSION  "v0.2 (07-09-2019)"
/*
**  Description : With this program you can configure 
**                I2C_RotaryEncoder unit's
**    
**  Copyright (c) 2019 Willem Aandewiel
**
**  TERMS OF USE: MIT License. See bottom of file.                                                            
***************************************************************************      
*/


#define  I2C_DEFAULT_ADDRESS  0x28    // the 7-bit address 
//#define _SDA                  4
//#define _SCL                  5

#define LED_ON                LOW
#define LED_OFF               HIGH
#ifdef ARDUINO_ARCH_ESP8266
  #define ISR_PREFIX        ICACHE_RAM_ATTR
  #define _INTERRUPTPIN     12
#elif ARDUINO_ARCH_AVR
  #define ISR_PREFIX
  #define BUILTIN_LED       13
  #define _INTERRUPTPIN     3
#elif ARDUINO_ARCH_SAM
  #define ISR_PREFIX
  #define BUILTIN_LED       13
  #define _INTERRUPTPIN     3
#else
  #error "Don't know what type of board this is ..."
#endif

#include <I2C_RotaryEncoder.h>

I2CRE Encoder1; //Create instance of this object

static byte   I2C_Address, I2C_newAddress;

byte          whoAmI;
int16_t       rotVal;
int16_t       rotStep;
int16_t       rotMin;
int16_t       rotMax;
byte          majorRelease, minorRelease;
uint32_t      builtinLedTimer;

volatile bool interruptPending  = false;
bool          inConfigureMode   = false;
bool          doAnimate         = false;

//===========================================================================================
ISR_PREFIX void handleInterrupt()
{
  interruptPending = true;
}


//===========================================================================================
void requestEvent()
{
  // nothing .. yet
}

//===========================================================================================
//assumes little endian
void printRegister(size_t const size, void const * const ptr)
{
  unsigned char *b = (unsigned char*) ptr;
  unsigned char byte;
  int i, j;
  Serial.print(F("["));
  for (i=size-1; i>=0; i--) {
    for (j=7; j>=0; j--) {
      byte = (b[i] >> j) & 1;
      Serial.print(byte);
    }
  }
  Serial.print(F("] "));
} // printRegister()


//===========================================================================================
byte findSlaveAddress(byte startAddress)
{
  byte  error;
  bool  slaveFound = false;

  if (startAddress == 0xFF) startAddress = 1;
  for (byte address = startAddress; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error) {
      //Serial.print(F("-> Error["));
      //Serial.print(error);
      //Serial.println(F("]"));
    } else {
      slaveFound = true;
      Serial.print(F("\nFound one!\nDo you want to configure I2C RotaryEncoder at @[0x"));
      Serial.print(address , HEX);
      Serial.print(F("] (y/N) : "));
      String answerS = "";
      char answerC = '-';
      bool stayInWhile = true;
      while ((answerC != 'y' && answerC != 'Y') && stayInWhile) {
        Serial.setTimeout(10000);
        answerS = Serial.readStringUntil('\n');
        answerS.trim();
        answerC = answerS[answerS.length()-1];
        if (answerC == 'y' || answerC == 'Y') {
          Serial.println(F("\nNow in configuration mode .."));
          inConfigureMode = true;
          return address;
        }
        Serial.println(F("trying next address .."));
        inConfigureMode = false;
        stayInWhile     = false;
      }
    }
  }
  if (slaveFound) {
    Serial.println(F("\nstart reScan ..\n"));
  } else {
    Serial.println(F("\n\nNo Rotary Encoder found!! -> try again in 5 seconds .."));
    delay(5000);
  }
  return 0xFF;

} // findSlaveAddress()


//===========================================================================================
void handleRotaryEncoder()
{
  byte statusReg = Encoder1.getStatus();
  if (statusReg == 0) return;

  digitalWrite(BUILTIN_LED, LED_ON);
  builtinLedTimer = millis();

  Serial.print(F("StatusReg ["));
  printRegister(sizeof(statusReg), &statusReg);
  Serial.println(F("]"));
  if (Encoder1.isRotValChangedUp() ) {
    Serial.print(F("Request Value of [C] ROTARY .......... "));
    rotVal = Encoder1.getRotVal();
    Serial.print(F("["));
    Serial.print(rotVal);
    Serial.println(F("]"));
  } // ^^ rotary bit C set ^^
  if (Encoder1.isRotValChangedDown() ) {
    Serial.print(F("Request Value of [CC] ROTARY ......... "));
    rotVal = Encoder1.getRotVal();
    Serial.print(F("["));
    Serial.print(rotVal);
    Serial.println(F("]"));
  } // ^^ rotary bit CC set ^^

  if (Encoder1.isButtonPressed() ) {
    Serial.println(F("-------> Button Pressed"));
  }
  if (Encoder1.isButtonQuickReleased() ) {
    Serial.println(F("-------> Quick Release"));
  }
  if (Encoder1.isButtonMidReleased() ) {
    Serial.println(F("-------> Mid Release"));
  }
  if (Encoder1.isButtonLongReleased() ) {
    Serial.println(F("-------> Long Release"));
  }

} // handleRotaryEncoder();


//===========================================================================================
void setup()
{
  Serial.begin(115200);
  Serial.println(F("\r\nStart I2C-Rotary-Encoder Configurator ....\r\n"));
  Serial.print(F("Setup Wire .."));
//Wire.begin(_SDA, _SCL); // join i2c bus (address optional for master)
  Wire.begin();
//Wire.setClock(400000L);
  Serial.println(F(".. done"));

  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW);

  Serial.print(F("Attach Interrupt on pin ["));
  Serial.print(_INTERRUPTPIN);
  Serial.print(F("]/["));
  Serial.print(digitalPinToInterrupt(_INTERRUPTPIN));
  Serial.print(F("] .."));
  Serial.flush();
  pinMode(_INTERRUPTPIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(_INTERRUPTPIN), handleInterrupt, FALLING);
  Serial.println(F(".. done"));

  interruptPending  = false;
  inConfigureMode   = false;

  Serial.println(F("setup() done .. \n"));

} // setup()


//===========================================================================================
void loop()
{

  if (!inConfigureMode) {
    I2C_Address = findSlaveAddress(I2C_Address);
    if (I2C_Address != 0xFF) {
      Serial.print(F("\nConnecting to  I2C-Rotary-Encoder .."));
      if (Encoder1.begin(Wire, I2C_Address)) {
        majorRelease = Encoder1.getMajorRelease();
        minorRelease = Encoder1.getMinorRelease();
        Serial.print(F(". connected with slave @[0x"));
        Serial.print(I2C_Address, HEX);
        Serial.print(F("] Release[v"));
        Serial.print(majorRelease);
        Serial.print(F("."));
        Serial.print(minorRelease);
        Serial.println(F("]"));
        I2C_newAddress = Encoder1.getWhoAmI();
      } else {
        Serial.println(F(".. Error connecting to I2C slave .."));
      }
    } else {
      return;
    }
  }

  if (inConfigureMode) handleKeyInput();

//  if (interruptPending) {
  interruptPending = false;
  handleRotaryEncoder();

//  } // handle interrupt ..

  animateLeds();

  if ((millis() - builtinLedTimer) > 100) {
    digitalWrite(BUILTIN_LED, LED_OFF);
  }

} // loop()

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

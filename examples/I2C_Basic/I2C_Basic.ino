/*
***************************************************************************
**
**  Program     : I2C_Basic
*/
#define _FW_VERSION  "v0.2 (08-09-2019)"
/*
**  Description : Demo "howto" Read Rotary Value and Button States
**
**  Copyright (c) 2019 Willem Aandewiel
**
**  TERMS OF USE: MIT License. See bottom of file.
***************************************************************************
*/


#define I2C_DEFAULT_ADDRESS  0x28    // the 7-bit address 
//#define _SDA                  4
//#define _SCL                  5
#define LED_ON                  0
#define LED_OFF               255

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

I2CRE Encoder; //Create instance of the I2CRE object

byte          whoAmI;
int16_t       rotVal;
int16_t       rotStep;
int16_t       rotMin;
int16_t       rotMax;
byte          majorRelease, minorRelease;
uint32_t      builtinLedTimer;
int8_t        inMode = 0;

volatile bool interruptPending  = false;

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
void handleRotaryEncoder()
{
  byte statusReg = Encoder.getStatus();
  if (statusReg == 0) return;

  digitalWrite(BUILTIN_LED, LED_ON);
  builtinLedTimer = millis();

  if (Encoder.isRotValChanged() ) {
    Serial.print(F("Request Value of ROTARY .......... "));
    rotVal = Encoder.getRotVal();
    Serial.print(F("["));
    Serial.print(rotVal);
    Serial.println(F("]"));
    switch (inMode) {
    case 1:
      Encoder.setLedRed(rotVal);
      break;
    case 2:
      Encoder.setLedBlue(rotVal);
      break;
    case 3:
      Encoder.setLedGreen(rotVal);
      break;
    }
  }

  if (Encoder.isButtonPressed() ) {
    Serial.println(F("-------> Button Pressed"));
    inMode = 0;
    Encoder.setRotVal(0);
  }
  if (Encoder.isButtonQuickReleased() ) {
    Serial.println(F("-------> Quick Release (change RED led)"));
    inMode = 1;
    Encoder.setRotVal(0);
  }
  if (Encoder.isButtonMidReleased() ) {
    Serial.println(F("-------> Mid Release   (Change BLUE led)"));
    inMode = 2;
    Encoder.setRotVal(0);
  }
  if (Encoder.isButtonLongReleased() ) {
    Serial.println(F("-------> Long Release  (Change GREEN led)"));
    inMode = 3;
    Encoder.setRotVal(0);
  }

} // handleRotaryEncoder();


//===========================================================================================
void setup()
{
  Serial.begin(115200);
  Serial.println(F("\r\nStart I2C-Rotary-Encoder Basic ....\r\n"));
  Serial.print(F("Setup Wire .."));
//Wire.begin(_SDA, _SCL); // join i2c bus (address optional for master)
  Wire.begin();
//Wire.setClock(400000L);
  Serial.println(F(".. done"));

  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LED_ON);

  Serial.print(F("Attach Interrupt on pin ["));
  Serial.print(_INTERRUPTPIN);
  Serial.print(F("]/["));
  Serial.print(digitalPinToInterrupt(_INTERRUPTPIN));
  Serial.print(F("] .."));
  Serial.flush();
  pinMode(_INTERRUPTPIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(_INTERRUPTPIN), handleInterrupt, FALLING);
  Serial.println(F(".. done"));

  if (Encoder.begin(Wire, I2C_DEFAULT_ADDRESS)) {
    majorRelease = Encoder.getMajorRelease();
    minorRelease = Encoder.getMinorRelease();
    Serial.print(F(". connected with slave @[0x"));
    Serial.print(I2C_DEFAULT_ADDRESS, HEX);
    Serial.print(F("] Release[v"));
    Serial.print(majorRelease);
    Serial.print(F("."));
    Serial.print(minorRelease);
    Serial.println(F("]"));
  } else {
    Serial.println(F(".. Error connecting to I2C slave .."));
    delay(1000);
    return;
  }

  Encoder.setRotMin(0);                 // set Minimal value of the RotEncoder
  Encoder.setRotMax(255);               // set Maximal value of the RotEncoder
  Encoder.setRotStep(5);                // change RotVal 5 positions per click
  Encoder.setRotSpinTime(20);           // milli Seconds
  Encoder.setDebounceTime(50);          // in micro Seconds
  Encoder.setModeSetBit(STNG_FLIPMODE); // Flip from Max to Min and from Min to Max
  Encoder.setLedGreen(128);             // turn green led on half intensity

  interruptPending  = false;

  Serial.println(F("setup() done .. \n"));

} // setup()


//===========================================================================================
void loop()
{
  if (interruptPending) {
    interruptPending = false;
    handleRotaryEncoder();
  } // handle interrupt ..

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

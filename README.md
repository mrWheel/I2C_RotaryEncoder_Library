# Arduino library for the I2C_RotaryEncoder

<p>Find the project description at <a href="https://willem.aandewiel.nl/">www.aandewiel.nl</a> (not yet but soon).</p>
<br>

This library gives an interface between your own program and the I2C_RotaryEncoder.

<center><img src="images/I2C_RotaryEncoder_v22-3D.png"></center>

To use it you have to include this library in your sketch

```
#include <I2C_RotaryEncoder.h>
```

Declare an Encoder object (declare one for every I2C_RotaryEncoder):

```
I2CRE Encoder1; // Create instance of the I2CRE object
```

Create a interrupt service routine:

```
#ifdef ARDUINO_ARCH_ESP8266
  #define ISR_PREFIX        ICACHE_RAM_ATTR
//#define BUILTIN_LED       2
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

volatile bool interruptPending  = false;

ISR_PREFIX void handleInterrupt()
{
  interruptPending = true;
}
```

And in your <code>setup()</code> function attach the <code>_INTERRUPT_PIN</code> to this ISRoutine:

```
  pinMode(_INTERRUPTPIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(_INTERRUPTPIN), handleInterrupt, FALLING);
```

In the main <code>loop()</code> function handle <code>interruptPending</code>.

```
void loop() 
{
  if (interruptPending) {
    interruptPending = false;
    if (Encoder1.isRotValChanged() ) {
    	Serial.print("Request Value of ROTARY [");
    	rotVal = Encoder1.getRotVal();
    	Serial.print(rotVal);
    	Serial.println("]");
    }

    if (Encoder1.isButtonPressed() ) {
    	Serial.println("-------> Button Pressed ");
    }
    if (Encoder1.isButtonQuickReleased() ) {
    	Serial.println("-------> Quick Release");
    }
    if (Encoder1.isButtonMidReleased() ) {
    	Serial.println("-------> Mid Release);
    }
    if (Encoder1.isButtonLongReleased() ) {
    	Serial.println("-------> Long Release);
    }
  }
}	// loop()
```

<center><img src="images/I2C_RotaryEncoder_v22-PCB_top.png"></center>

The library gives you the following setters:

| Setter             | Returns | Parms    | Description             |
|:-------------------|:-------:|:---------|:------------------------|
| setRotVal()        | bool    | int16_t  | set the value of the Rotary Encoder (-5000 .. + 5000)|
| setRotStep()       | bool    | int16_t  | set the rotary Step (1 .. 50) |
| setRotMin()        | bool    | int16_t  | set the Minimum rotary value (-5000 .. +5000)|
| setRotMax()        | bool    | int16_t  | set the Maximum rotary value (-5000 .. +5000)|
| setRotSpinTime()   | bool    | uint8_t  | set the Rotary Spin thime value (2 .. 100 milli seconds)|
| setRGBcolor()      | bool    | uint8_t, uint8_t, uint8_t|set the color of all 3 leds  Red, Green, Blue (0 .. 255, 0 .. 255, 0 .. 255)|
| setRGBcolor()      | bool    | uint32_t | set the RGB color of all 3 leds (0x000000 .. 0xFFFFFF)|
| setLedRed()        | bool    | uint8_t  | set the PWM value of the Red led (0 .. 255)|
| setLedGreen()      | bool    | uint8_t  | set the PWM value of the Green led (0 .. 255)|
| setLedBlue()       | bool    | uint8_t  | set the PWM value of the Blue led (0 .. 255)|
| setDebounceTime()  | bool    | uint8_t  | set the Debounce Time of the switch (5 .. 250 micro seconds)|
| setMidPressTime()  | bool    | uint16_t | set the Mid Press Time of the switch (100 .. 5000 milli seconds)|
| setLongPressTime() | bool    | uint16_t | set the Long Press Time of the switch (300 .. 10000 milli seconds)|
| setModeSetBit()    | bool    | uint8_t  | set the Mode Bit (STNG_HWROTDIR \| STNG_FLIPMODE \| STNG_TURNMODE)|
| setModeClearBit()  | bool    | uint8_t  | clears the Mode Bit (STNG_HWROTDIR \| STNG_FLIPMODE \| STNG_TURNMODE)|
| setI2Caddress()    | bool    | uint8_t  | set a new I2C address for this Slave (1 .. 127)|
| writeCommand()     | bool    | uint8_t  | write a command to the Slave (CMD_READCONF \| CMD_WRITECONF \| CMD_REBOOT)|

The library gives you the following getters:

| Getter             | Returns  | Parms | Description |
|:-------------------|:--------:|:-----:|:------------|
| getStatus()        | uint8_t  | none  | reads the status byte
| getRotVal()        | int16_t  | none  | read the value of the rotary (-5000 .. +5000)
| getRotStep()       | int16_t  | none  | read the rotary Step (1 .. 50)
| getRotMin()        | int16_t  | none  | read the minimum rotary value (-5000 .. +5000)
| getRotMax()        | int16_t  | none  | read the maximum rotary value (-5000 .. +5000)
| getRotSpinTime()   | uint8_t  | none  | read the rotary spin time (2 .. 100 milli seconds)
| getWhoAmI()        | int8_t   | none  | read the Address Register
| getLedRed()        | uint8_t  | none  | read the current Red led PWM value (0 .. 255)
| getLedGreen()      | uint8_t  | none  | read the current Green led PWM value (0 .. 255)
| getLedBlue()       | uint8_t  | none  | read the current Blue led PWM value (0 .. 255)
| getDebounceTime()  | uint8_t  | none  | read the Debounce Time of the switch (5 .. 250 micro seconds)
| getMidPressTime()  | uint16_t | none  | read the Mid Press Time of the switch (100 .. 5000 milli seconds)
| getLongPressTime() | uint16_t | none  | read the Long Press Time of the switch (300 .. 10000 milli seconds)
| getMajorRelease()  | uint8_t  | none  | read the Major Firmware Release byte (0 .. 255)
| getMinorRelease()  | uint8_t  | none  | read the Minor Firmware Release byte (0 .. 255)
| getModeSettings()  | uint8_t  | none  | read the Mode register byte (0 .. 255)
| getModeSettings()  | bool     | uint8_t  | read the Mode register byte and test against (STNG_HWROTDIR \| STNG_FLIPMODE \| STNG_TURNMODE)

And the library gives you the following helpers:

| Helper                 | Returns | Parms | Description |
|:-----------------------|:-------:|:-----:|:------------|
|isRotValChanged()       | bool    | none  | true if the Rotary Value has changed
|isRotValChangedUp()     | bool    | none  | true if the Rotary Value > previous value
|isRotValChangedDown()   | bool    | none  | true if the Rotary Value < previous value
|isButtonPressed()       | bool    | none  | true if the Button is pressed
|isButtonQuickReleased() | bool    | none  | true if the Button is released before midPressTime
|isButtonMidReleased()   | bool    | none  | true if the Button is released between midPressTime and longPressTime
|isButtonLongReleased()  | bool    | none  | true if the Button is released after longPressTime


<center><img src="images/I2CRE_Factory_Parts.png"></center>

# Arduino library for the I2C_RotaryEncoder

<p>Find the project description at <a href="https://willem.aandewiel.nl/">www.aandewiel.nl</a>.</p>
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

<center><img src="images/I2CRE_Factory_Parts.png"></center>

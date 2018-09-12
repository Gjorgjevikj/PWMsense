# PWMsense
Arduino PWM sensing library - determines the frequency and the duty of signals at any pin

Interrupt driven library for measuring parameters (duty cycle & frequency) of a PWM signal on any Arduino pin 
Uses EnableInterrupt library from https://github.com/GreyGnome/EnableInterrupt.

Supports AVR and ESP8266 architectures.

Single header file defining template fully static class and some global functions.

## PWMsense

PWMsense object (templetized by pin) `PWMsense<pin> obj` installs ISR that continually records the signal averaging the calculated duty and frequency (during multiple cycles) of the signal. 
  
  ```C
  obj::begin(); // installs the ISR and starts monitoring the signal
  obj::end(); // uninstalls the ISR (stops monitoring)
  obj::reset(); // resets measures (starts monitoring from scratch)
  long int obj::pulseCount(); // returns the number of times the signal has changed (either raised or fallen) since the begin or reset has been called 
  float obj::duty(); // returns the duty of the signal in % (0-100) - averaged over all cycles since reset
  float obj::frequency(); // returns the frequency of the signal in Hz - averaged over all cycles since reset
  ```
  
  Global functions:

  `FreqEstimate<pin>(timeout, n)` - determine the frequency of the signal on the given pin and returns as soon as n full cycles or at most timeout milliseconds have passed
  
  `DutyEstimate<pin>(timeout, n)` - determine the duty of the signal on the given pin and returns as soon as n full cycles or at most timeout milliseconds have passed
  
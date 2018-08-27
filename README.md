# PWMsense
Arduino PWM sensing library

Simple interrupt driven library for measuring parameters (duty cycle & frequency) of a PWM signal on any Arduino pin 
Uses EnableInterrupt library from https://github.com/GreyGnome/EnableInterrupt.

Supports ESP8266 also.

Includes 2 header files (can be used independently - actually 2 libraries in one) defining template fully static class and some global functions.

## PWMinfo 

PWMinfo object (templetized by pin) `PWMinfo<pin> obj` installs ISR that records the timestamps of the last 2 signal level changes that are used to determine the frequency and the duty cycle of the signal. 
  
  ```C
  obj::begin(); // installs the ISR and starts monitoring the signal
  obj::end(); // uninstalls the ISR (stops monitoring)
  obj::reset(); // resets timestamps (starts monitoring from scratch)
  bool obj::valid(); // returns true if the signal has changed sufficient times in order to determine the frequency and the duty 
  float obj::dutyCycle(); // returns the duty of the signal in % (0-100) - estimate using the timings of the last cycle only
  float obj::prf(); // returns the frequency of the signal in Hz  - estimate using the timings of the last cycle only
  ```
  
  global functions:
  `OnePulseFreqEstimate<pin>(timeout)` and `OnePulseDutyEstimate<pin>(timeout)` - deretmine the frequency / duty of the signal on the given pin and returns as soon as one full cycle or at most timeout milliseconds have passed
  
## PWMsense

PWMsense object (templetized by pin) `PWMsense<pin> obj` installs ISR that continually records the signal averaging the calculated duty and frequency (during multiple cycles) of the signal. 
  
  ```C
  obj::begin(); // installs the ISR and starts monitoring the signal
  obj::end(); // uninstalls the ISR (stops monitoring)
  obj::reset(); // resets measures (starts monitoring from scratch)
  long int obj::pulseCount(); // returns the number of times the signal has changed (either raised or fallen) since the begin or reset has been called 
  float obj::dutyCycle(); // returns the duty of the signal in % (0-100) - averaged over all cycles since reset
  float obj::prf(); // returns the frequency of the signal in Hz - averaged over all cycles since reset
  ```
  
  Global functions:
  `FreqEstimate<pin>(timeout, n)` and `DutyEstimate<pin>(timeout, n)` - determine the frequency / duty of the signal on the given pin and returns as soon as n full cycles or at most timeout milliseconds have passed
  
  
  
  

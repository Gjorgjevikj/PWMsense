/*
Library for simple PWM signal measuring

(c) Dejan Gjorgjevikj, 2017

*/

#ifndef PWMINFO_H
#define PWMINFO_H

#include "Arduino.h"

// To enable pinchange intterupt on any pin on Arduino EnableIntterupt library from GrayGnome is used 
//#define LIBCALL_ENABLEINTERRUPT
#if defined(ARDUINO_ARCH_AVR)
#define EI_ARDUINO_INTERRUPTED_PIN
#include <EnableInterrupt.h>
#define ICACHE_RAM_ATTR
#endif

#define DEBUG_ACCESS
// the value to be returned from the frequency sense functions when the frequency can not be determined
#define UNDETERMINED_FREQ 0.0

#define PWMinf(obj,pin) typedef PWMinfo<pin> obj;

// The data filled in intterupt handler packed in one single structure
struct Cycle
{
  unsigned long fallTime; // time of last pulse fall event
  unsigned long riseTime; // time of last pulse rise event
  unsigned long prevFallTime; // time of previous pulse fall event
  unsigned long prevRiseTime; // time of previous pulse rise event
};

// The interrupt handler functions per pin and the class handling the sensing (reding the recorded counters by the interrupt handlers and calculating the duty/frequency) are templates 
template <uint8_t PIN> 
class PWMinfo;

//#pragma GCC push_options
//#pragma GCC optimize ("-O4")
template <uint8_t PIN>
void __attribute__ ((optimize("-O4"))) ICACHE_RAM_ATTR ISR_Change(void) 
//void ICACHE_RAM_ATTR ISR_Change(void) 
{
#if defined(ESP8266)      
  bool PinState = digitalRead(PIN);
#else
  bool PinState = !!arduinoPinState;
#endif
  unsigned long t = micros();

  if (PinState) // raise
  {
	  PWMinfo<PIN>::pulses.prevRiseTime = PWMinfo<PIN>::pulses.riseTime;
	  PWMinfo<PIN>::pulses.riseTime = t;
  }
  else //fall
  {
	  PWMinfo<PIN>::pulses.prevFallTime = PWMinfo<PIN>::pulses.fallTime;
	  PWMinfo<PIN>::pulses.fallTime = t;
  }
}
//#pragma GCC pop_options

template <uint8_t PIN> // templetized by pin, so there will be a separate class / object / ISR for every monitored pin
class PWMinfo
{
  private:
    PWMinfo() {} // no need for constructor - everything is static, since every class will have a single instance 
  public:
    static void begin(void) 
    { 
      pinMode(PIN, INPUT_PULLUP); 
      reset();
#if defined(ARDUINO_ARCH_AVR)
      enableInterrupt(PIN, ISR_Change<PIN>, CHANGE);
#elif defined(ESP8266)      
      attachInterrupt(digitalPinToInterrupt(PIN), PWM_ISR_Change<PIN>, CHANGE);
#endif
    }
    
    static void end(void) 
    {
#if defined(ARDUINO_ARCH_AVR)
      disableInterrupt(PIN);
#elif defined(ESP8266)      
      detachInterrupt(digitalPinToInterrupt(PIN));
#endif
    }

	static bool valid(void)
	{
		return pulses.prevRiseTime && pulses.prevFallTime;
	}

	static float dutyCycle(bool rst = false) // returns the duty cycle of PWM signal in %
    {
      float dutyCalc(Cycle);
      float r=dutyCalc(*(const_cast<Cycle *>(&pulses))); // external function to calculate the duty to avoid bloating the all static class that will be generated for each obseved pin
		// unsigned long pulseWidth = pw();
/*		float r = 0.0;
		noInterrupts();
		if (pulses.prevRiseTime && pulses.prevFallTime)
		{
			unsigned long pulseWidth = ((pulses.riseTime - pulses.prevRiseTime) + (pulses.fallTime - pulses.prevFallTime)) >> 1;
			r = pulseWidth ? (dutyL() * 100.0) / pulseWidth : 0.0;
		}
		interrupts();
		*/
	  if(rst)
        reset();
      return r;
    }

    static float prf(void)  // returns the Pulse Repetition Frequency (PRF) in Hz, returns UNDETERMINED_FREQ if can not determine
    {
      float prfCalc(Cycle);
      return prfCalc(*(const_cast<Cycle *>(&pulses))); // external function to calculate the frequency to avoid bloating the all static class that will be generated for each obseved pin
/*		if (valid())
		{
			unsigned long pulseWidth = pw();
			return pulseWidth ? 1000000.0 / pulseWidth : UNDETERMINED_FREQ; // the frequency in Hz, according to the duration of the last whole pulse
		}
		else
			return UNDETERMINED_FREQ;
*/
	}
	
    static void reset() // resets the counters
    {
		pulses= { 0UL, 0UL, 0UL, 0UL };
    }

    friend void ISR_Change<PIN>(void); // enable acces to the private static variables to the ISR routine 

#ifndef DEBUG_ACCESS
  private:
#endif

  volatile static struct ICACHE_RAM_ATTR Cycle pulses;
};

template <uint8_t PIN> volatile Cycle PWMinfo<PIN>::pulses = { 0UL, 0UL, 0UL, 0UL };

// class end
///////////////////////////////////////////////////////////////////


float dutyCalc(Cycle pulses) // returns the duty cycle od PWM signal in %, 0 if can not determine
{
	float r = -1.0;
	if (pulses.prevRiseTime && pulses.prevFallTime && pulses.prevFallTime<pulses.riseTime && pulses.fallTime > pulses.prevRiseTime)
	{
		unsigned long f_pr = pulses.fallTime - pulses.prevRiseTime;
		unsigned long r_pf = pulses.riseTime - pulses.prevFallTime;
		unsigned long dutyCycle;
		if (f_pr > r_pf)
			dutyCycle = (f_pr - r_pf) >> 1;
		else
			dutyCycle = f_pr;
//		unsigned long pulseWidth = pulses.riseTime - pulses.prevRiseTime;
//		unsigned long pulseWidth = ((pulses.riseTime - pulses.prevRiseTime) + (pulses.fallTime - pulses.prevFallTime)) >> 1;
		unsigned long pulseWidth = (f_pr+r_pf) >> 1;

		if(pulseWidth)
			r = (dutyCycle * 100.0) / pulseWidth;
//			r = (float)dutyCycle;
	}
	return r;
}


float prfCalc(Cycle pulses)  // returns the frequency in Hz, UNDETERMINED_FREQ if can not determine
{
	if (pulses.prevRiseTime && pulses.prevFallTime && pulses.prevFallTime<pulses.riseTime && pulses.fallTime > pulses.prevRiseTime)
	{
		unsigned long pulseWidth = ((pulses.riseTime - pulses.prevRiseTime) + (pulses.fallTime - pulses.prevFallTime)) >> 1;
		if(pulseWidth)
			return 1000000.0 / pulseWidth; // the frequency in Hz, according to the duration of the last whole pulse
	}
	return UNDETERMINED_FREQ;
}


/*
void reset(volatile Cycle &pulses) // resets the counters
{
	//      noInterrupts();
	pulses.dutyCycleSum = 0L;
	pulses.pulseCounter = 0L;
	pulses.firstPulseTime = 0L;
	//      interrupts();
}
*/
#endif
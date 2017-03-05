/*
Library for simple PWM signal measuring

(c) Dejan Gjorgjevikj, 2017

*/

#ifndef PWMSENSE_H
#define PWMSENSE_H

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

#define PWMmonitor(obj,pin) typedef PWMinfo<pin> obj;

// The data filled in intterupt handler packed in one single structure
struct Pulse
{
  unsigned long fallTime; // time of last pulse fall event
  unsigned long riseTime; // time of last pulse rise event
  unsigned long dutyCycleSum;   // sum of time while signal was high
  unsigned long firstPulseTime; // record of time when counting begins (can be on either ris or fall event)
  unsigned long pulseCounter:31; // count of fall evnets
  unsigned long firstPulseIsRising:1; // true - start on rise, false start on fail
/*
  float getDuty() volatile // returns the duty cycle od PWM signal in %, 0 if can not determine
  {
    unsigned long ds = dutyCycleSum;
    if(pulseCounter == 1 && firstPulseIsRising) // not a whole cycle has passed yet
    return dutyCycleSum / (micros() - firstPulseTime);
        
    if(ds && firstPulseIsRising && fallTime > riseTime) // the last cycle has not finished, but duty part was already added on fall
      ds -= fallTime - riseTime;
    unsigned long period = (firstPulseIsRising ? riseTime : fallTime) - firstPulseTime;
    return period ? (100.0 * ds / period) : 0.0;
  }

  float getPRF() volatile  // returns the frequency in Hz, UNDETERMINED_FREQ if can not determine
  {
    unsigned long pc = pulseCounter;
    unsigned long period;

    if(!pc)
      return UNDETERMINED_FREQ;
    if(firstPulseIsRising)
    {
      period = riseTime - firstPulseTime;
      if(fallTime > riseTime) // we are at low but cycle has not finished yet, one more duty pulse is counted!
        pc--;
    }
    else
    {
      period = fallTime - firstPulseTime;
      pc--;
    }
    return period ? pc * 1000000.0 / period : UNDETERMINED_FREQ;
  }
*/
};

// The interrupt handler functions per pin and the class handling the sensing (reding the recorded counters by the interrupt handlers and calculating the duty/frequency) are templates 
template <uint8_t PIN> 
class PWMinfo;

//#pragma GCC push_options
//#pragma GCC optimize ("-O4")
template <uint8_t PIN>
void __attribute__ ((optimize("-O4"))) ICACHE_RAM_ATTR PWM_ISR_Change(void) 
//void ICACHE_RAM_ATTR PWM_ISR_Change(void) 
{
#if defined(ESP8266)      
  bool PinState = digitalRead(PIN);
#else
  bool PinState = !!arduinoPinState;
#endif
  unsigned long t = micros();
  
  if(!PWMinfo<PIN>::pulses.firstPulseTime)
  {
    PWMinfo<PIN>::pulses.firstPulseTime = PWMinfo<PIN>::pulses.riseTime = t;
    PWMinfo<PIN>::pulses.firstPulseIsRising = PinState;
  } 

  if(PinState) // rise
  {
    PWMinfo<PIN>::pulses.riseTime = t;
  }
  else //fall
  {
    PWMinfo<PIN>::pulses.fallTime = t;
    PWMinfo<PIN>::pulses.pulseCounter++;
    PWMinfo<PIN>::pulses.dutyCycleSum += (t - PWMinfo<PIN>::pulses.riseTime);
  }
}
//#pragma GCC pop_options

template <uint8_t PIN> // templetized by pin, so there will be a separate class / object for every monitored pin
class PWMinfo
{
  private:
    PWMinfo() {} // no need for constructor - everything is static, since every class will have a single instance 
  public:
    static void begin(void) 
    { 
      pinMode(PIN, INPUT_PULLUP); 
      Reset();
#if defined(ARDUINO_ARCH_AVR)
      enableInterrupt(PIN, PWM_ISR_Change<PIN>, CHANGE);
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

    // returns the time past (in microseconds) since the counting started == since the first pulse (rising or falling edge) was noticed
    // returns 0 if called befor the first pulse has even appeared
    static unsigned long int TimeCounting(void)
    {
      return (pulses.firstPulseTime) ? (micros() - pulses.firstPulseTime) : 0;
    }

    // returns the number of noticed falling edges
    // returns 0 if called befor the first pulse has even appeared
    static unsigned long int Pulses(void)
    {
      return pulses.pulseCounter;
    }

	static float Duty(bool rst = false) // returns the duty cycle of PWM signal in %
    {
//      float Duty(const volatile pulse &);
      float dutyCalc(Pulse);
      if(!pulses.pulseCounter) // if no pulses have been recorded return duty according to the current state of the pin
        return digitalRead(PIN) ? 100.0 : 0.0;
//      pulse safe_copy = *(const_cast<pulse *>(&pulses));
      float r=dutyCalc(*(const_cast<Pulse *>(&pulses))); // external function to calculate the duty to avoid bloating the all static class that will be generated for each obseved pin
//      float r=pulses.getDuty();
      if(rst)
        Reset();
      return r;
    }

    static float PRF(void)  // returns the Pulse Repetition Frequency (PRF) in Hz, returns UNDETERMINED_FREQ if can not determine
    {
      float prfCalc(Pulse);
//      pulse safe_copy = *(const_cast<pulse *>(&pulses));
      return prfCalc(*(const_cast<Pulse *>(&pulses))); // external function to calculate the frequency to avoid bloating the all static class that will be generated for each obseved pin
//      return pulses.getPRF();
    }
	
    static void Reset() // resets the counters
    {
//		void reset(volatile Pulse &);
//		reset(pulses);
//      noInterrupts();
      pulses.dutyCycleSum = 0L;
	  pulses.pulseCounter = 0L;
      pulses.firstPulseTime = 0L;
//      interrupts();
    }

    friend void PWM_ISR_Change<PIN>(void); // enable acces to the private static variables to the ISR routine 

#ifndef DEBUG_ACCESS
  private:
#endif

  volatile static struct ICACHE_RAM_ATTR Pulse pulses;
};

template <uint8_t PIN> volatile Pulse PWMinfo<PIN>::pulses = {0UL, 0UL, 0UL, 0UL, 0, 0};

// class end
///////////////////////////////////////////////////////////////////

float dutyCalc(Pulse pulses) // returns the duty cycle od PWM signal in %, 0 if can not determine
{
  unsigned long ds = pulses.dutyCycleSum;
  if(pulses.pulseCounter == 1 && pulses.firstPulseIsRising) // not a whole cycle has passed yet
    return pulses.dutyCycleSum / (micros() - pulses.firstPulseTime);
        
  if(ds && pulses.firstPulseIsRising && pulses.fallTime > pulses.riseTime) // the last cycle has not finished, but duty part was already added on fall (timer rolover unsafe!)
    ds -= pulses.fallTime - pulses.riseTime;
  unsigned long period = (pulses.firstPulseIsRising ? pulses.riseTime : pulses.fallTime) - pulses.firstPulseTime;
  
  return period ? (100.0 * ds / period) : 0.0;
}

float prfCalc(Pulse pulses)  // returns the frequency in Hz, UNDETERMINED_FREQ if can not determine
{
  unsigned long pc = pulses.pulseCounter;
  unsigned long period = 0;

  if(pc) 
  {
    if(pulses.firstPulseIsRising)
    {
      period = pulses.riseTime - pulses.firstPulseTime;
      if(pulses.fallTime > pulses.riseTime) // we are at low but cycle has not finished yet, one more duty pulse is counted! (timer rolover unsafe!)
        pc--;
    }
    else
    {
      period = pulses.fallTime - pulses.firstPulseTime;
      pc--;
    }
  }
  return period ? pc * 1000000.0 / period : UNDETERMINED_FREQ;
}

void reset(volatile Pulse &pulses) // resets the counters
{
	//      noInterrupts();
	pulses.dutyCycleSum = 0L;
	pulses.pulseCounter = 0L;
	pulses.firstPulseTime = 0L;
	//      interrupts();
}

#endif
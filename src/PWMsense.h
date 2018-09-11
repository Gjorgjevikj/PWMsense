/*
Library for simple PWM signal measuring

Estimates the frequency and the duty of a PWM signal
based on timing of the rising and falling edges of the signal 
using interrupts during multiple cycles

This library is distributed in the hope that it will be useful, but
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE either express or implied.
Released into the public domain.
Distributed under: GNU General Public License v3.0

(c) Dejan Gjorgjevikj, 2017
revised: 23.08.2018
revised: 11.09.2018 - simplification, renaming
*/

#ifndef PWMSENSE_H
#define PWMSENSE_H

#define PWMSENSE_VER 0.92.0.1

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

// To enable pinchange intterupt on any pin on Arduino EnableIntterupt library from GrayGnome 
// https://github.com/GreyGnome/EnableInterrupt.git is used 

//#define LIBCALL_ENABLEINTERRUPT
#if defined(ARDUINO_ARCH_AVR)
#define EI_ARDUINO_INTERRUPTED_PIN
#include <EnableInterrupt.h>
#define ICACHE_RAM_ATTR
#endif

// the value to be returned from the frequency sense functions when the frequency can not be determined
#define UNDETERMINED_FREQ 0.0

#define PWMmonitor(obj,pin) typedef PWMsense<pin> obj;

// The data filled in intterupt handler packed in one single structure

struct Pulse
{
  unsigned long fallTime; // time of last pulse fall event
  unsigned long riseTime; // time of last pulse rise event
  unsigned long dutyCycleSum;   // sum of time while signal was high
  unsigned long firstPulseTime; // record of time when counting begins (can be on either rise or fall event)
  unsigned long pulseCounter:31; // count of fall evnets
  unsigned long firstPulseIsRising:1; // 1 (true) - start on rise, 0 (false) - start on fail
};

// The interrupt handler functions per pin and the class handling the sensing 
// (reding the recorded counters by the interrupt handlers and calculating the duty/frequency) are templates 
template <uint8_t PIN> 
class PWMsense;

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
  
  if(!PWMsense<PIN>::pulses.firstPulseTime)
  {
    PWMsense<PIN>::pulses.firstPulseTime = PWMsense<PIN>::pulses.riseTime = t;
    PWMsense<PIN>::pulses.firstPulseIsRising = PinState;
  } 

  if(PinState) // rise
  {
    PWMsense<PIN>::pulses.riseTime = t;
  }
  else //fall
  {
    PWMsense<PIN>::pulses.fallTime = t;
    PWMsense<PIN>::pulses.pulseCounter++;
    PWMsense<PIN>::pulses.dutyCycleSum += (t - PWMsense<PIN>::pulses.riseTime);
  }
}

/// <summary> PWMsense Template Class 
/// templetized by pin, so there will be a separate class / object / ISR for every monitored pin
/// </summary>
/// <remarks>
/// Monitors a signal on a given pin continually recording the time the signal is high and low and counting the pulses 
/// so thet a more precise estimate of the frequency and the duty can be calculated even on unstabile signals
/// </remarks>
template <uint8_t PIN> 
class PWMsense
{
  private:
    PWMsense() {} // no need for constructor - everything is static, since every class will have a single instance 
	volatile static struct ICACHE_RAM_ATTR Pulse pulses;

  public:
	/// <summary> Initializes the monitoring object and starts continual monitoring </summary>
    static void begin(void) 
    { 
      pinMode(PIN, INPUT_PULLUP); 
      reset();
#if defined(ARDUINO_ARCH_AVR)
      enableInterrupt(PIN, PWM_ISR_Change<PIN>, CHANGE);
#elif defined(ESP8266)      
      attachInterrupt(digitalPinToInterrupt(PIN), PWM_ISR_Change<PIN>, CHANGE);
#endif
    }
    
	/// <summary> Stops monitoring </summary>
	static void end(void)
    {
#if defined(ARDUINO_ARCH_AVR)
      disableInterrupt(PIN);
#elif defined(ESP8266)      
      detachInterrupt(digitalPinToInterrupt(PIN));
#endif
    }

	/// <summary> Gives the time the signal is beeing monitored 
	/// or since the last reset of monitoring </summary>
	/// <returns> the time past (in microseconds) since the first transition </returns>
	/// <remark> i.e. since the first pulse (rising or falling edge) was noticed 
    /// returns 0 if called before the first pulse has even appeared 
	/// will return 0 in a case of constant signal! </remark>
    static unsigned long int timeCounting(void)
    {
      return (pulses.firstPulseTime) ? (micros() - pulses.firstPulseTime) : 0;
    }

	/// <summary> Gives the number of noticed falling edges since the monitoring has started 
	/// (or has been reseted) </summary>
	/// <returns> the number of noticed falling edges
	/// returns 0 if called befor the first pulse has even appeared </returns>
    static unsigned long int pulseCount(void)
    {
      return pulses.pulseCounter;
    }

	/// <summary> Returns the duty cycle of PWM signal in % </summary>
	/// <param name="rst"> whrethere the mesurments should be resesed after the call </param>
	/// <returns> the duty in percents 0-100 </returns>
	static float duty() 
    {
      float PWMsenseDutyCalc(Pulse);
      if(!pulses.pulseCounter) // if no pulses have been recorded return duty according to the current state of the pin
        return digitalRead(PIN) ? 100.0 : 0.0;
      float r=PWMsenseDutyCalc(*(const_cast<Pulse *>(&pulses))); // external function to calculate the duty to avoid bloating the all static class that will be generated for each obseved pin
      return r;
    }

	/// <summary> Returns the Pulse Repetition Frequency (PRF) in Hz </summary>
	/// <returns> frequency in Hz, UNDETERMINED_FREQ if can not determine </returns>
	static float frequency(void)  // returns the Pulse Repetition Frequency (PRF) in Hz, returns UNDETERMINED_FREQ if can not determine
    {
      float PWMsenseFreqCalc(Pulse);
      return PWMsenseFreqCalc(*(const_cast<Pulse *>(&pulses))); // external function to calculate the frequency to avoid bloating the all static class that will be generated for each obseved pin
    }
	
	/// <summary> Resets the object </summary>
	/// <remark> resets all the counters and timestamps
	/// starting measuring from scratch </remark>
	static void reset() // resets the counters
    {
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
		// produces slihgtly shorter code on Arduino, not accepted by ESP8266 compiler
		pulses = { 0 };
#else
		pulses.dutyCycleSum = pulses.pulseCounter = pulses.firstPulseTime = 0L;
#endif
	}

	/// <summary> Returns the pin being monitored </summary>
	/// <returns> the pin being monitored </returns> 
	static uint8_t pin()
	{
		return PIN;
	}

    friend void PWM_ISR_Change<PIN>(void); // enable acces to the private static variables to the ISR routine 

};

template <uint8_t PIN> volatile Pulse PWMsense<PIN>::pulses = {0UL, 0UL, 0UL, 0UL, 0, 0};
// template <uint8_t PIN> volatile PWMsense<PIN>::struct Pulse PWMsense<PIN>::pulses = { 0UL, 0UL, 0UL, 0UL, 0UL, 0UL };

// class end
///////////////////////////////////////////////////////////////////

/// <summary> Calculates the duty of the signal </summary>
/// <param name="pulses"> Pulse stucture with timings and counters of the signal </param>
/// <returns> returns the duty cycle od PWM signal in % (0-100), 0 if can not determine </returns>
float PWMsenseDutyCalc(Pulse pulses) 
{
  unsigned long ds = pulses.dutyCycleSum;
  if(pulses.pulseCounter == 1 && pulses.firstPulseIsRising) // not a whole cycle has passed yet
    return pulses.dutyCycleSum / (micros() - pulses.firstPulseTime);
        
  if(ds && pulses.firstPulseIsRising && pulses.fallTime > pulses.riseTime) // the last cycle has not finished, but duty part was already added on fall (timer rolover unsafe!)
    ds -= pulses.fallTime - pulses.riseTime;
  unsigned long period = (pulses.firstPulseIsRising ? pulses.riseTime : pulses.fallTime) - pulses.firstPulseTime;
  
  return period ? (100.0 * ds / period) : 0.0;
}

/// <summary> Calculates the frequency of the signal </summary>
/// <param name="pulses"> Pulse stucture with timings and counters of the signal </param>
/// <returns> returns the frequency in Hz, UNDETERMINED_FREQ if can not determine </returns>
float PWMsenseFreqCalc(Pulse pulses)  // returns the frequency in Hz, UNDETERMINED_FREQ if can not determine
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

// Some usefull functions

/// <summary> Estimeates the frequency recording n pulses </summary>
/// <param name="t"> maximum time in milliseconds to wait for the cycles to end </param>
/// <param name="n"> number of cycles to wait </param>
/// <returns> the frequency in Hz, or the best estimate before it timedout </returns>
template <uint8_t PIN>
float FreqEstimate(unsigned long t, unsigned int n = 1)
{
	typedef PWMsense<PIN> pwmMonitor;
	pwmMonitor::begin();
	n++; // we need at least 2 falling edges for full cycle 
	unsigned long start = millis();
	while (pwmMonitor::pulseCount() < n && (millis() - start) < t)
		yield();
	pwmMonitor::end();
	return pwmMonitor::frequency();
}

/// <summary> Estimeates the duty recording n pulses </summary>
/// <param name="t"> maximum time in milliseconds to wait for the cycles to end </param>
/// <param name="n"> number of cycles to wait </param>
/// <returns> the duty in % 1-100, ? if timedout </returns>
template <uint8_t PIN>
float DutyEstimate(unsigned long t, unsigned int n = 1)
{
	typedef PWMsense<PIN> pwmMonitor;
	pwmMonitor::begin();
	n++; // we need at least 2 falling edges for full cycle 
	unsigned long start = millis();
	while (pwmMonitor::pulseCount() < n && (millis() - start) < t)
		yield();
	pwmMonitor::end();
	return pwmMonitor::duty();
}

#endif
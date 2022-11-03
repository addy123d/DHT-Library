/*
 * DHT-11 Temperature Sensor Library
 */

/*
 * The MIT License (MIT)
 * Copyright (c) 2022 Aditya Chaudhary and contributors
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE
 */

#include "DHT.h"

#define MIN_INTERVAL 2000 /**< min interval value */
#define TIMEOUT
UINT32_MAX /**< Used programmatically for timeout.
                 Not a timeout duration. Type: uint32_t. */

DHT::dht_sensor(uint8_t pin, uint8_t type)
{

  _pin = pin;
  _type = type;

#ifdef __AVR
  _bit = digitalPinToBitMask(pin);
  _port = digitalPinToPort(pin);
#endif

  _maxCycles = microsecondsToClockCycles(1000);

  /*
   * 1 millisecond timeout for
   * reading pulses from DHT sensor.
   * Note that count is now ignored as the DHT reading algorithm adjusts itself
   * based on the speed of the processor.
   */
}

/*!
 * @brief  Setup sensor pins and set pull timings
 * @param  micro_seconds
 *          Optionally pass pull-up time (in microseconds) before DHT reading
 *          starts. Default is 55 (see function declaration in DHT.h).
 */
void DHT::init(uint8_t micro_seconds) {
  // set up the pins!
  pinMode(_pin, INPUT_PULLUP);

  _lastreadtime = millis() - MIN_INTERVAL;
  pullUptime = micro_seconds;
}


float DHT::readTemp(bool fahrenheit, bool force){
 if(read(force)) return data;
}


/*!
   * @brief : Read value from sensor, or  return last reading, if less than two seconds.
   * @param force: If using force mode
   * @return: bool value
*/
bool DHT::read_sensor(bool force)
{

  /*
   * Reading from the sensor will done in steps:
   * Check whether sensor was read less than 2 secs ago, return lastreading
   * Reset all 40 bits to zero.
   * To understand below steps, refer datasheet: https://www.mouser.com/datasheet/2/758/DHT11-Technical-Data-Sheet-Translated-Version-1143054.pdf
   * Send Start Signal, as DHT will be receiving start signal, set pin to INPUT_PULLUP Mode. DHT changes from low power consumption mode to running mode.
   * Change the pinMode to OUTPUT, as DHT will be in position to send response, Set data line to LOW for a period according to sensor you are using.
   * Read 40 bits from sensor.
   * Collect all pulses, and examine which bit is 0 and 1 .
   * Bit is 1, if the data line is high for more than 70 usec.
   * Bit is 0, if the data line is high for ~26-28 usec.
   * Approach we will be following, if high_state_cyclecount > low_state_cyclecount, then it is 1, otherwise 0 .
   */

  /* Return, if you are reading too early. */

  uint32_t now_time = millis();

  if ((now_time - _lastreadtime) < MIN_INTERVAL)
    return _lastresult; /* return last reading. */


  /* Timer Update */

  _lastreadtime = currenttime;


  /*Reset data bits*/

  for (int i = 0; i < 5; i++)
    	data[i] = 0;


  /* Communication Starts */
  /* Start Reading from DHT Side */

  pinMode(_pin, INPUT_PULLUP);
  delay(1);

  /* 
   * Send Start Signal On Same Pin and wait for some period, for e.g it is 18ms in case of DHT11
   */

  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);

  switch (_type){

  	case DHT22:
  	case DHT21:
    		delayMicroseconds(1100);
    		break;
  	case DHT11:
  	default:
    		delay(19);
    		break;

  }


  /*
   * Read 40 bit data by number of cycles method
   * Collect all the pulses, low pulse followed by HIGH pulse, return timeout when, time exceeds for respective pulse
   */

  uint32_t pulses[80]; //Why 80 pulses? Because DHT-11 will send response for 80usecs and we are converting microseconds to cycles.


  /* If we are getting LOW pulse for longer time, return false. */

  if(read_pulse(LOW) == TIMEOUT) return false;
  

  /* If we are getting HIGH pulse for longer time, return false. */

  if(read_pulse(HIGH) == TIMEOUT) return false;

  for(int i = 0; i < 80; i++){

	  /* LOW followed by HIGH */

  	pulses[i] = read_pulse(LOW);
	  pulses[i+1] = read_pulse(HIGH);
  }



 /*
  * It's time to inspect all the pulses, which we have stored.
  */

  uint32_t lowCycles, highCycles;

  for(int i = 0; i < 40; i++){
  	lowCycles = pulses[2 * i];
	  highCycles = pulses[(2 * i) + 1];
  

  	//Check for timeout.
  	if((lowCycles == TIMEOUT) || (highCycles == TIMEOUT)) return false;

  	data[i / 8] <<= 1; //Store all the data

  	if(highCycles > lowCycles) data[i / 8] |= 1;

  }
}


uint32_t DHT::read_pulse(bool level){

	#if (F_CPU > 16000000L) || (F_CPU == 0L)
		uint32_t count_of_cycles = 0;
	#else
		uint16_t count_of_cycles = 0;
	#endif

	/*
	 * There are two methods: 
	 * :1: Directly access GPIO port
	 * :2: Use digitalRead() function
	 */
	
	#ifdef __AVR
	   uint8_t current_portState = level ? bit : 0;
	   
	   while((*portInputRegister(_port) & _bit) == current_portState)
		   if(count_of_cycles++ >= _maxCycles)
			   return TIMEOUT;
	#else	
  
      	   while(digitalRead(_pin) == level)
            	   if(count_of_cycles++ >= _maxCycles)
          		   return TIMEOUT;
	
	#endif
	
	return count_of_cycles;


}



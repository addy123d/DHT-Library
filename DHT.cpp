/*
 * DHT Library
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

#define MIN_INTERVAL 2000  /**< min interval value */
#define TIMEOUT UINT32_MAX /**< Used programmatically for timeout. Not a timeout duration. Type: uint32_t. */

DHT::DHT(uint8_t pin, uint8_t type)
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
 * @brief:  Setup sensor pins and set pull timings
 * @param:  1.micro_seconds
 *          Optionally pass pull-up time (in microseconds) before DHT reading
 *          starts. Default is 55 (see function declaration in DHT.h).
 *
 *          2. test, @keep this false in production mode.
 *          If you want to test, whether is working or not, tune it to true, else leave it as it is !
 *
 *          3. alert_pin, By Default, alert_pin = 13
 *          Pin will be HIGH when sensor is not working and viceversa.
 */
void DHT::init(uint8_t micro_seconds, bool test, uint8_t alert_pin)
{
  // set up the pin.
  pinMode(_pin, INPUT_PULLUP);

  _lastreadtime = millis() - MIN_INTERVAL;
  pullUptime = micro_seconds;

  // User should use different pins for DHT and alert_pin.
  if (_pin == alert_pin)
  {
    Serial.println("Pin Error: Use different pin for test");
    return;
  }

  sensor_test(test, alert_pin);
}

/*!
 * @brief: Test whether sensor is OK, don't use this function in production.
 * @param: None
 * @return: void
 */

void DHT::sensor_test(bool test, uint8_t alert_pin)
{

  if (!test)
    return;

  Serial.println("********** Test Begins **********");

  float tempIn_fahrenheit = getTemperature(true);
  float tempIn_Celsius = getTemperature();
  float tempIn_Kelvin = getKelvin_temperature();
   float humidity = getHumidity();

  if (isnan(tempIn_fahrenheit) || isnan(tempIn_Celsius) || isnan(tempIn_Kelvin))
  {
    /*Indicate user, that sensor is not working */
    indicator(alert_pin, 2);

    Serial.println("Test Failed: DHT is not responding");
    return;
  }

  String temp_in_F = "Temperature in Fahrenheit: ";
  temp_in_F += tempIn_fahrenheit;
  temp_in_F += " F";

  String temp_in_C = "Temperature in Celsius: ";
  temp_in_C += tempIn_Celsius;
  temp_in_C += " C";
  String temp_in_Kelvin = "Temperature in Kelvin: ";
  temp_in_Kelvin += tempIn_Kelvin;
  temp_in_Kelvin += " K";

  String humidityReading = "Humidity Reading: ";
  humidityReading += humidity;
  humidityReading += " %";

  Serial.println(temp_in_F);
  Serial.println(temp_in_C);
  Serial.println(temp_in_Kelvin);
  Serial.println(humidityReading);

  Serial.println("********** Test Success **********");
}

/*!
 * @brief: Indicates that sensor is not communicating properly.
 *
 * @param: @alert_pin [type: uint8_t] - you can attach led or buzzer for indication purpose.
 * 	        @notification_count [type: int] - number of times you want to get notified.
 *
 * @return: void.
 */

void DHT::indicator(uint8_t alert_pin, int notification_count)
{

  int count = 0;
  pinMode(alert_pin, OUTPUT);

  while (1)
  {
    /* Alert */
    {
      digitalWrite(alert_pin, HIGH);
      delay(500);
      digitalWrite(alert_pin, LOW);
      delay(500);

      if (count == notification_count)
        break;
    }

    count++;
  }
}

/*!
 * @brief: Reads Temperature Data from sensor, data[2]: 8 bit integral temperature data; data[3]: 8 bit decimal temperature data.
 * @param: fahrenheit, force
 * @return: temperature reading in float.
 */

float DHT::getTemperature(bool fahrenheit, bool force)
{

  /* Sampling Period is 1sec, create a delay for around 1 sec */
  delay(2000); //@update : @addy123d

  float f = NAN;
  if (read_sensor(force))
  {

    f = data[2]; // 8 bit Integral temperature data

    /*
     * For DHT11, DHT12, DHT22
     */

    if ((data[3] & 0x80) && ((_type == DHT11) || (_type == DHT22)))
      f = -1 - f; // Conversion to positive and subtracting by 1

    if ((data[3] & 0x80) && ((_type == DHT12) || (_type == DHT22)))
      f *= -1; // Conversion to positive

    // Adding precision, like after 24.00, you should get values 24.110, 24.90 etc.
    f += (data[3] & 0x0f) * 0.1; // Ignoring first 4 bits * 0.1

    /*
     * Default is Celsius, Conversion to Fahrenheit
     */
    if (fahrenheit)
      f = f * 1.8 + 32;
  }

  return f;
}

/*!
 * @brief: Get Temperature in Kelvin
 * @param: None
 * @return: Kelvin Temperature in float.
 */
float DHT::getKelvin_temperature()
{
  return (getTemperature() + 273.15);
}

/*!
 * @brief : Get Humidity readings from sensor.
 * @param : force, read mode
 * @return : humidity value in float
 */
float DHT::getHumidity(bool force)
{

  delay(2000);

  float humidity = NAN;
  humidity = data[0]; // Integral humidity data.

  if (read_sensor(force))
    humidity += (data[1] & 0x0f) * 0.1;

  return humidity;
}

/*!
 * @brief: calculate heat index, important considerations for the human body's comfort.
 * @param: temperature [type: float], humidity [type: float], isFahrenheit [type: boolean]
 * @return: value of heat index in float
 */
float DHT::heatIndex(float temperature, float humidity, bool isFahrenheit)
{

  if (!isFahrenheit)
    temperature = (temperature * 1.8) + 32; // Convert to Fahrenheit

  /* Reference: https://en.wikipedia.org/wiki/Heat_index#Formula && https://github.com/adafruit/DHT-sensor-library/blob/master/DHT.cpp
   * Formula: Heat Index = c1 + c2T + c3R + c4TR + c5T^2 + c6R^2 + c7T^2R + c8TR^2 + c9T^2R^2
   * where temperature is in Fahrenheit
   * c1 = -42.379, c2 = 2.04901523, c3 = 10.14333127
   * c4 = -0.22475541, c5 = -6.83783 * 10^-3, c6 = -5.481717 * 10^-2
   * c7 = 1.22874 * 10^-3, c8 = 8.5282 * 10^-4, c9 = -1.99 * 10^-6
   */

  float heat_index = 0.5 * (temperature + 61.0 + ((temperature - 68.0) * 1.2) +
                            (humidity * 0.094));

  if (heat_index > 79)
  {

    heat_index = -42.379 + 2.04901523 * temperature + 10.14333127 * humidity +
                 -0.22475541 * temperature * humidity +
                 -0.00683783 * pow(temperature, 2) +
                 -0.05481717 * pow(humidity, 2) +
                 0.00122874 * pow(temperature, 2) * humidity +
                 0.00085282 * temperature * pow(humidity, 2) +
                 -0.00000199 * pow(temperature, 2) * pow(humidity, 2);

    if ((humidity < 13) && (temperature >= 80.0) && (temperature <= 112.0))
      heat_index -= ((13.0 - humidity) * 0.25) * sqrt((17.0 - abs(temperature - 95.0)) * 0.05882);

    else if ((humidity > 85.0) && (temperature >= 80.0) && (temperature <= 87.0))
      heat_index += ((humidity - 85.0) * 0.1) * ((87.0 - temperature) * 0.2);
  }

  if (!isFahrenheit)
    return ((heat_index - 32) * 0.55555); // In degree Celsius

  return heat_index; // In fahrenheit
}

/*!
 * @brief: Calculate Dew Point.
 * @param: None.
 * @return: dew point temperature in double.
 */

double DHT::dewPoint()
{
  float temperature = getTemperature();
  float humidity = getHumidity();

  /*
   * @ref "https://wahiduddin.net/calc/density_algorithms.htm"
   * @formula: -C1*(TS/TK-1.)+C2*ALOG10(TS/TK)-C3*(10.**(C4*(1.-TK/TS))-1)+C5*(10.**(-C6*(TS/TK-1.))-1.)+ALOG10(EWS)
   * WHERE
   * DATA     C1,      C2,      C3,      C4,       C5,       C6
   *  1  / 7.90298, 5.02808, 1.3816E-7, 11.344, 8.1328E-3, 3.49149 /
   *
   *  Reference: https://media.bom.gov.au/social/upload/images/table-ppt-2.PNG
   */

  /*
   * Boiling Temperature (Tk) = 373.15 K (100 Degree Celsius)
   * Initial Temperature (T) = 273.15 K (0 Degree Celsius)
   */

  double T0 = 273.15, Tk = 373.15;

  double A0 = Tk / (T0 + temperature);


  double SUM = -7.90298 * (A0 - 1) + 
         5.02808 * log10(A0) + 
         -1.3816e-7 * (pow(10, (11.344 * (1 - 1 / A0))) - 1) +
         8.1328e-3 * (pow(10, (-3.49149 * (A0 - 1))) - 1) +
         log10(1013.246);


  double VP = pow(10, SUM - 3) * humidity;
  double T = log(VP / 0.61078); // temp var


  return (241.88 * T) / (17.558 - T);
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

  /* Return last_reading, if you are reading too early. */

  uint32_t now_time = millis();

  if ((force == false) && ((now_time - _lastreadtime) < MIN_INTERVAL))
    return _lastresult; /* return last reading. */

  /* Timer Update */

  _lastreadtime = now_time;

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

  delay(19); // Let's play safe !

  /*
   * Read 40 bit data by number of cycles method
   * Collect all the pulses, low pulse followed by HIGH pulse, return timeout when, time exceeds for respective pulse
   */

  uint32_t pulses[80]; // Why 80 pulses? As DHT-11 will send response for 80usecs.
  {
    // Turn I/O into input mode.
    pinMode(_pin, INPUT_PULLUP);

    // Delay a bit , to pull the data line low.
    delayMicroseconds(pullUptime);

    /* If we are getting LOW pulse for longer time, return false. */

    if (read_pulse(LOW) == TIMEOUT)
      return false;

    /* If we are getting HIGH pulse for longer time, return false. */

    if (read_pulse(HIGH) == TIMEOUT)
      return false;

    for (int i = 0; i < 80; i += 2)
    {
      /* LOW followed by HIGH */

      pulses[i] = read_pulse(LOW);
      pulses[i + 1] = read_pulse(HIGH);
    }
  }

  /*
   * It's time to inspect all the pulses, which we have stored.
   */

  uint32_t lowCycles, highCycles;

  for (int i = 0; i < 40; i++)
  {
    lowCycles = pulses[2 * i];
    highCycles = pulses[(2 * i) + 1];

    // Check for timeout.
    if ((lowCycles == TIMEOUT) || (highCycles == TIMEOUT))
      return false;

    data[i / 8] <<= 1; // Store data

    if (highCycles > lowCycles)
      data[i / 8] |= 1;
  }
}

uint32_t DHT::read_pulse(bool level)
{

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
  uint8_t current_portState = level ? _bit : 0;

  while ((*portInputRegister(_port) & _bit) == current_portState)
  {
    count_of_cycles++;
  }

#else

  while (digitalRead(_pin) == level)
  {
    count_of_cycles++;
  }

#endif

  if (count_of_cycles >= _maxCycles)
    return TIMEOUT;

  return count_of_cycles;
}

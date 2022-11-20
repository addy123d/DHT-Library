#include "DHTesp.h"

DHT::DHT(uint8_t pin, uint8_t type)
{
  _pin = pin;
  _type = type;
}

void DHT::init(uint8_t micro_seconds, bool test, uint8_t alert_pin){
  // User should use different pins for DHT and alert_pin.
  if (_pin == alert_pin)
  {
    Serial.println("Pin Error: Use different pin for test");
    return;
  }

  // sensor_test(test, alert_pin);
}

double DHT::getTemperature(bool fahrenheit)
{
  double temperature = NAN;

  temperature = read_sensor(true);

  if (fahrenheit)
    temperature = (temperature * 1.8) + 32;

  return temperature;
}

double DHT::getKelvin_temperature()
{
  return (read_sensor(true) + 273.15);
}

double DHT::getHumidity()
{
  return read_sensor(false);
}

double DHT::heatIndex(double temperature, double humidity)
{

  /* Reference: https://en.wikipedia.org/wiki/Heat_index#Formula && https://github.com/adafruit/DHT-sensor-library/blob/master/DHT.cpp
   * Formula: Heat Index = c1 + c2T + c3R + c4TR + c5T^2 + c6R^2 + c7T^2R + c8TR^2 + c9T^2R^2
   * where temperature is in Fahrenheit
   * c1 = -42.379, c2 = 2.04901523, c3 = 10.14333127
   * c4 = -0.22475541, c5 = -6.83783 * 10^-3, c6 = -5.481717 * 10^-2
   * c7 = 1.22874 * 10^-3, c8 = 8.5282 * 10^-4, c9 = -1.99 * 10^-6
   */

  double heat_index = 0.5 * (temperature + 61.0 + ((temperature - 68.0) * 1.2) + (humidity * 0.094));

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

  return ((heat_index - 32) * 0.55555); // In degree Celsius
}

double DHT::dewPoint()
{
  float temperature = read_sensor(true);
  float humidity = read_sensor(false);

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

double DHT::read_sensor(bool temp)
{
  delay(2000);

  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);
  delay(18);
  digitalWrite(_pin, HIGH);
  pinMode(_pin, INPUT);
  digitalWrite(_pin, HIGH);

  uint16_t rawTemperature = 0;
  uint16_t rawHumidity = 0;
  uint16_t data = 0;

  uint8_t tempi;
  uint8_t tempd;

  uint8_t humi;
  uint8_t humd;

  unsigned long startTime;

  for (int8_t i = -3; i < 80; i++)
  {
    byte live;
    startTime = micros();

    do
    {
      live = (unsigned long)(micros() - startTime);
      if (live > 90)
      {
        Serial.println("ERROR_TIMEOUT");
        return -1.0;
      }
    } while (digitalRead(_pin) == (i & 1) ? HIGH : LOW);

    if (i >= 0 && (i & 1))
    {
      data <<= 1;

      // TON of bit 0 is maximum 30 usecs and of bit 1 is at least 68 usecs.
      if (live > 30)
      {
        data |= 1; // we got a one
      }
    }

    switch (i)
    {
    case 31:
      rawHumidity = data;
      break;
    case 63:
      rawTemperature = data;
      break;
    }
  }

  if (temp)
  {
    tempi = rawTemperature >> 8;
    rawTemperature = rawTemperature << 8;
    tempd = rawTemperature >> 8;

    double result_temp = tempi + ((double)tempd / 100);
    return result_temp;
  }

  humi = rawHumidity >> 8;
  rawHumidity = rawHumidity << 8;
  humd = rawHumidity >> 8;

  double result_humidity = humi + ((double)humd / 100);

  return result_humidity;
}

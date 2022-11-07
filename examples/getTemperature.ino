#include "DHT.h"

#define DHTPIN 2     // Digital pin connected to the DHT sensor

#define DHTTYPE DHT11   // DHT 11


DHT dht(DHTPIN, DHTTYPE);

void setup() {
  // put your setup code here, to runonce:
  Serial.begin(9600);
  dht.init();
}

void loop() {
  
  //put your main code here, to run repeatedly:
  float tempIn_fahrenheit = dht.getTemperature(true);
  float tempIn_Celsius = dht.getTemperature();
  float tempIn_Kelvin = dht.getKelvin_temperature();
  
  String temp_in_F = "Temperature in Fahrenheit: ";
         temp_in_F += tempIn_fahrenheit;
         temp_in_F += " F";

  String temp_in_C = "Temperature in Celsius: ";
         temp_in_C += tempIn_Celsius;
         temp_in_C += " C";

  String temp_in_Kelvin = "Temperature in Kelvin: ";
         temp_in_Kelvin += tempIn_Kelvin;
         temp_in_Kelvin += " K";

    

  Serial.println(temp_in_F);
  Serial.println(tempIn_Celsius);
  Serial.println(temp_in_Kelvin);

  Serial.println("**********************************************************");
  

  
}
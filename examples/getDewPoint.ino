#include "DHT.h"

#define DHTPIN 2     // Digital pin connected to the DHT sensor

#define DHTTYPE DHT11   // DHT 11


DHT dht(DHTPIN, DHTTYPE);

void setup() {
  // put your setup code here, to runonce:
  Serial.begin(9600);

//  dht.init(55,true,13);
  dht.init();
}

void loop() {
  
  //put your main code here, to run repeatedly:   
  float dew_point = dht.dewPoint();   
  
  String dew_temp_in_C = "Dew Point Temperature in Celsius: ";
         dew_temp_in_C += dew_point;
         dew_temp_in_C += " C";

  Serial.println(dew_temp_in_C);

  Serial.println("**********************************************************");
  
}
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
  float tempIn_Celsius = dht.getTemperature();

  float humidity = dht.getHumidity();   
  float heatIndex = dht.heatIndex(tempIn_Celsius, humidity, false);  


  String HeatIndex_inC = "Heat Index in Celsius: ";
         HeatIndex_inC += heatIndex;
         HeatIndex_inC += " C";
  Serial.println(HeatIndex_inC);

  Serial.println("**********************************************************");
  

  
}
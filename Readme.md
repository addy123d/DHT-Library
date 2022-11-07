# DHT Sensors Official Documentation


![https://nskelectronics.in/image/cache/catalog/SENSORS/TEMP/dht11-500x500.jpg](https://nskelectronics.in/image/cache/catalog/SENSORS/TEMP/dht11-500x500.jpg)

## Description
Arduino Library for DHT sensors.

# Example

## 🌡️ Get Temperature and Humidity Values

```arduino
#include "DHT.h"

#define DHTPIN 2     // Digital pin connected to the DHT sensor

#define DHTTYPE DHT11   // DHT 11

//General Syntax: DHT dht(pin, sensor_number);
DHT dht(DHTPIN, DHTTYPE);

void setup() {
  // put your setup code here, to runonce:
  Serial.begin(9600);
  dht.init();
}

void loop(){

	float tempIn_fahrenheit = getTemperature(true); 
	float tempIn_Celsius = getTemperature();
	float tempIn_Kelvin = getKelvin_temperature();
	float humidity = getHumidity();

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

}
```

## 🥵 Get Dew Point and Heat Index Values:

```arduino
#include "DHT.h"

#define DHTPIN 2     // Digital pin connected to the DHT sensor

#define DHTTYPE DHT11   // DHT 11

//General Syntax: DHT dht(pin, sensor_number);
DHT dht(DHTPIN, DHTTYPE);

void setup() {
  // put your setup code here, to runonce:
  Serial.begin(9600);
  dht.init();
}

void loop(){

	float tempIn_Celsius = getTemperature();
	float humidity = dht.getHumidity();

	float dew_point = dht.dewPoint();   
  float heatIndex = dht.heatIndex(tempIn_Celsius, humidity, false);

	String dew_temp_in_C = "Dew Point Temperature in Celsius: ";
         dew_temp_in_C += dew_point;
         dew_temp_in_C += " C";

  String HeatIndex_inC = "Heat Index in Celsius: ";
         HeatIndex_inC += heatIndex;
         HeatIndex_inC += " C";

	Serial.println(dew_temp_in_C);
  Serial.println(HeatIndex_inC);

}
```

## 🧪 Sensor Test Code with 3 Blink Indication

```arduino
#include "DHT.h"

#define DHTPIN 2     // Digital pin connected to the DHT sensor

#define DHTTYPE DHT11   // DHT 11

//General Syntax: DHT dht(pin, sensor_number);
DHT dht(DHTPIN, DHTTYPE);

void setup() {
  // put your setup code here, to runonce:
  Serial.begin(9600);

	//dht.init(pulltime_in_usecs, sensor_test, notification_pin);
  dht.init(55, true, 13);
}

void loop(){}
```

## ❌ 3 Blink Indication

[Sensor is not working.mp4](DHT%20Sensors%20Official%20Documentation%2037f201ef8aae49f99ad52dc9482888b7/Sensor_is_not_working.mp4)

<aside>
🔴 Support for DHT-21 is not added.

</aside>

# Issues and Bugs 🐛

Please report any issues or bugs here:

[https://github.com/addy123d/DHT-Sensors](https://github.com/addy123d/DHT-Sensors)

# Contributions

Contributions are welcome!

# License

Copyright (c) 2022 Aditya Chaudhary (ac3101282@gmail.com) and contributors. Available under the MIT License. For more information, see `LICENSE`.
# Integrated_treatment_system
It is an industrial monitoring kit for monitoring 5 water components plus light and C02
Ph
Temp
Dissolved oxygen
Electric conductivity
ORP-oxidation reduction potential
Spectral light
C02
Also controls pumps via matlab live scripts and thingspeak

The two_sensor file holds the arduino nano 33 iot code to get the AS7341 and the SCD30 C02 sensors. The data is collected via i2c to the arduino on pins D18 Aand D19 (THE DEFAULT I2C PINS). This data is then sent to the ESP8266 via UART for POSTING.

The Industrial kit code is the main code to get all the data to push to thingspeak.
There are 3 channels used in thingspeak which can be seen in the sim800 posting function.

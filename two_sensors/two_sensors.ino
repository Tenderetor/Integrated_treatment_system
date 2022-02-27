#include <Wire.h>
#include "Arduino.h"
#include <Adafruit_AS7341.h>
#include <ArduinoJson.h>
#include "SparkFun_SCD30_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_SCD30

#define LEDPIN 4
SCD30 airSensor;
Adafruit_AS7341 as7341;

uint16_t readings[12];
float counts[12];
uint16_t carbon;
String output; // data to be sent to esp

void CreateJsonString();

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT); //we want the builtin led to flicker as data is being transmitted. if its not flickering then the mcu may have encountered an error
  pinMode(LEDPIN, OUTPUT);
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial.println("SCD30 Example");
  Wire.begin();

  if (airSensor.begin() == false)
  {
    Serial.println("Air sensor not detected. Please check wiring. Freezing...");
    digitalWrite(LED_BUILTIN, HIGH);   
    while (1)
    {
      digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(10);                       // wait 
      digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
      delay(10);   
    }
  }
  
  if (!as7341.begin()){
    Serial.println("Could not find AS7341");
    digitalWrite(LED_BUILTIN, HIGH);   
    while (1)
    {
      digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(10);                       // wait 
      digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
      delay(10);   
    }
  }
  
  as7341.setATIME(100);
  as7341.setASTEP(999);
  as7341.setGain(AS7341_GAIN_64X);
}

void loop()
{
  digitalWrite(LED_BUILTIN, HIGH);  
  // digitalWrite(LEDPIN, HIGH);
  // delay(3000);
  delay(100);                       

  if (airSensor.dataAvailable())
  {
    Serial.print("co2(ppm):");
    carbon = airSensor.getCO2();
    Serial.print(carbon);
    
    Serial.print(" temp(C):");
    Serial.print(airSensor.getTemperature(), 1);

    Serial.print(" humidity(%):");
    Serial.print(airSensor.getHumidity(), 1);

    Serial.println();
  }
  else
    Serial.println("Waiting for new data");


  // put your main code here, to run repeatedly:

  if (!as7341.readAllChannels(readings)){
    Serial.println("Error reading all channels!");
    return;
  }

  for(uint8_t i = 0; i < 12; i++) {
    if(i == 4 || i == 5) continue;
    // we skip the first set of duplicate clear/NIR readings
    // (indices 4 and 5)
    counts[i] = as7341.toBasicCounts(readings[i]);
  }

  //  digitalWrite(LEDPIN, LOW); //this the LED from the previous stellenbosch sensor system

  Serial.print("F1 415nm : ");
  Serial.println(counts[0]);
  Serial.print("F2 445nm : ");
  Serial.println(counts[1]);
  Serial.print("F3 480nm : ");
  Serial.println(counts[2]);
  Serial.print("F4 515nm : ");
  Serial.println(counts[3]);
  Serial.print("F5 555nm : ");
  // again, we skip the duplicates  
  Serial.println(counts[6]);
  Serial.print("F6 590nm : ");
  Serial.println(counts[7]);
  Serial.print("F7 630nm : ");
  Serial.println(counts[8]);
  Serial.print("F8 680nm : ");
  Serial.println(counts[9]);
  Serial.print("Clear    : ");
  Serial.println(counts[10]);
  Serial.print("NIR      : ");
  Serial.println(counts[11]);

  Serial.println();
  //delay(2);

  CreateJsonString();

}

void CreateJsonString()
{

  Serial.println("====================================================================================================================================");
  Serial.println("Begining of CreateJsonString function");
  Serial.println("====================================================================================================================================");

  //calculate the capacity of json document
  const int capa = JSON_ARRAY_SIZE(1) + 1 * JSON_OBJECT_SIZE(14);
  DynamicJsonDocument doc(capa);

  JsonObject root = doc.createNestedObject();

  root["F1_415nm"].set(counts[0]);
  root["F2_445nm"].set(counts[1]);

  root["F3_480nm"].set(counts[2]);
  root["F4_515nm"].set(counts[3]);

  root["F5_555nm"].set(counts[6]);
  root["F6_590nm"].set(counts[7]);

  root["F7_630nm"].set(counts[8]);
  root["F8_680nm"].set(counts[9]);

  root["Clear"].set(counts[10]);
  root["NIR"].set(counts[11]);

  root["Carbon_dioxide"].set(carbon);

  //Serial.println(F("Print serialized data to serial monitor... "));

  //format 1
  //serializeJsonPretty(doc, Serial); //output was once serial
  serializeJson(doc, output); //output was once serial

  //format 2 fails to print when there is too many passengers data
  //serializeJson(doc, Serial);
  Serial.println();
  Serial.println();
  Serial.println(F("Print the output variable... "));
  Serial.println();
  Serial.println(output);
  //Serial.println(F("=================================="));

  //clear document for reuse
  doc.clear();



  Serial.println("====================================================================================================================================");
  Serial.println("End of CreateJsonString function");
  Serial.println("====================================================================================================================================");

  Serial1.begin(9600);
  
    while (!Serial1) {
    delay(1);
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(1000);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(1000);   
  }

  int bytesSent = Serial1.write(output.c_str());

  //int bytesSent = Serial1.write("HELLO"); 

  if(bytesSent>0)
  {
    digitalWrite(LED_BUILTIN, LOW); 
    delay(100);
  }

  output = "\0";
  //carbon = 0;
  for(byte i =0; i<12; i++)
  {
    counts[i]=0;
  }
  return;
}

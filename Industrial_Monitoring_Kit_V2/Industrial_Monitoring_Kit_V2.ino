#include <iot_cmd.h>
#include "ThingSpeak.h"                                          //include thingspeak library
#include <sequencer4.h>                                          //imports a 4 function sequencer 
#include <sequencer1.h>                                          //imports a 1 function sequencer 
#include <GSMSimHTTP.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>

// If the LoRa Hel;tec Board is used need to define the LORA directive - sets to Wire1 not Wire for I2C
//#define LORA

#include <Ezo_i2c_util.h>                                        //brings in common print statements
#include <Ezo_i2c.h>                                             //include the EZO I2C library from https://github.com/Atlas-Scientific/Ezo_I2c_lib
#include <Wire.h>                                                //include arduinos i2c library


const long myChannelNumber = 1654631;                            //Your Thingspeak channel number
const char * myWriteAPIKey = "NLLWF4NNX3G94G3M";                 //Your ThingSpeak Write API Key
//------------------------------------------------------------------

// Initialise constants/variables for battery voltage measurement
const int battPin = 35;         // Pin number
float battMeasurement;          // Measured pin value
float voltage;                  // Calculated voltage value
float sensorTemp;               // Measured RTD temperature


Ezo_board PH = Ezo_board(99, "PH");       //create a PH circuit object, who's address is 99 and name is "PH"
Ezo_board EC = Ezo_board(100, "EC");      //create an EC circuit object who's address is 100 and name is "EC"
Ezo_board DO = Ezo_board(97, "DO");       //create an DO circuit object who's address is 97 and name is "DO"
Ezo_board RTD = Ezo_board(102, "RTD");    //create an RTD circuit object who's address is 102 and name is "RTD"
Ezo_board PMP = Ezo_board(103, "PMP");    //create an PMP circuit object who's address is 103 and name is "PMP"
Ezo_board ORP = Ezo_board(98, "ORP");    //create an PMP circuit object who's address is 98 and name is "OPR"

Ezo_board device_list[] = {               //an array of boards used for sending commands to all or specific boards
  PH,
  EC,
  RTD,
  DO,
  ORP,
  PMP
};

Ezo_board* default_board = &device_list[0]; //used to store the board were talking to

//gets the length of the array automatically so we dont have to change the number every time we add new boards
const uint8_t device_list_len = sizeof(device_list) / sizeof(device_list[0]);

//enable pins for each circuit
//const int EN_PH = 13;
//const int EN_EC = 12;
const int EN_PH = 15;
const int EN_EC = 14;
const int EN_RTD = 33;
const int EN_AUX = 27;

const unsigned long reading_delay = 1000;                 //how long we wait to receive a response, in milliseconds 
const unsigned long thingspeak_delay = 15000;             //how long we wait to send values to thingspeak, in milliseconds

unsigned int poll_delay = 2000 - reading_delay * 2 - 300; //how long to wait between polls after accounting for the times it takes to send readings

//parameters for setting the pump output
#define PUMP_BOARD        PMP       //the pump that will do the output (if theres more than one)
#define PUMP_DOSE         -0.5      //the dose that the pump will dispense
#define EZO_BOARD         EC        //the circuit that will be the target of comparison
#define IS_GREATER_THAN   true      //true means the circuit's reading has to be greater than the comparison value, false mean it has to be less than
#define COMPARISON_VALUE  1000      //the threshold above or below which the pump is activated

#define one_pump_ 14
#define two_pump_ 15

float k_val = 0;                                          //holds the k value for determining what to print in the help menu

bool polling  = true;                                     //variable to determine whether or not were polling the circuits
bool send_to_thingspeak = true;                           //variable to determine whether or not were sending data to thingspeak

void print_help();

//GSM SETTINGS
#define RESET_PIN 16 // you can use any pin.
void sim800_setup();
String Sim800Posting();
String sim800_top();
String post_status(String status_message);
void reconnect_GSM();

String APN_ = "internet";
String USER_ = " ";
String PWD_ = " ";
String _buffer; //for reading incoming gsm uart bytes 

String field_one = "";  //for setting the thingspeak fields
String field_two = "";
String field_three = "";
String field_four = "";
String field_five = "";
String field_SIX ="";

String f_415 = "";
String f_445 = "";
String f_480 = "";
String f_515 = "";
String f_555 = "";
String f_590 = "";
String f_630 = "";
String f_680 = "";
String f_clear = "";
String f_NIR = "";
String carbon_dioxide = "";

bool process_coms(const String &string_buffer);
const byte txPin = 12;
const byte rxPin = 13;

SoftwareSerial mySerial(rxPin, txPin);
GSMSimHTTP http(mySerial, RESET_PIN); 

bool kit_steps_complete;

//to arduino via uart
const byte rx_two = 0;
const byte tx_two = 2;

SoftwareSerial My_Arduino_Serial(rx_two, tx_two);

//lets handle the get request which has json data
String get_rx_bffr;
String get_rx_bffr_without_codes;
void pwm_fetch();
uint16 pump_one;
uint16 pump_two;
uint16 new_pump_one; //if the fetch has a new value
uint16 new_pump_two;

//wiring to arduino for the 2 sensors


void thingspeak_send(){
  if (send_to_thingspeak == true) {     //if we're datalogging
  //String respo_nse="";
  //respo_nse = sim800_top();
    //if((respo_nse != "ERROR:NO_IP")||(respo_nse != "ERROR:NO_IP_FETCH"))   
    //{
      if(Sim800Posting() == "ok")
      {
        Serial.println("sent to thingspeak");
      }
      else{
        Serial.println("couldnt send to thingspeak");
      }
    //}                                            
  }
}

void step1();      //forward declarations of functions to use them in the sequencer before defining them
void step2();
void step3();
void step4();
void step5();

Sequencer4 Seq(&step1, reading_delay,   //calls the steps in sequence with time in between them
               &step2, 300, 
               &step3, reading_delay,
               &step4, poll_delay);

Sequencer1 GSM_Seq(&reconnect_GSM, (1000*60*5));  //calls the Gsm reconnect function every 10 minutes. For now the top of setup gets called often too so there is no need to sequence this

//Sequencer1 GSM_recon(&sim800_setup, (1000*60*20)); //we want to reboot the GSM after every 20 minutes

Sequencer1 Thingspeak_seq(&thingspeak_send, thingspeak_delay); //sends data to thingspeak with the time determined by thingspeak delay

Sequencer1 PWM_get(&pwm_fetch, (1000*60*5)); //we want to poll for the latest pwm value for the pump every 5 minutes.

int incomingbyte=0;
String incomingstring="";
volatile bool start_reading;
void step6();

void setup() {

  pinMode(txPin, OUTPUT);
  pinMode(rxPin, INPUT);
  mySerial.begin(9600);
  start_reading=false;

  pinMode(one_pump_,OUTPUT);
  pinMode(two_pump_,OUTPUT);

  pinMode(tx_two, OUTPUT);
  pinMode(rx_two, INPUT);

  pinMode(EN_PH, OUTPUT);                                                         //set enable pins as outputs
  pinMode(EN_EC, OUTPUT);
  pinMode(EN_RTD, OUTPUT);
  pinMode(EN_AUX, OUTPUT);
  digitalWrite(EN_PH, LOW);                                                       //set enable pins to enable the circuits
  digitalWrite(EN_EC, LOW);
  digitalWrite(EN_RTD, HIGH);
  digitalWrite(EN_AUX, LOW);
  pump_one = 0; //the pump is always at 100% in the begning
  pump_two = 0;
  new_pump_one = 0; 
  new_pump_two = 0;

  analogWrite(one_pump_,pump_one);
  analogWrite(two_pump_,pump_two);

  kit_steps_complete = true;

  Wire.begin();                           //start the I2C
  Serial.begin(9600);                     //start the serial communication to the computer
  My_Arduino_Serial.begin(9600);
  //sim800_setup();
 
  //pwm_fetch();
  //GSM_Seq.reset();
  //Seq.reset();
  //Thingspeak_seq.reset();
  //GSM_recon.reset();
  //PWM_get.reset();
}

void loop() {

  step5();
  
  if(kit_steps_complete==true)
  {
    kit_steps_complete = false;
    step6();
  }
 String cmd;                            //variable to hold commands we send to the kit

  
  GSM_Seq.run();                        //run the sequncer to do the polling. for now, the sim top art which is called by this is called by things every 15 sec so ther is no need
  //GSM_recon.run();
  PWM_get.run();
  
  if (receive_command(cmd)) {            //if we sent the kit a command it gets put into the cmd variable
    polling = false;                     //we stop polling  
    send_to_thingspeak = false;          //and sending data to thingspeak
    if(!process_coms(cmd)){              //then we evaluate the cmd for kit specific commands
      process_command(cmd, device_list, device_list_len, default_board);    //then if its not kit specific, pass the cmd to the IOT command processing function
    }
  }
  
  if (polling == true) {                 //if polling is turned on, run the sequencer
    Seq.run();
    Thingspeak_seq.run();
  }

}

//function that controls the pumps activation and output
void pump_function(Ezo_board &pump, Ezo_board &sensor, float value, float dose, bool greater_than){
 if (sensor.get_error() == Ezo_board::SUCCESS) {                    //make sure we have a valid reading before we make any decisions
    bool comparison = false;                                        //variable for holding the reuslt of the comparison
    if(greater_than){                                               //we do different comparisons depending on what the user wants
      comparison = (sensor.get_last_received_reading() >= value);   //compare the reading of the circuit to the comparison value to determine whether we actiavte the pump
    }else{
      comparison = (sensor.get_last_received_reading() <= value);
    }
    if (comparison) {                                               //if the result of the comparison means we should activate the pump
      pump.send_cmd_with_num("d,", dose);                           //dispense the dose
      delay(100);                                                   //wait a few milliseconds before getting pump results
      Serial.print(pump.get_name());                                //get pump data to tell the user if the command was received successfully
      Serial.print(" ");
      char response[20]; 
      if(pump.receive_cmd(response, 20) == Ezo_board::SUCCESS){
        Serial.print("pump dispensed ");
      }else{
        Serial.print("pump error ");
      }
      Serial.println(response);
    }else {
      pump.send_cmd("x");                                          //if we're not supposed to dispense, stop the pump
    }
  }
}

void step1() {
  //send a read command. we use this command instead of RTD.send_cmd("R"); 
  //to let the library know to parse the reading
  Serial.print("\n*** Step 1 ***\n");
  RTD.send_read_cmd();
  
  // Battery monitoring related:
  battMeasurement = analogRead(battPin);                          // Read the current battery pin value
  voltage = battMeasurement * 0.001772893772894 - 0.246;          // Calculate battery voltage from measured pin value

  Serial.print("1: Battery Voltage : ");
  Serial.print(voltage);
  Serial.print("\n");
}

void step2() {
  Serial.print("\n*** Step 2 ***\n");

  // Let's do a temperature Reading
  receive_and_print_reading(RTD);             //get the reading from the RTD circuit and print it

  Serial.print("\n2: Print the RTD Error Code : ");
  Serial.print(RTD.get_error());
  Serial.print("\n");

  Serial.print("\n2: Print the Address of the device : ");
  Serial.print(RTD.get_address());
  Serial.print("\n");
  
  sensorTemp = RTD.get_last_received_reading();
  Serial.print("\n2: Temperature : ");
  Serial.print(String(sensorTemp));
  Serial.print("\n");

  if ((RTD.get_error() == Ezo_board::SUCCESS) && (RTD.get_last_received_reading() > -1000.0)) { //if the temperature reading has been received and it is valid
    PH.send_cmd_with_num("T,", RTD.get_last_received_reading());
    EC.send_cmd_with_num("T,", RTD.get_last_received_reading());
    DO.send_cmd_with_num("T,", RTD.get_last_received_reading());
    ORP.send_cmd_with_num("T,", RTD.get_last_received_reading());

    //ThingSpeak.setField(3, String(RTD.get_last_received_reading(), 2));                 // Assign temperature readings to the third column of thingspeak channel
    field_three="";
    field_three = "&field3=";
    field_three += String(RTD.get_last_received_reading(), 2);

    //ThingSpeak.setField(5, String(voltage, 2));                                         // Assign battery voltage reading to the fifth column of thingspeak channel
    Serial.print("\n2: Battery voltage: ");                                                  // Print battery voltage to serial
    Serial.print(voltage);
    field_five="";
    field_five = "&field5=";
    field_five += String(voltage, 2);

  } else {                                                                              // If the temperature reading is invalid
    PH.send_cmd_with_num("T,", 25.0);
    EC.send_cmd_with_num("T,", 25.0);                                                   // Send default temp = 25 deg C to EC sensor
    DO.send_cmd_with_num("T,", 20.0);
    ORP.send_cmd_with_num("T,", 20.0);
    
    //ThingSpeak.setField(3, String(21.0, 2));                                            // Assign temperature readings to the third column of thingspeak channel
    // field_three="";
    // field_three = "&field3=";
    // field_three += String(21.0, 2);
  }

  Serial.print(" ");
}

void step3() {
  Serial.print("\n***Step 3 ***\n");
  //send a read command. we use this command instead of PH.send_cmd("R");
  //to let the library know to parse the reading
  PH.send_read_cmd();
  EC.send_read_cmd();
  DO.send_read_cmd();
  ORP.send_read_cmd();
}

void step4() {
  Serial.print("\n*** Step 4 ***\n");
  receive_and_print_reading(PH);             //get the reading from the PH circuit
  if (PH.get_error() == Ezo_board::SUCCESS) {                                          //if the PH reading was successful (back in step 1)
     //ThingSpeak.setField(1, String(PH.get_last_received_reading(), 2));                 //assign PH readings to the first column of thingspeak channel
     field_one="";
     field_one = "&field1=";
     field_one += String(PH.get_last_received_reading(), 2);
  }
  Serial.print("  ");
  receive_and_print_reading(EC);             //get the reading from the EC circuit
  if (EC.get_error() == Ezo_board::SUCCESS) {                                          //if the EC reading was successful (back in step 1)
     //ThingSpeak.setField(2, String(EC.get_last_received_reading(), 0));                 //assign EC readings to the second column of thingspeak channel
     field_two="";
     field_two = "&field2=";
     field_two += String(EC.get_last_received_reading(), 0);
  }
  Serial.print("  ");
  receive_and_print_reading(DO);             //get the reading from the DO circuit
  if (DO.get_error() == Ezo_board::SUCCESS) {                                          //if the DO reading was successful (back in step 1)
     //ThingSpeak.setField(4, String(DO.get_last_received_reading(), 2));                 //assign DO readings to the fourth column of thingspeak channel
     field_four="";
     field_four = "&field4=";
     field_four += String(DO.get_last_received_reading(), 2);  
  }
  Serial.print("  ");
  receive_and_print_reading(ORP);             //get the reading from the EC circuit
  if (ORP.get_error() == Ezo_board::SUCCESS) {                                          //if the DO reading was successful (back in step 1)
     //ThingSpeak.setField(4, String(DO.get_last_received_reading(), 2));                 //assign DO readings to the fourth column of thingspeak channel
     field_SIX="";
     field_SIX = "&field6=";
     String res_ult = "";
     res_ult = String(ORP.get_last_received_reading(), 2);
     //Serial.println(res_ult);
     field_SIX += res_ult;  
  }

  Serial.println();
  pump_function(PUMP_BOARD, EZO_BOARD, COMPARISON_VALUE, PUMP_DOSE, IS_GREATER_THAN);
  
  kit_steps_complete = true;
}

void step5()
{
  //Serial.println("Receive data from arduino");
  while(My_Arduino_Serial.available()>0)
  {
    char b =My_Arduino_Serial.read();
    incomingstring+=b;
  }


}

void step6()
{
  Serial.println("I have received:");
  Serial.println(incomingstring);
  int16_t first_bracket = incomingstring.indexOf('[');
  int16_t closing_bracket = incomingstring.lastIndexOf(']');
  closing_bracket = closing_bracket + 1;
  //now search for an opening bracket given that we have a complete set of data somewhere
  String trim = incomingstring.substring(first_bracket,closing_bracket);
  Serial.println("The following is waht is left after triming");
  Serial.println(trim);
  int16_t first_data_bracket = trim.lastIndexOf('[');
  String pure_data = trim.substring(first_data_bracket);
  Serial.println("Pure data");
  Serial.println(pure_data);
  StaticJsonDocument<1024> doc;

  DeserializationError error = deserializeJson(doc, pure_data);

  //test if pasrsing is ok
  if (error)
  {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    incomingstring="";
    return;
  }

  else{
    Serial.println();
    Serial.println("============================================================");
    Serial.println("After deserilizing");
    Serial.println("============================================================");
    float counts[10];
    uint16_t carbon;

    counts[0] = doc[0]["F1_415nm"];
    f_415 = "";
    f_415 = "&field1=";
    f_415 += String(counts[0]);

    counts[1] = doc[0]["F2_445nm"];
    f_445 = "";
    f_445 = "&field2=";
    f_445 +=  String(counts[1]);

    counts[2] = doc[0]["F3_480nm"];
    f_480 = "";
    f_480 = "&field3=";
    f_480 +=  String(counts[2]);

    counts[3] = doc[0]["F4_515nm"];
    f_515 = "";
    f_515 = "&field4=";
    f_515 +=  String(counts[3]);

    counts[4] = doc[0]["F5_555nm"];
    f_555 = "";
    f_555 = "&field5=";
    f_555 +=  String(counts[4]);

    counts[5] = doc[0]["F6_590nm"];
    f_590 = "";
    f_590 = "&field1=";
    f_590 +=  String(counts[5]);

    counts[6] = doc[0]["F7_630nm"];
    f_630 = "";
    f_630 = "&field2=";
    f_630 +=  String(counts[6]);

    counts[7] = doc[0]["F8_680nm"];
    f_680 = "";
    f_680 = "&field3=";
    f_680 +=  String(counts[7]);

    counts[8] = doc[0]["Clear"];
    f_clear = "";
    f_clear = "&field4=";
    f_clear +=  String(counts[8]);

    counts[9] = doc[0]["NIR"];
    f_NIR = "";
    f_NIR = "&field5=";
    f_NIR +=  String(counts[9]);

    carbon = doc[0]["Carbon_dioxide"];
    carbon_dioxide = "";
    carbon_dioxide = "&field6=";
    carbon_dioxide +=  String(carbon);


    Serial.print("F1 415nm : ");
    Serial.println(counts[0]);
    Serial.print("F2 445nm : ");
    Serial.println(counts[1]);
    Serial.print("F3 480nm : ");
    Serial.println(counts[2]);
    Serial.print("F4 515nm : ");
    Serial.println(counts[3]);
    Serial.print("F5 555nm : ");
    Serial.println(counts[4]);
    Serial.print("F6 590nm : ");
    Serial.println(counts[5]);
    Serial.print("F7 630nm : ");
    Serial.println(counts[6]);
    Serial.print("F8 680nm : ");
    Serial.println(counts[7]);
    Serial.print("Clear    : ");
    Serial.println(counts[8]);
    Serial.print("NIR      : ");
    Serial.println(counts[9]);
    Serial.print("Carbon_dioxide: ");
    Serial.println(carbon);
    
    Serial.println();
    Serial.println("============================================================");
    Serial.println("After deserilizing");
    Serial.println("============================================================");
    Serial.println();
  }

  incomingstring="";
  
}

void start_datalogging() {
  polling = true;                                                 //set poll to true to start the polling loop
  send_to_thingspeak = true;
  Thingspeak_seq.reset();
}

bool process_coms(const String &string_buffer) {      //function to process commands that manipulate global variables and are specifc to certain kits
  if (string_buffer == "HELP") {
    print_help();
    return true;
  }
  else if (string_buffer.startsWith("DATALOG")) {
     start_datalogging();
    return true;
  }
  else if (string_buffer.startsWith("POLL")) {
    polling = true;  
    Seq.reset();
    
    int16_t index = string_buffer.indexOf(',');                    //check if were passing a polling delay parameter
    if (index != -1) {                                              //if there is a polling delay
      float new_delay = string_buffer.substring(index + 1).toFloat(); //turn it into a float

      float mintime = reading_delay*2 + 300;
      if (new_delay >= (mintime/1000.0)) {                                       //make sure its greater than our minimum time
        Seq.set_step4_time((new_delay * 1000.0) - mintime);          //convert to milliseconds and remove the reading delay from our wait
      } else {
        Serial.println("delay too short");                          //print an error if the polling time isnt valid
      }
    }
    return true;
  }
  return false;                         //return false if the command is not in the list, so we can scan the other list or pass it to the circuit
}

void get_ec_k_value(){                                    //function to query the value of the ec circuit
  char rx_buf[10];                                        //buffer to hold the string we receive from the circuit
  EC.send_cmd("k,?");                                     //query the k value
  delay(300);
  if(EC.receive_cmd(rx_buf, 10) == Ezo_board::SUCCESS){   //if the reading is successful
    k_val = String(rx_buf).substring(3).toFloat();        //parse the reading into a float
  }
}

void print_help() {
  get_ec_k_value();
  Serial.println(F("Atlas Scientific I2C hydroponics kit                                       "));
  Serial.println(F("Commands:                                                                  "));
  Serial.println(F("datalog      Takes readings of all sensors every 15 sec send to thingspeak "));
  Serial.println(F("             Entering any commands stops datalog mode.                     "));
  Serial.println(F("poll         Takes readings continuously of all sensors                    "));
  Serial.println(F("                                                                           "));
  Serial.println(F("ph:cal,mid,7     calibrate to pH 7                                         "));
  Serial.println(F("ph:cal,low,4     calibrate to pH 4                                         "));
  Serial.println(F("ph:cal,high,10   calibrate to pH 10                                        "));
  Serial.println(F("ph:cal,clear     clear calibration                                         "));
  Serial.println(F("                                                                           "));
  Serial.println(F("do:cal               calibrate DO probe to the air                         "));
  Serial.println(F("do:cal,0             calibrate DO probe to O dissolved oxygen              "));
  Serial.println(F("do:cal,clear         clear calibration                                     "));
  Serial.println(F("                                                                           "));
  Serial.println(F("ec:cal,dry           calibrate a dry EC probe                              "));
  Serial.println(F("ec:k,[n]             used to switch K values, standard probes values are 0.1, 1, and 10 "));
  Serial.println(F("ec:cal,clear         clear calibration                                     "));

  if(k_val > 9){
     Serial.println(F("For K10 probes, these are the recommended calibration values:            "));
     Serial.println(F("  ec:cal,low,12880     calibrate EC probe to 12,880us                    "));
     Serial.println(F("  ec:cal,high,150000   calibrate EC probe to 150,000us                   "));
  }
  else if(k_val > .9){
     Serial.println(F("For K1 probes, these are the recommended calibration values:             "));
     Serial.println(F("  ec:cal,low,12880     calibrate EC probe to 12,880us                    "));
     Serial.println(F("  ec:cal,high,80000    calibrate EC probe to 80,000us                    "));
  }
  else if(k_val > .09){
     Serial.println(F("For K0.1 probes, these are the recommended calibration values:           "));
     Serial.println(F("  ec:cal,low,84        calibrate EC probe to 84us                        "));
     Serial.println(F("  ec:cal,high,1413     calibrate EC probe to 1413us                      "));
  }
  
  Serial.println(F("                                                                           ")); 
  Serial.println(F("rtd:cal,t            calibrate the temp probe to any temp value            "));
  Serial.println(F("                     t= the temperature you have chosen                    "));
  Serial.println(F("rtd:cal,clear        clear calibration                                     "));
 }


String post_status(String status_message)
{
  int first_marker = status_message.indexOf('|');
  int second_marker = status_message.lastIndexOf('|');
  String status_code = status_message.substring(first_marker, (second_marker + 1));
  String Datalenghth = status_message.substring(second_marker + 1);
  String SuccessCode = "|HTTPCODE:200|";
  String WrongDataLength = "LENGTH:0";

  Serial.print("This is the extracted status code :: ");
  Serial.println(status_code);
  Serial.print("This is the extracted posted data length :: ");
  Serial.println(Datalenghth);

  //now we check if the status code is right
  if (strcmp((SuccessCode.c_str()), (status_code.c_str())) == 0) {
    Serial.println("The HTTP IS GOOD");
    return "ok";
  }
  else {
    Serial.println("The HTTP IS BAD");
    return "bad";
  }

}

String Sim800Posting()
{
  String PostResult = "";
  String to_send = "";
  to_send = "api.thingspeak.com/update?api_key=NLLWF4NNX3G94G3M";
  //Serial.println(F("Check if field and data has anything..."));
  //Serial.println(field_and_data);
  to_send += field_one;  //for setting the thingspeak fields

  to_send += field_two;

  to_send += field_three;

  to_send += field_four;

  to_send += field_five;
  
  to_send += field_SIX;

  Serial.print("1/3 Sending the following... ");
  Serial.println(to_send);

  PostResult = http.get(to_send);
  Serial.println(PostResult);
  
  //now we send to the first of the light and c02 channels
  PostResult = "";
  to_send = "";
  to_send = "api.thingspeak.com/update?api_key=81CKE6T3M2EZI7ZO";

  to_send += f_415;

  to_send +=  f_445;

  to_send +=  f_480;

  to_send +=  f_515;

  to_send +=  f_555;

  Serial.print("2/3 Sending the following... ");
  Serial.println(to_send);

  PostResult = http.get(to_send);
  Serial.println(PostResult);

  //now let us send to the last half of light and c02
  PostResult = "";
  to_send = "";
  to_send = "api.thingspeak.com/update?api_key=YRJB755GM76XQB46";

  to_send += f_590;

  to_send += f_630;

  to_send += f_680;

  to_send += f_clear;

  to_send += f_NIR;

  to_send += carbon_dioxide;

  Serial.print("3/3 Sending the following... ");
  Serial.println(to_send);

  PostResult = http.get(to_send);
  Serial.println(PostResult);

  return post_status(PostResult);

}

void reconnect_GSM()
{
  String resultss = "";
  resultss = sim800_top();
  //sim800_setup();
}

String sim800_top()
{
  String hold_IP="";
  Serial.println("sim800_top function to restart GPS");
  Serial.print(F("First close GPRS... "));
  Serial.println(http.closeConn());
  delay(1000);

  Serial.print(F("Now connect GPRS."));

  Serial.print(F("."));
  Serial.println(http.connect());
  delay(1000);

  Serial.println(F("GSM has already been initiated continue..."));
  Serial.print(F("Get IP Address... "));
  Serial.println(http.getIP());
  hold_IP = http.getIP();
  Serial.println(hold_IP);
  return hold_IP;
}

void sim800_setup()
{
  // Init module...
  //delay(1000);
  //http.reset(); 
  //http.init(); // use for init module. Use it if you dont have any valid reason.
  //http.gprsInit(APN_, USER_, PWD_);
  
  Serial.println(F("Setting up gsm to go into gprs mode once and for all:"));
  
  Serial.print(F("Network reboot off...")); //might wantto have many trials here
  Serial.println(http.setPhoneFunc(0));
  delay(1000);
  Serial.print(F("Network reboot on... ")); //might wantto have many trials here
  Serial.println(http.setPhoneFunc(1));
  delay(1000);

  Serial.print(F("is Module Registered to Network?... "));
  Serial.println(http.isRegistered());
  delay(1000);

  Serial.print(F("Signal Quality... "));
  Serial.println(http.signalQuality());
  delay(1000);

  Serial.print(F("Operator Name... "));
  Serial.println(http.operatorNameFromSim());
  delay(1000);

  Serial.print(F("First close GPRS... "));
  Serial.println(http.closeConn());
  delay(1000);


  Serial.print(F("Now connect GPRS."));
  
  Serial.print(F("."));
  Serial.println(http.connect());
  delay(500);


  Serial.println(F("gsm_int done!!"));


  Serial.println(F("GSM has already been initiated continue..."));
  Serial.print(F("Get IP Address... "));
  Serial.println(http.getIP());
  delay(1000);

  //return 0;
}

void pwm_fetch()
{
  Serial.println("THE PWM FETCH FUNCTION");
  //sim800_top();
  Serial.println("GET...");
  get_rx_bffr = http.get("http://api.thingspeak.com/channels/1659120/feeds.json?api_key=59088D47E1QH82QF",true);
  Serial.println(get_rx_bffr);
  Serial.println("=======================================");
  Serial.println();

  int json_begin = get_rx_bffr.indexOf('{');
  get_rx_bffr_without_codes = get_rx_bffr.substring(json_begin);
  Serial.println("The following is what I want to deserialize");
  Serial.println(get_rx_bffr_without_codes);
  
  Serial.println("=======================================");
  Serial.println();

  StaticJsonDocument<1024> doc;
  DeserializationError error = deserializeJson(doc, get_rx_bffr_without_codes);

  //test if pasrsing is ok
  if (error)
  {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  else {
    new_pump_one = doc["feeds"][0]["field1"];
    Serial.println();
    Serial.print("The current pump1 PWM value is: ");
    Serial.print(pump_one/10.24);
    Serial.println("%");    
    Serial.print("The pump1 fetched PWM value is: ");
    Serial.print(new_pump_one);
    Serial.println("%"); 
    Serial.println();   
    new_pump_one = new_pump_one*10.24;

    if(new_pump_one != pump_one)
    {
      Serial.println("Assign the new pump1 to the pin"); 
      if(new_pump_one > pump_one)
      {
        Serial.print("changing PWM ...");
        for(uint16 i=pump_one; i<=new_pump_one; i++){
          Serial.print(i);   
          Serial.print(" ");  
          analogWrite(one_pump_, i);
          delay(1);
        }
        Serial.println();  
      }
      else{
          for(uint16 i=new_pump_one; i>pump_one; i--){
            Serial.print(i);   
            Serial.print(" ");  
            analogWrite(one_pump_, i);
            delay(1);
        }
        Serial.println();  
      }
      pump_one = new_pump_one;
    }    
    new_pump_two = doc["feeds"][0]["field2"];
    Serial.println();
    Serial.print("The current pump2 PWM value is: ");
    Serial.print(new_pump_two/10.24);
    Serial.println("%");    
    Serial.print("The pump2 fetched PWM value is: ");
    Serial.print(new_pump_two);
    Serial.println("%"); 
    Serial.println();   
    new_pump_two = new_pump_two*10.24;

    if(pump_two != new_pump_two)
    {
      Serial.println("Assign the new pump2 to the pin"); 
      if(new_pump_two > pump_two)
      { 
        Serial.print("Chnaging PWM...");  
        for(uint16 i=pump_two; i<=new_pump_two; i++){
          Serial.print(i);   
          Serial.print(" ");            
          analogWrite(two_pump_, i);
          delay(1);
        }
        Serial.println();
      }
      else{
          for(uint16 i=new_pump_two; i>pump_two; i--){
            Serial.print(i);   
            Serial.print(" ");   
            analogWrite(two_pump_, i);
            delay(1);
        }
        Serial.println();
      }
      pump_two = new_pump_two;
    }
  }
  get_rx_bffr='\0';
  doc.clear();

  return;
}

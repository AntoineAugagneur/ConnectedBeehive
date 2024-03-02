/*
* Title:    LoRaWAN beehive
* Author:   Antoine AUGAGNEUR
+ Date:     November, 2023
* Sources:  https://github.com/SylvainMontagny/MKRWAN1310/blob/main/MKRWAN1310-OTAA/MKRWAN1310-OTAA.ino
*
*/

/**** Librairies *******************************************************************/
#include <MKRWAN.h>                   // LoRaWAN Library
#include <CayenneLPP.h>               // CAYENNE LPP
#include "ArduinoLowPower.h"          // Low Power Library
#include <pins_arduino.h>             // To ensure the code does not meet any issues with pin definition     
#include "DHT.h"                      // Lib for moisture-temperature sensor (DHT22)
#include <HX711_ADC.h>                // Lib for HX711 reading (HX711 & 4 load cells)

/**** LoRaWAN Setup ***************************************************************/
#define FRAME_DELAY       30
#define DATA_RATE         5
#define ADAPTIVE_DR       true
#define CONFIRMED         false
#define myPORT            1
#define CAYENNE_LPP       true
#define LOW_POWER         false
#define DEBUG
//#undef DEBUG
LoRaModem modem;
String appEui = "0000000000000000"; 
String appKey = "3E612760870FFBCFB931D185549CFF4C";
uint8_t dataToSend[20] = {0};         // Data to send
uint8_t dataReceived[20] = {0};       // Data received via Donwlink
uint32_t frameDelay = FRAME_DELAY * 1000;
uint8_t maxTryStartModule = 5;
uint8_t maxTryJoinRequest = 5;

/**** Cayenne LPP Setup ************************************************************/
CayenneLPP dataToSendCayenne(51);
CayenneLPP dataReceivedCayenne(51);
#define LPP_Channel_Battery       1
#define LPP_Channel_Temperature   2
#define LPP_Channel_Humidity      3
#define LPP_Channel_Weight        4

/**** Modules setup ***************************************************************/

// Battery voltage
int BatteryVoltPin = A1;    // A1 as analog input to get voltage from the bridge divider
float batt_voltage_adc;     // No conversion is done. The rough ADC value is sent (-float- because of LPP requirements)
float batt_voltage;
float batt_voltage_percent;

// Temperature & Humidity
#define DHTPIN 7
#define DHTTYPE DHT22       // DHT 22  (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE);
float humidity = 0.0;
float temperature = 0.0;

// Weight
#define HX711_DOUT_PIN  0
#define HX711_SCK_PIN   1
uint8_t hx711NbMeas = 30;
long hx711Temp = 0;
uint8_t hx711Period = 100;
uint8_t lastMeasToAverage = 10;
uint8_t maxTryStartHX = 5;
HX711_ADC LoadCell(HX711_DOUT_PIN,HX711_SCK_PIN);
long stabilisingtime = 2000;      // tare precision
float hx711Factor = -22101;       // calculated once
bool HXisOK = false;

// Reset
void(* resetFunc) (void) = 0;

/**** modules definition **********************************************************/

bool initLoRa(void);

bool JoinRequest(void);

void sendLoRa();

void dowlink_handler(String Received_Payload);

void getBatteryVoltage(void);

void getTemperature_Humidity(void);

void getWeight(void);

void enterLowPower(void);

void wait(void);

/**** Code ********************************************************************************************************/

/**** Setup *********************************************************************/

void setup() {
  
  /***** Serial Link Configuration ******/
  Serial.begin(115200);
  // while (!Serial);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("\r\n\r\n\r\n");
  Serial.println("########################################");
  Serial.println("######  Auga Connected Beehive    ######");
  Serial.println("########################################");

  /****** Configuration of Battery voltage module *****/
  batt_voltage_adc = 0.0;

  /****** Configuration of Temperature & humidity module *********/
  Serial.println("\n> DHT22 Configuration");
  dht.begin();

  /****** Configuration of Weight module **************/
  Serial.println("\n> HX711 Configuration");
  for (uint8_t i = 0; i<maxTryStartHX; i++){
    Serial.print("  Start - Try ");Serial.println(i);
    LoadCell.begin();
    LoadCell.start(stabilisingtime);
    if (LoadCell.getTareTimeoutFlag()) {
      Serial.println("  Failed. Verify wiring.");
      HXisOK = false;
    }
    else {
      LoadCell.setCalFactor(1.0); //  (float)
      Serial.println("  HX711 Configuration done.");
      HXisOK = true;
      break;
    }
  }

  if(HXisOK){
    while (!LoadCell.update());
    float c = hx711Factor; LoadCell.setCalFactor(c);
    Serial.print("  Calibration value: "); Serial.println(c);
  }
  else{
    Serial.println("  HX711 connection aborted. Continue anyway.");
  }


  /**** LoRaWAN Configuration *************************/
  Serial.println("\n> LoRaWAN Configuration");
  if(!initLoRa()){resetFunc();}
  Serial.println("  LoRaWAN Configuration done.");
 
}

/**** Loop *********************************************************************/
void loop() {

  Serial.println("\n> Device woke-up. ##########################################");

  /****** Call modules and update the LPP buffer *******/
  getBatteryVoltage();
  getTemperature_Humidity();
  if (HXisOK){getWeight();}

  /****** Send data via LoRaWAN ************************/
  sendLoRa();

  /****** LowPOWER for FRAME_DELAY ms ******************/
  if(LOW_POWER == true)   enterLowPower();
  else                    wait();

}


/**** Module definition **********************************************************/

bool initLoRa(void){

  // Init module
  for(uint8_t i = 0; i< maxTryStartModule; i++){
    Serial.print("  Start Module - Try ");Serial.println(i);
    if (!modem.begin(EU868)) {
      Serial.println("  Failed to start module");
      if (i>=maxTryStartModule-1){return false;}
    }
    else{
      Serial.println("  Module started.");
      Serial.print("  DevEUI: "); Serial.println(modem.deviceEUI());
      Serial.print("  AppEUI: "); Serial.println(appEui);
      Serial.print("  AppKey: "); Serial.println(appKey);
      break;
    }
  }

  // JoinRequest
  if(!JoinRequest()){return false;}

  // LoRaWAN settings
  modem.dataRate(DATA_RATE);
  modem.setADR(1);
  modem.setPort(myPORT);
  Serial.print("  Frame will be sent every ");Serial.print((frameDelay<7000)?7000:frameDelay);Serial.println("ms\r\n");  

  return true;
}

bool JoinRequest(void){

  for(uint8_t i = 0; i< maxTryJoinRequest; i++){
    Serial.print("  Join Request - Try ");Serial.println(i);
    if (!modem.joinOTAA(appEui, appKey)) {
      Serial.println("  Join Request failed.");
      digitalWrite(LED_BUILTIN, HIGH);
      delay(2000);
      digitalWrite(LED_BUILTIN, LOW);
      if (i>=maxTryJoinRequest-1){return false;}
    }
    else{
      Serial.println("  Join procedure completed.");
      for(uint8_t l=0;l<4;l++){
        digitalWrite(LED_BUILTIN, HIGH); delay(200);
        digitalWrite(LED_BUILTIN, LOW); delay(200);
      }
      break;
    }
  }

  return true;
}


void sendLoRa(){

  Serial.println("\n> sendLoRa");

  int status;
  int i = 0;

  /***** Send data *****/
  if(CONFIRMED)   Serial.print("  Uplink CONFIRMED on fPort ");
  else            Serial.print("  Uplink UNCONFIRMED on fPort ");
  Serial.println(myPORT);
  Serial.print("  Sending CAYENNE LPP: ");

  uint8_t size = dataToSendCayenne.getSize();
  uint8_t *payload = dataReceivedCayenne.getBuffer();

  for(uint8_t i = 0 ; i < size ; i++){
    Serial.print(payload[i],HEX);
    Serial.print(" ");
  }
  Serial.print(" / Size: "); Serial.println(size);

  modem.beginPacket();
  modem.write(dataToSendCayenne.getBuffer(),size);
  status = modem.endPacket(CONFIRMED);
  if (status < 0) {
    Serial.println("  Send Frame failed.");
  } else {
    Serial.println("  Frame sent. Waiting for Downlink...");
  } 
  delay(3000); // Wait for downlink

 
  /***** Receive data *****/
  String Downlink = "";
  if (!modem.available()) {
    Serial.println("  No data received.");
  }
  else {
    while (modem.available()) {
      dataReceived[i++] = (char)modem.read();
    }
    Serial.print("  Received Downlink (Hexa) : ");
    for (int j = 0; j < i; j++) {
      Downlink += String(dataReceived[j] >> 4, HEX);
      Downlink += String(dataReceived[j] & 0xF, HEX);
      
    }  
    Serial.println(Downlink);

    // Handle the downlink
    dowlink_handler(Downlink);

  } 
  dataToSendCayenne.reset();
}


void dowlink_handler(String Received_Payload){
// frame format 0x[1-byte command ID][x-bytes payload] // Payload size depends on CID
// frequency setup: 01 xx...x

  Serial.println("\n> Downlink handler");
  Serial.print("  Payload : ");
  Serial.println(Received_Payload);

  String CID = Received_Payload.substring(0,2);
  Serial.print("  CID: "); Serial.print(CID);

  String Downlink_Ack = "";

  if (CID == "01"){ // Modif of uplink frequency, payload size = 2 bytes
    Serial.println(" / Sending Period modification)");
    String P_hex = Received_Payload.substring(2,6);
    long P_int = strtol(P_hex.c_str(),NULL,16);
    frameDelay = (int)P_int * 1000;
    Serial.print("  --> HEX: ");
    Serial.print(P_hex);
    Serial.print(" / INT: ");
    Serial.print(P_int);
    Serial.print(" / frameDelay (ms): ");
    Serial.println(frameDelay);

  }
  else if (CID == "02"){
    Serial.println(" / Software reset");
    String P_hex = Received_Payload.substring(2,6);
    long P_int = strtol(P_hex.c_str(),NULL,16);
    Serial.print("  --> HEX: ");
    Serial.print(P_hex);
    if (P_hex == "0001"){
      Serial.println("Reset..");
      resetFunc();
    }
  }
  else if (CID == "03"){
    Serial.println(" / init LoRa");
    String P_hex = Received_Payload.substring(2,6);
    long P_int = strtol(P_hex.c_str(),NULL,16);
    Serial.print("  --> HEX: ");
    Serial.println(P_hex);
    if (P_hex == "0001"){
      if(!initLoRa()){
        Serial.println("Reset..");
        resetFunc();
      }
    }
  }
  else{
    Serial.println(" (CID unknown. exit.)");
  }

}


void getBatteryVoltage(void){
  Serial.println("\n> getBatteryVoltage");
  batt_voltage_adc = 0.0;
  batt_voltage_adc = float(analogRead(BatteryVoltPin));
  batt_voltage = 2 * batt_voltage_adc * 3.3 / 1023;
  batt_voltage_percent = batt_voltage * 100 / 3.7;

  dataToSendCayenne.addAnalogOutput(LPP_Channel_Battery, batt_voltage_percent);

  Serial.print("  VBatt: ");Serial.print(batt_voltage);
  Serial.print(" V / PercentBatt: ");Serial.print(batt_voltage_percent);Serial.println(" %");
  Serial.print("  PercentBatt added on channel "); Serial.println(LPP_Channel_Battery);
}

void  getTemperature_Humidity(void){

  Serial.println("\n> getTemperature_Humidity");

  humidity = dht.readHumidity();        // in %
  temperature = dht.readTemperature();  // in °C
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println(F("  Failed to read from DHT sensor!"));
    Serial.println(F("  Sending 0.0 by default"));
    humidity = 0.0;
    temperature = 0.0;
  }
  else {
    Serial.print("  Temperature: ");Serial.print(temperature);
    Serial.print(" °C / Humidity: "); Serial.print(humidity); Serial.println(" %");
  }
  dataToSendCayenne.addTemperature(LPP_Channel_Temperature, temperature);
  dataToSendCayenne.addRelativeHumidity(LPP_Channel_Humidity, humidity);
  Serial.print("  Temperature added on channel "); Serial.println(LPP_Channel_Temperature);
  Serial.print("  Humidity added on channel "); Serial.println(LPP_Channel_Humidity);
}

void getWeight(void){

  Serial.println("\n> getWeight");

  LoadCell.begin();

  uint8_t hx711Itr = 0;
  float weight = 0.0;
  float weightBuffer = 0.0;

  while(hx711Itr<hx711NbMeas){

    LoadCell.update();

    if (millis() > hx711Temp + hx711Period) {
      weightBuffer = LoadCell.getData();

      if (hx711Itr >= hx711NbMeas-lastMeasToAverage){
        weight += weightBuffer;
      }
      Serial.print("  Weight: "); Serial.print(weightBuffer); Serial.println(" Kg");
      hx711Temp = millis();
      hx711Itr++;
    }

  }

  weight = weight/(lastMeasToAverage);
  dataToSendCayenne.addAnalogOutput(LPP_Channel_Weight, weight);
  Serial.print("  Weight average (");Serial.print(hx711NbMeas);Serial.print(" tries and ");Serial.print(lastMeasToAverage);Serial.print(" measures): ");
  Serial.print(weight); Serial.println(" Kg");
  Serial.print("  Weight average added on channel "); Serial.println(LPP_Channel_Weight);

}

void enterLowPower(void){
  Serial.print("\n> Processor goes in Low Power mode during : ");
  Serial.print(frameDelay);Serial.println("ms\r\n");
  LowPower.deepSleep(frameDelay);  
}

void wait(void){
  Serial.print("\n> Processor is going to wait during : ");
  Serial.print(frameDelay);Serial.println("ms\r\n");
  delay(frameDelay); 
}

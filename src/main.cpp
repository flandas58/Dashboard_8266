#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
//#include <DNSServer.h>
#include <ArduinoJson.h>
#include <Adafruit_MCP3008.h>
#include "FS.h"

//#include <JeVe_EasyOTA.h>  // https://github.com/jeroenvermeulen/JeVe_EasyOTA/blob/master/JeVe_EasyOTA.h
#define DEBUGDASHBOARD
#ifdef DEBUGDASHBOARD
  #include "RemoteDebug.h"   // https://github.com/JoaoLopesF/RemoteDebug
#endif


//******************************************* DEFINITIONS *********************************************//
#define DBG_OUTPUT_PORT Serial
#define HOST_NAME "Dashboard-debug"

#define TYRE_CIRCUMFERENCE 195 // distance car travels in one wheel rotation, measured in centimeters

// ***************************************** PIN Defines **********************************************//

//digital inputs  GPIO 0 -4 //
// NOTE: GPIO 13-15 (D5-D8) are reserved for SPI communication with MCP3008 ADC
// voltage uses simple divider circuit

#define HIGHBEAM_PIN D0// D0 cannont be used as an interrupt pin but can be used as a GPIO pin
#define LEFTINDICATOR_PIN D1 // GPIO0 (D3)
#define RIGHTINDICATOR_PIN D2 // GPIO5 (D1)
// so now we can use D3 and D4 for interrupts
#define TACHO_PIN     D3 // GPI O2 (D4) Input - Tachometer signal thru schmitt trigger optocoupler.
#define SPEEDO_PIN  D4 //  GPIO4 (D2) physical 10 D6, logical 10
// Analogue (ADC) inputs
#define VOLTS_PIN 0
#define FUEL_PIN 1
#define OILPRESSURE_PIN 2
#define OILTEMPERATURE_PIN 3

// other definitions used to calculate various Values
#define TACHFILTERSIZE  10// Size of the moving average filter for RPM.
#define SPEEDOFILTERSIZE 5
#define OILPRESSURESIZE 5
#define OILTEMPERATURESIZE 5
#define VOLTAGESIZE 10
#define FUELSIZE 20 // has to be large to allow for slopping in the tank

#define systemLedPin 13

//******************************************* GLOBALS *********************************************//
//************************************* VARIABLES AND CONSTANTS ***************************************//

const char* ssid = "VWDash";
const char* password = "IGiveUp24318";
// OTA values
char* WIFI_SSID = "Gannymede";
char* WIFI_PASSWORD = "IGiveUp24318";
char* ARDUINO_HOSTNAME = "vw_dashboard";

#ifdef DEBUGDASHBOARD
  RemoteDebug Debug;
#endif
//EasyOTA OTA;
Adafruit_MCP3008 adc;
ESP8266WebServer server(80);

// tachometer calculation values
volatile long fTachCounter = 0;
float fTachFrequencyArray[TACHFILTERSIZE];  // moving average filter for the RPM
volatile long fElapsedTime;
int fPreviousTime=millis();
float fThisRPM = 0;
// speedo
volatile int fLastTimeAround=millis();
volatile float fSpeedoKPHArray[SPEEDOFILTERSIZE];
long fFilterSum = 0;
volatile float fThisKph;
volatile long fDataCount = 0;

// moving averages
float fOilPressureArray[OILPRESSURESIZE];
float fOilTemperatureArray[OILTEMPERATURESIZE];
float fVoltageArray[VOLTAGESIZE];
float fFuelTankArray[FUELSIZE];

//************************************* END VARIABLES AND CONSTANTS ***************************************//

// ****************************************** METHODS ***************************************//
// ************************************** General methods ***********************************//
void listSPIFFS(){
  #ifdef DEBUGDASHBOARD
    rdebugIln("Listing SPIFFS contents");
  #endif
  Dir dir = SPIFFS.openDir("");
  while (dir.next()){
    DBG_OUTPUT_PORT.print(dir.fileName());
    DBG_OUTPUT_PORT.print(": \t");
    DBG_OUTPUT_PORT.println(dir.fileSize());
  }
}

float movingAverage(float pTarget[], int pArraySize, float pNewValue){
  float fArraySum = 0;
  for ( int i = 0; i < pArraySize - 1 ; i++ ) {
    pTarget[i] = pTarget[i + 1];
    fArraySum += pTarget[i]; // adding the
  }
  // add the newest reading to the end of the array.
  pTarget[pArraySize - 1] = pNewValue;
  fArraySum+=pNewValue;

  return fArraySum/pArraySize;
}


// *************************************** Tacho methods ************************************//
  // Tach signal input triggers this interrupt vector on the falling edge of pin D3 //
  void tacho_isr(){
    fTachCounter++;
  }
  // calculate the engine speed using a moving average
  float getRpm(){
    fThisRPM = (fTachCounter/(millis() - fPreviousTime)) * 60000;
                 //saves the current time
    fTachCounter = 0;
    fPreviousTime=millis();
    return movingAverage(fTachFrequencyArray, TACHFILTERSIZE, fThisRPM)  * 15 ; //60/4 ppr=15
  }
// ***************************************** Speedo methods ***************************************//
// speedo signal input triggers this interrupt vector on the rising edge of pin D4 //
void speedo_isr(){
  fElapsedTime = millis() - fLastTimeAround;
  fThisKph = (TYRE_CIRCUMFERENCE/100000)/(fElapsedTime/3600000 );

  for ( int i = 0; i < SPEEDOFILTERSIZE - 1 ; i++ ) {
    fSpeedoKPHArray[i] = fSpeedoKPHArray[i + 1];
  }
  // add the newest reading to the end of the array.
  fSpeedoKPHArray[SPEEDOFILTERSIZE - 1] = fThisKph; // Because the ISR is called @ counterFreq Hz
  fLastTimeAround = millis();
}

float getSpeedOverDistance(){
  float fKphSum = 0;
  for ( int i = 0; i < SPEEDOFILTERSIZE - 1 ; i++ ) {
    fKphSum+=fSpeedoKPHArray[i];
  }
  return fKphSum/SPEEDOFILTERSIZE;
}
// ******************************************* Other methods **************************************//

float getRotation(float reading, int startDegrees, int endDegrees, int startValue, int endValue){
    #ifdef DEBUGDASHBOARD
      rdebugDln("Reading %.2f StartDegrees %i, endDegrees %i startValue %i endValue %i ", reading, startDegrees, endDegrees, startValue, endValue);
    #endif
    if(reading<=startValue)
    {
      #ifdef DEBUGDASHBOARD
        rdebugDln("Using StartDegrees");
      #endif
      return startDegrees;
    }
    else if (reading>=endValue) {
      #ifdef DEBUGDASHBOARD
        rdebugDln("using endDegrees");
      #endif
      return endDegrees;
    }
    else
    {
      return round( startDegrees + ((reading - startValue) * ((endDegrees - startDegrees)/(endValue - startValue))));
    }

}

float mapFloat(long x, long in_min, long in_max, float out_min, float out_max)
{
  float returnValue = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  return  returnValue;
}

//******************************************** getData() *******************************************//
void getData() {
  // NOTE: **************** this is called frequently so keep it lean ******************************//
  // Tachometer values
  fDataCount++;
  #ifdef DEBUGDASHBOARD
    rdebugD("getData() called from web page iteration %6ld \t", fDataCount);
  #endif
  detachInterrupt(digitalPinToInterrupt(TACHO_PIN));           //detaches the interrupt while we update the values
  float fRPM=getRpm();
  attachInterrupt(digitalPinToInterrupt(TACHO_PIN),tacho_isr,RISING); // re-attaches the interupt coil goes low on each pulse but input is inverted by the H11L1M
  // Speedo values
  detachInterrupt(digitalPinToInterrupt(SPEEDO_PIN));
  float fKph = getSpeedOverDistance();
  attachInterrupt(digitalPinToInterrupt(SPEEDO_PIN),speedo_isr,FALLING);
  //This is a JSON formatted string that will be served. You can change the values to whatever like.
  // {"data":[{"dataValue":"1024"},{"dataValue":"23"}]} This is essentially what is will output you can add more if you like
  #ifdef DEBUGDASHBOARD
    rdebugIln("Reading values from mcp3008 ADC");
    rdebugI("ADC from pin %i is %i \t",VOLTS_PIN,adc.readADC(VOLTS_PIN) );
  #endif
  float fTrueVoltage = movingAverage(fVoltageArray, VOLTAGESIZE, mapFloat(adc.readADC(VOLTS_PIN), 0, 1023, 0, 3.3)/ 0.1072);
  #ifdef DEBUGDASHBOARD
    rdebugI("voltage is %.2f \t", fTrueVoltage);
  #endif
  float fFuelReading = movingAverage(fFuelTankArray, FUELSIZE, mapFloat(adc.readADC(FUEL_PIN), 0, 1023, 0, 100));
  #ifdef DEBUGDASHBOARD
    rdebugDln("Fuel from pin %i voltage %f",FUEL_PIN,fFuelReading);
  #endif
  float fOilPressureReading = movingAverage(fOilPressureArray, OILPRESSURESIZE, mapFloat(adc.readADC(OILPRESSURE_PIN),0,1023,0,100)) ;
  #ifdef DEBUGDASHBOARD
    rdebugDln("Oil Pressure from pin %i voltage %f",OILPRESSURE_PIN,fOilPressureReading);
  #endif
  float fOilTempReading = movingAverage(fOilTemperatureArray, OILTEMPERATURESIZE, mapFloat(adc.readADC(OILTEMPERATURE_PIN),0,1023,0,160));
  #ifdef DEBUGDASHBOARD
    rdebugDln("Oil Temp from pin %i voltage %f",OILTEMPERATURE_PIN,fOilTempReading);
    rdebugDln("Digital RPM %f",fRPM);
    rdebugDln("Speed over distance %f",fKph);
    rdebugIln("Creating JSON object");
  #endif
  StaticJsonBuffer<2048> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();

  root["speed"] = getRotation(fKph,-135,135,0,180);
  root["rpm"] = getRotation(fRPM,-135,105,0,8000);

  root["fuel"] = getRotation(fFuelReading,-45,45,0,100);
  root["volts"] = getRotation(fTrueVoltage,-45,45,8,16);
  #ifdef DEBUGDASHBOARD
    rdebugIln("rotation is %.0f degrees", getRotation(fTrueVoltage,-45,45,8,16) );
  #endif
  root["pressure"] = getRotation(fOilPressureReading,-45,45,0,80);
  root["temp"] = getRotation(fOilTempReading,-45,45,60,160);
  root["leftIndicator"] = !digitalRead(LEFTINDICATOR_PIN);
  root["rightIndicator"] = !digitalRead(RIGHTINDICATOR_PIN);
  root["highBeamWarning"] = !digitalRead(HIGHBEAM_PIN);

  root["lowFuelWarning"] = fFuelReading<10;
  root["lowOilWarning"]= fOilPressureReading < 30;
  root["lowBatteryWarning"] = fTrueVoltage < 12;
  root["hotOilWarning"] = fOilTempReading > 200;

  // Just for testing
  int fMilesCount = fDataCount / 10;
  if (fMilesCount/10 > 9999999){
    fDataCount = 0;
    fMilesCount = 0;
  }
  // odoMiles has preceeding zeros


  String sOdoMiles = "";
  sOdoMiles+=(fMilesCount / 10);
  while(sOdoMiles.length() < 7){
    sOdoMiles = "0" + sOdoMiles;
  }
  root["odoMiles"] = sOdoMiles;
  root["odoTenths"] = fMilesCount % 10;
  //print the resulting JSON to a String
  String output;
  root.printTo(output);
  server.send(200, "text/html", output);
  #ifdef DEBUGDASHBOARD
    rdebugDln("returning JSON values for iteration %6ld ", fDataCount);
  #endif
}

bool createWifiAP() {
  DBG_OUTPUT_PORT.print("Using ssid: ");
  DBG_OUTPUT_PORT.println(ssid);
  DBG_OUTPUT_PORT.print("Using password: ");
  DBG_OUTPUT_PORT.println(password);

  WiFi.softAP(ssid, password);

  IPAddress myIP = WiFi.softAPIP();

  DBG_OUTPUT_PORT.print("AP IP address: ");
  DBG_OUTPUT_PORT.println(myIP);
  DBG_OUTPUT_PORT.print("WiFi Hostname: ");
  DBG_OUTPUT_PORT.println(WiFi.hostname());

  String hostNameWifi = HOST_NAME;
  hostNameWifi.concat(".local");
  WiFi.hostname(hostNameWifi);
  if ( MDNS.begin(HOST_NAME ) ) {
    DBG_OUTPUT_PORT.print("* MDNS responder started. Hostname -> ");
    DBG_OUTPUT_PORT.println(HOST_NAME);
    MDNS.addService("telnet", "tcp", 23);
  }

  // handlers called from browser to access files in SPIFFS file system on 8266
  server.serveStatic("/", SPIFFS, "/dashboard.html");
  server.serveStatic("/index.html", SPIFFS, "/dashboard.html");
  server.serveStatic("/dashboard.svg", SPIFFS, "/dashboard.svg");

  server.on("/data", getData);

  server.begin();
  DBG_OUTPUT_PORT.println("HTTP server started");

  return true;
}

void setup() {
  DBG_OUTPUT_PORT.begin(115200);
  DBG_OUTPUT_PORT.setDebugOutput(true);
  DBG_OUTPUT_PORT.print("\n");
  if (!SPIFFS.begin()) {
    DBG_OUTPUT_PORT.println("Failed to mount file system");
    return;
  }
  if (!createWifiAP()) {
    DBG_OUTPUT_PORT.println("Failed to create WiFi AP");
    return;
  }
  // initialise debugging over telnet
  #ifdef DEBUGDASHBOARD
    Debug.begin(WiFi.hostname()); // Initiaze the telnet server
    Debug.setResetCmdEnabled(true); // Enable the reset command

    Debug.showProfiler(true); // Profiler
  	Debug.showColors(true); // Colors
    Debug.setSerialEnabled(true);
  #endif
  // This callback will be called when JeVe_EasyOTA has anything to tell you.
  //OTA.onMessage([](char *message, int line) {
  //  Serial.println(message);
  //});
  #ifdef DEBUGDASHBOARD
    rdebugIln("Setting up OTA");
  #endif
  //OTA.setup(WIFI_SSID, WIFI_PASSWORD, ARDUINO_HOSTNAME);
  // Define the input pins // remember the inputs through the H11L1Ms are inverted
  #ifdef DEBUGDASHBOARD
    rdebugIln("configuring input pins");
  #endif
  pinMode( HIGHBEAM_PIN, INPUT_PULLUP);
  pinMode( LEFTINDICATOR_PIN, INPUT_PULLUP);
  pinMode( RIGHTINDICATOR_PIN, INPUT_PULLUP);
  // initialise the Interrupt handling on the input pins
  #ifdef DEBUGDASHBOARD
    rdebugIln("Setting up interrupts for Tacho and Speedo");
  #endif
  fTachCounter = 0;
  attachInterrupt(digitalPinToInterrupt(TACHO_PIN),tacho_isr,RISING);
  attachInterrupt(digitalPinToInterrupt(SPEEDO_PIN),speedo_isr,FALLING);
  // start the mcp3008 ADC
  #ifdef DEBUGDASHBOARD
    rdebugIln("initialise ADC");
  #endif
  adc.begin();
  fDataCount = 0;
  #ifdef DEBUGDASHBOARD
    rdebugIln("end of setup");
  #endif
}

void loop() {
  server.handleClient();
  // Over-the-air deployment
//  OTA.loop();
  #ifdef DEBUGDASHBOARD
    // Remote debug over telnet
    Debug.handle();
  #endif
  yield();
}

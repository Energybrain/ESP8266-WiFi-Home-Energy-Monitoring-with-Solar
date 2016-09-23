/*
  Original calculatePower() function is from calcVI() which is part of 
  Emon.cpp - Library for openenergymonitor.org
  Created by Trystan Lea, April 27 2010
  GNU GPL
  modified to use up to 12 bits ADC resolution (ex. Arduino Due)
  by boredman@boredomprojects.net 26.12.2013
  Low Pass filter for offset removal replaces HP filter 1/1/2015 - RW
  July 2016 - Modified by Bobby Chung to work with the ESP8266 module.

  Original ESP8266 code from
  Copyright (c) 2015 Chris Howell at openevse.com
  August 2016 - Modified by Bobby Chung to support the following
  - home energy monitoring hardware consisting of Adafruit's Huzzah ESP8266 module and ADS1015 a/d modules
  - resistor voltage divider network
  - 2 current tranformers that measure each split phase currents
  - 1 current transformer that measure the solar generation current
  - removed the OpenEVSE specific items
  
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
*/
extern "C"{
#include "user_interface.h"
}
#include "Adafruit_ADS1015_modified.h"
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h> 
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <ArduinoOTA.h>

#define VERSION "1.0"

// for EEPROM map
#define EEPROM_SIZE 512
#define SSID_START 0
#define SSID_MAX_LENGTH 32
#define PASS_START 32
#define PASS_MAX_LENGTH 64
#define KEY_START 96
#define KEY_MAX_LENGTH 50
#define KEY2_START 146
#define KEY2_MAX_LENGTH 50
#define NODE_START 196
#define NODE_MAX_LENGTH 1
#define HOST_START 197
#define HOST_MAX_LENGTH 32
#define HOST2_START 229
#define HOST2_MAX_LENGTH 32
#define DIRECTORY_START 261
#define DIRECTORY_MAX_LENGTH 32
#define DIRECTORY2_START 293
#define DIRECTORY2_MAX_LENGTH 32

#define SERVER_UPDATE_RATE 10000 //update emoncms server ever 10 seconds

ESP8266WebServer server(80);
Adafruit_ADS1015 ads1015;                  //Construct an ads1015 at the default address: 0x48 - used to measure reference voltage
Adafruit_ADS1015 ads1015b(0x4a);           //Construct a second ads1015 at the address: 0x4a - used to measure main current draw

int16_t sampleV,                           //Raw differential voltage reference sample
        sampleIi,                          //Raw single-ended solar inverter sample
        sampleIm,                          //Raw differential main panel current sample
        startV,                            //First raw differential voltage reference sample of the series
        previousSampleV,                   //Previous raw differntial voltage reference sample    
        adjustedSampleIi;                  //Adjusted value of solar inverter sample after subtracting DC offset
double Irms, 
       Vrms = 124,
       prevVrms,
       Irmsi, 
       Irmsm,
       phaseShiftedV;                       //Holds the calibrated phase shifted voltage.
unsigned int numberOfSamples,
       reportedCount;       
double sqV,
       sumV,                            
       sqIm,
       sumIm,
       sqIi,
       sumIi,
       instPi,
       sumPi,
       instPm,
       sumPm;                                //sq = squared, sum = Sum, inst = instantaneous
double realPoweri,
       apparentPoweri,
       powerFactori,
       realPowerm,
       apparentPowerm,
       powerFactorm;
boolean lastVCross,
        checkVCross;                         //Used to measure number of times threshold is crossed.

//Default SSID and PASSWORD for AP Access Point Mode
const char *ssid = "CurrentSensor";
//const char *ssid = "test";
const char *password = "currentsensor";
String st = "not_scanned";
String privateKey = "";
String privateKey2 = "";
String node = "0";
String esid = "";
String epass = "";

//SERVER strings and interface for emoncms
String host = ""; 
String host2 =  ""; 
String directory = ""; 
String directory2 = ""; 
const char *e_url = "input/post.json?node=";
const char* inputID_AMP_C   = "AMP_consumed:";
const char* inputID_AMP_G   = "AMP_generated:";
const char* inputID_VOLT   = "V_house:";
const char* inputID_RealPowerM = "Real_powm:";
const char* inputID_AppPowerM = "Apparent_powm:";
const char* inputID_PowerFactorM = "Power_factorm:";
const char* inputID_RealPowerI = "Real_powi:";
const char* inputID_AppPowerI = "Apparent_powi:";
const char* inputID_PowerFactorI = "Power_factori:";

int wifi_mode = 0;    // 0 - connected, 1 - SSID and no connection, 2 - no SSID and no connection  
int buttonState = 0;
int clientTimeout = 0;
int i = 0;
unsigned long Timer;
int current = 0;
unsigned long starttime, endtime, cycleTime;

void bootOTA() {
  Serial.println("<BOOT> OTA Update");
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  // ArduinoOTA.setPassword((const char *)"123");

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

String readEEPROM(int start_byte, int allocated_size) {
  String variable;
  for (int i = start_byte; i < (start_byte + allocated_size); ++i) {
    variable += char(EEPROM.read(i));
  }
  delay(10);
  return variable;
}

void writeEEPROM(int start_byte, int allocated_size, String contents) {
  int length_of_contents = contents.length();
  if (length_of_contents > allocated_size)
    length_of_contents = allocated_size;
  for (int i = 0; i < length_of_contents; ++i) {
    EEPROM.write(start_byte + i, contents[i]);
  }
  for (int i = (start_byte + length_of_contents); i < (start_byte + allocated_size); ++i)
    EEPROM.write(i,0);
  EEPROM.commit();
  delay(10);
}

void resetEEPROM(int start_byte, int end_byte) {
  Serial.println("Erasing EEPROM");
  for (int i = start_byte; i < end_byte; ++i) {
   EEPROM.write(i, 0);
    //Serial.print("#"); 
  }
  EEPROM.commit();   
}

void handleRoot() {
  String s;
  String sTmp;
  //Serial.println("inside root");
  delay(100);
  s = "<HTML><FONT SIZE=6><B>Current</B>Sensor<FONT FACE='Arial'>";
  s += " WiFi Configuration</FONT></FONT><FONT FACE='Arial'>";
  s += "<P>WiFi FW v";
  s += VERSION;
  s += "</P>";
  s += "<P>====================</P>";
  s += "<P><B>NETWORK CONNECT</B></P>";
  s += "<FORM ACTION='rescan'>";  
  s += "<INPUT TYPE=SUBMIT VALUE='     Rescan     '>";
  s += "</FORM>";
  s += "<FORM method='get' action='a'>";
  if (wifi_mode == 0)
    s += "<B><I>Connected to </B></I>";
  else
    s += "<B><I>Please choose a network </B></I>";
  s += st;
  s += "<P><LABEL><B><I>&nbsp;&nbsp;&nbsp;&nbsp;or enter SSID manually:</B></I></LABEL><INPUT NAME='hssid' MAXLENGTH='32' VALUE='empty'></P>";
  s += "<P><LABEL><B><I>Password:</B></I></LABEL><INPUT TYPE='password' SIZE = '25' NAME='pass' MAXLENGTH='32' VALUE='";
  sTmp = "";
  for (int i = 0; i < epass.length(); ++i) {    
      if (epass[i] == '\'')        //this is to allow user to use ' as part of the input and have it displayed
        sTmp += "&#39;";
      else
        sTmp += epass[i];
    }
  s += sTmp.c_str();  //need to use constant string to filter out unnecessary padding of control characters when reading from memory
  s += "'></P>";
  s += "<P>====================</P>";
  s += "<P><B>DATABASE SERVER</B></P>";
  s += "<P>__________</P>";
  s += "<P><B><I>Primary Server</B></I></P>";
  s += "<P><LABEL><I>Write key (devicekey=1..32):</I></LABEL><INPUT NAME='ekey' MAXLENGTH='50' VALUE='";
  sTmp = "";
  for (int i = 0; i < privateKey.length(); ++i) {    
      if (privateKey[i] == '\'')
        sTmp += "&#39;";
      else
        sTmp += privateKey[i];
    }
  s += sTmp.c_str();
  s += "'></P>";
  s += "<P><LABEL><I>Server address (example.com):</I></LABEL><INPUT NAME='host' MAXLENGTH='32' VALUE='";
  sTmp = "";
  for (int i = 0; i < host.length(); ++i) {    
      if (host[i] == '\'')
        sTmp += "&#39;";
      else
        sTmp += host[i];
    }
  s += sTmp.c_str();
  s += "'></P>";
  s += "<P><LABEL><I>Database directory (/emoncms/):</I></LABEL><INPUT NAME='dir' MAXLENGTH='32' VALUE='";
  sTmp = "";
  for (int i = 0; i < directory.length(); ++i) {    
      if (directory[i] == '\'')
        sTmp += "&#39;";
      else
        sTmp += directory[i];
    }
  s += sTmp.c_str();
  s += "'></P>";
  s += "<P>__________</P>";   
  s += "<P><B><I>Backup Server (optional)</B></I></P>";
  s += "<P><LABEL><I> Write key (apikey=1..32):</I></LABEL><INPUT NAME='ekey2' MAXLENGTH='50' VALUE='";
  sTmp = "";
  for (int i = 0; i < privateKey2.length(); ++i) {    
      if (privateKey2[i] == '\'')
        sTmp += "&#39;";
      else
        sTmp += privateKey2[i];
    }
  s += sTmp.c_str();
  s += "'></P>";
  s += "<P><LABEL><I>Server address (example2.com):</I></LABEL><INPUT NAME='host2' MAXLENGTH='32' VALUE='";
  sTmp = "";
  for (int i = 0; i < host2.length(); ++i) {    
      if (host2[i] == '\'')
        sTmp += "&#39;";
      else
        sTmp += host2[i];
    }
  s += sTmp.c_str();
  s += "'></P>";  
  s +=  "<P><LABEL><I>Database directory (/):</I></LABEL><INPUT NAME='dir2' MAXLENGTH='32' VALUE='";
  sTmp = "";
  for (int i = 0; i < directory2.length(); ++i) {    
      if (directory2[i] == '\'')
        sTmp += "&#39;";
      else
        sTmp += directory2[i];
    }
  s += sTmp.c_str();
  s += "'></P>";
  s += "<P>___________</P>";
  s += "<P><LABEL><I>Node for both servers (default is 0):</I></LABEL><SELECT NAME='node'>";  
  for (int i = 0; i <= 8; ++i) {
    s += "<OPTION VALUE='" + String(i) + "'";
    if (node == String(i))
      s += "SELECTED";
    s += ">" + String(i) + "</OPTION>";
  }
  s += "</SELECT></P>";
  s += "<P><INPUT TYPE='submit'></FORM></P>";        
  s += "<FORM ACTION='confirm'>";  
  s += "<P><INPUT TYPE=SUBMIT VALUE='Erase WiFi Settings'></P>";
  s += "</FORM></P>";
  s += "<P>====================</P>";
  s += "<P><B>SENSOR STATUS</B></P>";
  s += "<P>Number of samples per ";
  s += cycleTime;
  s += " msec is " + String(reportedCount) + "</P>";
  s += "</FONT></HTML>\r\n\r\n";
	server.send(200, "text/html", s);
  delay(1000);
}

void handleCfg() {
  String s;
  String qsid = server.arg("ssid");
  String qhsid = server.arg("hssid");
  if (qhsid != "empty")
    qsid = qhsid;
  String qpass = server.arg("pass");   
  String qkey = server.arg("ekey"); 
  String qkey2 = server.arg("ekey2");
  String qnode = server.arg("node");
  String qhost = server.arg("host");       
  String qhost2 = server.arg("host2");  
  String qdirectory = server.arg("dir");
  String qdirectory2 = server.arg("dir2");  

  writeEEPROM(PASS_START, PASS_MAX_LENGTH, qpass);
  if (privateKey != qkey) {
    writeEEPROM(KEY_START, KEY_MAX_LENGTH, qkey);
    privateKey = qkey;
  }
  if (privateKey2 != qkey2) {
    writeEEPROM(KEY2_START, KEY2_MAX_LENGTH, qkey2);
    privateKey2 = qkey2;
  }
  if (node != qnode) {    
    writeEEPROM(NODE_START, NODE_MAX_LENGTH, qnode);
    node = qnode;
  }
  if (host != qhost) {
    writeEEPROM(HOST_START, HOST_MAX_LENGTH, qhost);
    host = qhost;
  }
  if (host2 != qhost2) {
    writeEEPROM(HOST2_START, HOST2_MAX_LENGTH, qhost2);
    host2 = qhost2;
  }
  if (directory != qdirectory) {
    writeEEPROM(DIRECTORY_START, DIRECTORY_MAX_LENGTH, qdirectory);
    directory = qdirectory;
  }
  if (directory2 != qdirectory2) {
    writeEEPROM(DIRECTORY2_START, DIRECTORY2_MAX_LENGTH, qdirectory2);
    directory2 = qdirectory2;
  }
  if (qsid != "not chosen") {
    writeEEPROM(SSID_START, SSID_MAX_LENGTH, qsid);
    s = "<HTML><FONT SIZE=6><B>Current</B>Sensor</FONT<FONT FACE='Arial'> WiFi Configuration</FONT></FONT><FONT FACE='Arial'><P>Updating Settings...</P>";
    if (qsid != esid.c_str() || qpass != epass.c_str()) {
      s += "<P>Saved to Memory...</P>";
      s += "<P>The Current Sensor will reset and try to join " + qsid + "</P>";
      s += "<P>After about 30 seconds, if successful, please use the IP address</P>";
      s += "<P>assigned by your DHCP server to the Current Sensor in your Browser</P>";
      s += "<P>in order to re-access the CurrentSensor WiFi Configuration page.</P>";
      s += "<P>---------------------</P>";
      s += "<P>If unsuccessful after 90 seconds, the Current Sensor will go back to the";
      s += "<P>default access point at SSID:CurrentSensor.</P>";
      server.send(200, "text/html", s);
      s += "</FONT></HTML>\r\n\r\n";
      WiFi.disconnect();
      delay(2000);
      ESP.reset(); 
    }
    else{
      s += "<FORM ACTION='.'>";
      s += "<P>Saved to Memory...</P>"; 
      s += "<P><INPUT TYPE=SUBMIT VALUE='Continue'></P>";
    }
  }
  else{
     EEPROM.commit();
     delay(100); 
     s = "<HTML><FONT SIZE=6><B>Current</B>Sensor</FONT><FONT FACE='Arial'><P>Error. No network selected. Please select or enter a network.";  
     s += "<FORM ACTION='.'>";  
     s += "<P><INPUT TYPE=SUBMIT VALUE='     OK     '></P>";
  }
  s += "</FORM></FONT></HTML>\r\n\r\n";
  server.send(200, "text/html", s);
}

void handleCfm() {
  String s;
  s = "<HTML><FONT SIZE=6><B>Current</B>Sensor<FONT FACE='Arial'> Confirmation</FONT></FONT>";
  s += "<FONT FACE='Arial'><P>You are about to erase the WiFi settings!</P>";
  s += "<FORM ACTION='reset'>";
  s += "&nbsp;<TABLE><TR>";
  s += "<TD><INPUT TYPE=SUBMIT VALUE='    Continue    '></TD>";
  s += "</FORM><FORM ACTION='.'>";
  s += "<TD><INPUT TYPE=SUBMIT VALUE='    Cancel    '></TD>";
  s += "</FORM>";
  s += "</TR></TABLE>";
  s += "</FONT></HTML>";
  s += "\r\n\r\n";
  server.send(200, "text/html", s);
}

void handleRescan() {
  String s;
  s = "<HTML><FONT SIZE=6><B>Current</B>Sensor<FONT FACE='Arial'> Rescanning...</FONT></FONT>";
  s += "<FONT FACE='Arial'><P>Note.  You may need to manually reconnect to the access point after rescanning.</P>";  
  s += "<P><B>Please wait at least 30 seconds before continuing</B></P>";
  s += "<FORM ACTION='.'>";  
  s += "<P><INPUT TYPE=SUBMIT VALUE='Continue'></P>";
  s += "</FORM></P>";
  s += "</FONT></HTML>\r\n\r\n";
  server.send(200, "text/html", s);
  WiFi.disconnect();
  delay(2000);  
  ESP.reset();
}

void handleRst() {
  String s;
  s = "<HTML><FONT SIZE=6><B>Current</B>Sensor</FONT><FONT FACE='Arial'> WiFi Configuration</FONT></FONT><FONT FACE='Arial'><P>Reset to Defaults:</P>";
  s += "<P>Clearing the EEPROM...</P>";
  s += "<P>The Current Sensor will reset and have an IP address of 192.168.4.1</P>";
  s += "<P>After about 30 seconds, the Current Sensor will activate the access point</P>";
  s += "<P>SSID:CurrentSensor and password:currentsensor</P>";       
  s += "</FONT></HTML>\r\n\r\n";
  resetEEPROM(0, EEPROM_SIZE);
  EEPROM.commit();
  server.send(200, "text/html", s);
  WiFi.disconnect();
  delay(2000);
  ESP.reset();
}

//-------------------------------------------------------------------------------------------------------------------------
// Calculates realPower,apparentPower,powerFactor,Vrms,Irms
// for main panel and solar inverter
// Uses 120Vac line voltaga as the reference voltage 
//-------------------------------------------------------------------------------------------------------------------------

void calculatePower(unsigned int crossings, unsigned int timeout)
{
  unsigned int crossCount = 0;     //Used to measure number of times threshold is crossed.
  
  //main panel current ADS1015 ADC parameters - differential measurement
  double Icalm = 100;              //Ical =  no turns/burden = 100 = (120.0/.04)/30
  int fullscalem = 8192;           //Max voltage in mV
  int adccountm = 4096;
  double phasecal = 2.25;          //For main current measured - given the ADC is 3300sps continuous mode 
                                   //and the delay to read (150 + 43 + 102 usec) 44 is close to the sampling period of 303 usec
                              
  //solar inverter onboard ESP8266 ADC parameters - single ended measurement        
  double Icali = 131;              //Ical =  no turns/burden = 133.33 = (100/.05)/15
  int fullscalei = 1000;           //Max voltage in mV
  double offsetIi = 527;
  int adccounti = 1024;
  double phasecal_inv = 2.6;      //For solar inverter current - measured 43 usec delay from voltage read but worst case can be previous sample 
  
  //reference voltage ADS1015 ADC parameters - differential measurement
  double Vcal = 62.1;              
  int fullscalev = 8192;           //Max voltage in mV
  int adccountv = 4096;
  boolean leave=false;                            //An indicator to exit the while loop
  unsigned long start = millis();              //millis()-start makes sure it doesnt get stuck in the loop if there is an error.
  
  numberOfSamples = 0;
  startV = 4096;
  checkVCross = true;

  //-------------------------------------------------------------------------------------------------------------------------
  // 1) Waits for the waveform to be close to the positive slope 'zero' part in sin curve.
  //-------------------------------------------------------------------------------------------------------------------------
 
  while(leave == false)                                         //Ensure the reference voltage measurements start about the same place
  {
     delayMicroseconds(155);
     previousSampleV = startV;                                  //Used for delay/phase compensation
     starttime = millis();
     startV = ads1015.readADC_Differential_2_3_cont(1);         //Read in the reference voltage only                  
     if (((startV - previousSampleV) > 0) && (startV < (2048*0.05)) && (startV > (2048*(-.05)))) leave=true;  //Check its within range and positive slope      
     if ((starttime - start) > timeout) leave = true;
  }

  //-------------------------------------------------------------------------------------------------------------------------
  // 2) Main measurement loop
  //------------------------------------------------------------------------------------------------------------------------- 
  start = millis();
  while ((crossCount < crossings) && ((millis() - start) < timeout)) 
  {
   
    //-----------------------------------------------------------------------------
    // A) Read in raw voltage and current samples
    //-----------------------------------------------------------------------------
    if (!leave) {
       previousSampleV = sampleV;                          //Used for delay/phase compensation
       sampleV = ads1015.readADC_Differential_2_3_cont(1); //Read in raw voltage signal
    }
    if (leave) {
       sampleV = startV;                                  //Using startV as the first sampleV to ensure no extra delay
       leave = false;
    }
    sampleIi = analogRead(A0);                            //Read in raw solar inverter current signal
    sampleIm = ads1015b.readADC_Differential_0_1_cont(1); //Read in raw main panel current signal         
    numberOfSamples++;                                    //Count number of samples for this set of measurement
    
    //-----------------------------------------------------------------------------
    // B) Subtract offset if necessary.  Not necessary on differential measurements
    //-----------------------------------------------------------------------------
    adjustedSampleIi = sampleIi - offsetIi;               //Single ended measurement for solar inverter currrent
   
    //-----------------------------------------------------------------------------
    // C) Root-mean-square method voltage
    //-----------------------------------------------------------------------------  
    sqV= sampleV * sampleV;                      //1) square voltage values
    sumV += sqV;                                 //2) sum
    
    //-----------------------------------------------------------------------------
    // D) Root-mean-square method current
    //-----------------------------------------------------------------------------   
    sqIi = adjustedSampleIi * adjustedSampleIi;   //1) square current values for the solar inverter current
    sumIi += sqIi;                                //2) sum 
    sqIm = sampleIm * sampleIm;                   //1) square current values for the main panel current
    sumIm += sqIm;                                //2) sum 
    
    //-----------------------------------------------------------------------------
    // E) Phase calibration and Instantaneous power calc
    //-----------------------------------------------------------------------------
    phaseShiftedV = previousSampleV + phasecal * (sampleV - previousSampleV);       //Phase calibration for main panel
    instPm = phaseShiftedV * sampleIm;                                              //Instantaneous Power for main panel
    sumPm += instPm;                                                                //Sum for main panel
    phaseShiftedV = previousSampleV + phasecal_inv * (sampleV - previousSampleV);   //Phase calibration for solar inverter
    instPi = phaseShiftedV * adjustedSampleIi;                                      //Instantaneous Power for solar inverter
    sumPi += instPi;                                                                //Sum for solar inverter
    
    //-----------------------------------------------------------------------------
    // F) Find the number of times the voltage has crossed the initial voltage
    //    - every 2 crosses we will have sampled 1 wavelength 
    //-----------------------------------------------------------------------------       
    lastVCross = checkVCross;   
    if ((millis() - starttime) > (8*crossCount+7)) {       //Checking for next transition after a few ms in case of noise           
      if (sampleV >= startV) checkVCross = true; 
        else checkVCross = false;
      if (lastVCross != checkVCross) crossCount++;
    }
  }
  endtime = millis();
  
  //-------------------------------------------------------------------------------------------------------------------------
  // 3) Post loop calculations
  //------------------------------------------------------------------------------------------------------------------------- 
  //Calculation of the root of the mean of the voltage and current squared (rms)
  //Calibration coefficients applied.
  delay(10);
  double V_RATIO = Vcal *((fullscalev/1000.0) / (adccountv));
  Vrms = V_RATIO * sqrt(sumV / numberOfSamples); 
  
  double I_RATIOi = Icali *((fullscalei/1000.0) / (adccounti));
  Irmsi = I_RATIOi * sqrt(sumIi / numberOfSamples); 

  double I_RATIOm = Icalm *((fullscalem/1000.0) / (adccountm));
  Irmsm = I_RATIOm * sqrt(sumIm / numberOfSamples); 

  //Calculation power values
  realPowerm = V_RATIO * I_RATIOm * sumPm / numberOfSamples;
  apparentPowerm = Vrms * Irmsm;
  if (apparentPowerm != 0)                          // prevent divide by 0 condition
     powerFactorm = realPowerm / apparentPowerm;
  realPoweri = 2 * V_RATIO * I_RATIOi * sumPi / numberOfSamples;
  apparentPoweri = 2 * Vrms * Irmsi;
  if (apparentPoweri != 0)
    powerFactori = realPoweri / apparentPoweri;
 
  //Reset accumulators
  sumV = 0;
  sumIi = 0;
  sumPi = 0;
  sumIm = 0;
  sumPm = 0;
//--------------------------------------------------------------------------------------       
}

void downloadEEPROM() {
  esid = readEEPROM(SSID_START, SSID_MAX_LENGTH);
  epass = readEEPROM(PASS_START, PASS_MAX_LENGTH);
  privateKey  = readEEPROM(KEY_START, KEY_MAX_LENGTH);
  privateKey2  = readEEPROM(KEY2_START, KEY2_MAX_LENGTH);
  node = readEEPROM(NODE_START, NODE_MAX_LENGTH);
  host = readEEPROM(HOST_START, HOST_MAX_LENGTH);
  host2 = readEEPROM(HOST2_START, HOST2_MAX_LENGTH);
  directory = readEEPROM(DIRECTORY_START, DIRECTORY_MAX_LENGTH);
  directory2 = readEEPROM(DIRECTORY2_START, DIRECTORY2_MAX_LENGTH);
}

void setup() {
  ads1015.setGain(GAIN_ONE);
  ads1015b.setGain(GAIN_ONE);
  ads1015.begin();              // Initialize ads1015, used to sample the reference voltage
  ads1015b.begin();             // Initialize ads1015b, used to sample the main panel current
  Wire.setClock(400000);        // default is 100kHz but need faster
	Serial.begin(115200);
  delay(1000);
  EEPROM.begin(512);
  pinMode(0, INPUT);
  char tmpStr[40];
  downloadEEPROM();
// go ahead and make a list of networks in case user needs to change it 
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(1000);
  int n = WiFi.scanNetworks();
  Serial.print(n);
  Serial.println(" networks found");
  st = "<SELECT NAME='ssid'>";
  int found_match = 0;
  delay(1000);
  for (int i = 0; i < n; ++i) {
    st += "<OPTION VALUE='"; 
    st += String(WiFi.SSID(i)) + "'";
    if (String(WiFi.SSID(i)) == esid.c_str()) {
      found_match = 1;
      Serial.println("found match");
      st += "SELECTED";
    }
    st += "> " + String(WiFi.SSID(i));
    st += " </OPTION>";
  }
  if (!found_match)
    if (esid != 0)   
      st += "<OPTION VALUE='" + esid + "'SELECTED>" + esid + "</OPTION>";
    else{
      if (!n)
        st += "<OPTION VALUE='not chosen'SELECTED> No Networks Found!  Select Rescan or Manually Enter SSID</OPTION>";
      else
        st += "<OPTION VALUE='not chosen'SELECTED> Choose One </OPTION>";
    }
  st += "</SELECT>";
  delay(100);     
  //Serial.println(esid);
  if ( esid != 0 ) { 
    //Serial.println(" ");
    //Serial.print("Connecting as Wifi Client to: ");
    //Serial.println(esid.c_str());
    //Serial.println(epass.c_str());
    //Serial.println(esid);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(); 
    WiFi.begin(esid.c_str(), epass.c_str());
    delay(50);
    int t = 0;
    int attempt = 0;
    while (WiFi.status() != WL_CONNECTED) {
      // test esid
      // Serial.print("#");
      delay(500);
      t++;
      if (t >= 20) {
        Serial.println(" ");
        Serial.println("Trying Again...");
        delay(2000);
        WiFi.disconnect(); 
        WiFi.begin(esid.c_str(), epass.c_str());
        t = 0;
        attempt++;
        if (attempt >= 5) {
          // Serial.println();
          // Serial.print("Configuring access point...");
          WiFi.mode(WIFI_STA);
          WiFi.disconnect();
          delay(1000);
          int n = WiFi.scanNetworks();
          //Serial.print(n);
          //Serial.println(" networks found");
          delay(1000);
          st = "<SELECT NAME='ssid'><OPTION VALUE='not chosen'SELECTED> Try again </OPTION>";
          esid = "";
          for (int i = 0; i < n; ++i) {
            st += "<OPTION VALUE='";
            st += String(WiFi.SSID(i)) + "'> " + String(WiFi.SSID(i));
            st += " </OPTION>";
          }
          st += "</SELECT>";
          delay(100);
          WiFi.softAP(ssid, password);
          IPAddress myIP = WiFi.softAPIP();
          // Serial.print("AP IP address: ");
          // Serial.println(myIP);
          Serial.println("SSID...CurentSensor.");
          delay(100);  
          Serial.println("PASS...currentsensor.");
          delay(100);
          Serial.println("IP_Address......");
          delay(100);
          sprintf(tmpStr,"%d.%d.%d.%d",myIP[0],myIP[1],myIP[2],myIP[3]);
          Serial.println(tmpStr);
          wifi_mode = 1;       //SSID, but no connection  
          break;
        }
      }
    }
  }
  else {  //AP mode with no SSID in EEPROM
    Serial.println();
    Serial.print("Configuring access point...");
    //WiFi.mode(WIFI_STA);
    //WiFi.disconnect();
    delay(100);
    WiFi.softAP(ssid, password);
    IPAddress myIP = WiFi.softAPIP();
    //Serial.print("AP IP address: ");
    //Serial.println(myIP);
    Serial.println("SSID...CurrentSensor.");
    delay(100);
    Serial.println("PASS...currentsensor.");
    delay(100);  
    Serial.println("IP_Address......");
    delay(100);
    sprintf(tmpStr,"%d.%d.%d.%d",myIP[0],myIP[1],myIP[2],myIP[3]);
    Serial.println(tmpStr);
    wifi_mode = 2; //AP mode with no SSID in EEPROM
  }
	if (wifi_mode == 0) {     // connected
    //Serial.println(" ");
    //Serial.println("Connected as a Client");
    IPAddress myAddress = WiFi.localIP();
    //Serial.println(myAddress);
    Serial.println("Client-IP.......");
    delay(100);
    sprintf(tmpStr,"%d.%d.%d.%d",myAddress[0],myAddress[1],myAddress[2],myAddress[3]);
    Serial.println(tmpStr);
  }
	server.on("/", handleRoot);
  server.on("/a", handleCfg);
  server.on("/confirm", handleCfm);
  server.on("/reset", handleRst);
  server.on("/rescan", handleRescan);
	server.begin();
	Serial.println("HTTP server started");
  delay(100);
  bootOTA();
  sampleV = ads1015.readADC_Differential_2_3_cont(0);   //Set up in continuous mode at 3300sps and read in reference voltage signal 
  sampleIm = ads1015b.readADC_Differential_0_1_cont(0); //Set up in continuous mode at 3300sps and read in main current signal 
  Timer = millis();
  //wifi_set_sleep_type(MODEM_SLEEP_T);
}

void loop() {
  server.handleClient();
  ArduinoOTA.handle();        // initiates OTA update capability
  int erase = 0;  
  char tmpStr[40];

  buttonState = digitalRead(0);
  while (buttonState == LOW) {
    buttonState = digitalRead(0);
    erase++;
    if (erase >= 15000) {
      resetEEPROM(0, SSID_MAX_LENGTH + PASS_MAX_LENGTH); // only want to erase ssid and password
      int erase = 0;
      WiFi.disconnect();  
      Serial.println("Finished...");  
      delay(2000);
      ESP.reset(); 
    } 
  }
  // Remain in AP mode for 5 Minutes before resetting
  if (wifi_mode == 1) {
    if ((millis() - Timer) >= 300000) {
      ESP.reset();
    }
  }   
  if (wifi_mode == 0 && privateKey != 0) {
    prevVrms = Vrms;            
    while ((numberOfSamples < 47) && ((millis()- Timer) < SERVER_UPDATE_RATE)) {   //based on 3300sps rate and reading time total read cycle is about 370 usec or at least 45 samples per cycle
      calculatePower(2,1500);                                                      //measure 1 cycle
      //sprintf(tmpStr,"COUNT = %d,  %d ms",numberOfSamples, endtime-starttime);
      //Serial.println(tmpStr);
      reportedCount = numberOfSamples;
      cycleTime = endtime - starttime;
      if ((sampleV - startV) > 30) numberOfSamples = 0;
      if ((endtime - starttime) != 17) numberOfSamples = 0;           //1 cycle at 60Hz is 16.667 ms
      if (abs(prevVrms-Vrms) > 0.5) numberOfSamples = 0;
    }
    //delay(100);
    if ((millis() - Timer) >= 10000) {
      Timer = millis();
      numberOfSamples = 0;
      // We now create a URL for database 
      String url;    
      String url2;
      String tmp;
      String url_amp_c = inputID_AMP_C;
      url_amp_c += Irmsm;
      url_amp_c += ",";
      String url_amp_g = inputID_AMP_G;
      url_amp_g += Irmsi;
      url_amp_g += ",";
      String url_volt = inputID_VOLT;
      url_volt += Vrms;
      url_volt += ",";
      String url_realm = inputID_RealPowerM;
      url_realm += realPowerm;
      url_realm += ",";
      String url_appm = inputID_AppPowerM;
      url_appm += apparentPowerm;
      url_appm += ",";
      String url_pfm = inputID_PowerFactorM;
      url_pfm += powerFactorm;
      url_pfm += ",";
      String url_reali = inputID_RealPowerI;
      url_reali += realPoweri;
      url_reali += ",";
      String url_appi = inputID_AppPowerI;
      url_appi += apparentPoweri;
      url_appi += ",";
      String url_pfi = inputID_PowerFactorI;
      url_pfi += powerFactori; 

      tmp = e_url;
      tmp += node;
      tmp += "&json={";  
      tmp += url_amp_c;  
      tmp += url_amp_g;   
      tmp += url_volt;   
      tmp += url_realm;   
      tmp += url_appm;     
      tmp += url_pfm; 
      tmp += url_reali;     
      tmp += url_appi;      
      tmp += url_pfi;
      tmp += "}&";

      url = directory.c_str();   //need to use constant string to filter out unnecessary padding of control characters when reading from memory
      url += tmp;
      url2 = directory2.c_str();
      url2 += tmp;
    
      url += privateKey.c_str();  //need to use constant string to filter out unnecessary padding of control characters when reading from memory
      url2 += privateKey2.c_str();
     
      // Use WiFiClient class to create TCP connections
      WiFiClient client;
      const int httpPort = 80;
        
      // This will send the request to the server   
      if (client.connect(host.c_str(), httpPort)) {  //need to use constant string to filter out unnecessary padding of control characters when reading from memory
        client.print(String("GET ") + url + " HTTP/1.1\r\n" + "Host: " + host.c_str() + "\r\n" + "Connection: close\r\n\r\n");
        delay(10);
        while(client.available()) {
          String line = client.readStringUntil('\r');
        }
      }
      if (client.connect(host2.c_str(), httpPort)) {
        client.print(String("GET ") + url2 + " HTTP/1.1\r\n" + "Host: " + host2.c_str() + "\r\n" + "Connection: close\r\n\r\n");
        delay(10);
        while(client.available()) {
          String line = client.readStringUntil('\r');
        }
      }  
      //Serial.println(host);
      //Serial.println(host.c_str());
      //Serial.println(directory.c_str());
      //Serial.println(e_url);
      //Serial.println(url);
      //Serial.println(url2);
    }
  }
}

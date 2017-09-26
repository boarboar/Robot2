#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <Wire.h>

#include "stat.h"
#include "cfg.h"
#include "cmd.h"
#include "controller.h"
#include "logger.h"

void doCycle();

const char* ssid = "NETGEAR";
const char* password = "boarboar";
const char* cfg_file = "/config.json";
const int udp_port = 4444;
const int CYCLE_TO = 5;
//const int CYCLE_MED_TO = 1000; // test only
const int CYCLE_MED_TO = 120; 
const int CYCLE_SLOW_TO = 2000;

uint32_t last_cycle;
uint32_t last_med_cycle;
uint32_t last_slow_cycle;

CmdProc& cmd = CmdProc::Cmd;

ADC_MODE(ADC_VCC);
//void pcf_test();

void setup() {
  delay(2000);
  Serial.begin(115200);
  
  if(CfgDrv::Cfg.init() && CfgDrv::Cfg.load(cfg_file)) {
    Serial.println(F("Cfg loaded"));
  } else {
    Serial.println(F("Failed to load cfg, using default!"));
  }

  byte mac[6]; 
  uint8_t i = 0;
  WiFi.macAddress(mac);
  Serial.print(F("MAC: "));
  for(i=0; i<6; i++) {
    Serial.print(mac[i],HEX);
    if(i<5) Serial.print(F(":"));
  }
 
  WiFi.begin(ssid, password);
  Serial.print(F("\nConnecting to ")); Serial.print(ssid);
  i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 40) {delay(500); Serial.print(".");}
  Serial.println();
  if(i == 21){
    Serial.print(F("Could not connect to ")); Serial.println(ssid);
    delay(10000);
    ESP.reset();
  }
  
  delay(1000);
 
  if(cmd.init(udp_port)) {
    Serial.print(F("Ready! Listening on "));
    Serial.print(WiFi.localIP());
    Serial.print(":");
    Serial.println(udp_port);
  } else {
    Serial.println(F("Failed to init UDP socket!"));
    delay(1000);
    ESP.reset();
  } 

  //cmd.sendAlarm(CmdProc::ALR_RESET, 0);

  Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_SYS,  Logger::UMP_LOGGER_ALARM, -1, "RSTED");  
 
  yield();

  CfgDrv::Cfg.printCtrlParams();

  if(Controller::ControllerProc.init()) { 
    Serial.print(F("Ctrl Ready! Sensors: "));
    Serial.println(Controller::ControllerProc.getNumSensors());        
  } else {
    Serial.println(F("Failed to init Ctrl!"));
  } 

  Logger::Instance.flushEvents();
  
  last_cycle=last_med_cycle=last_slow_cycle=millis();
}

void loop() {
  if(cmd.connected()) {
    if (cmd.read()) {
      doCycle(); yield();
      cmd.doCmd();
      doCycle(); yield();
      cmd.respond();      
    }
  }
  doCycle();
}

void doCycle() {
  uint32_t t = millis();
  uint16_t dt=t-last_cycle;
 
   // Do fast cycle
  if(dt < CYCLE_TO) return;
  last_cycle = t;
  // nothing to do...
  
  // Do medium cycle // (50ms???)
  dt=t-last_med_cycle;
  if(dt < CYCLE_MED_TO) return;
  last_med_cycle = t;

  Controller::ControllerProc.process();
  Controller::ControllerProc.processAlarms();  
  Logger::Instance.flushEvents(); // here????
  
// Do slow cycle // (2000 ms)
  dt=t-last_slow_cycle;
  if(dt < CYCLE_SLOW_TO) return;
  last_slow_cycle = t;

  yield();

  if(Controller::ControllerProc.isNeedReset()) {
    //cmd.sendAlarm(CmdProc::ALR_CTL_RESET, 0);
    Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL,  Logger::UMP_LOGGER_ALARM, -1, "NEEDRST");  
    Controller::ControllerProc.reset();
    //Controller::ControllerProc.init();
    //if(!mpu_rst) Controller::ControllerProc.start(); // forced start for testing purposes...    
    yield();
  }
  
  if(CfgDrv::Cfg.needToStore()) CfgDrv::Cfg.store(cfg_file);
  //cmd.sendSysLogStatus();

}



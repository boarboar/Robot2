#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include "cmd.h"
#include "cfg.h"
#include "logger.h"

Logger Logger::Instance ; // singleton

Logger::Logger() {
  id=0;
  q_h=q_t=0;
  for(int i=0; i<UMP_LOGGER_NQUEUE; i++) {
    q[i].type=UMP_LOGGER_TYPE_NONE; q[i].params.s[0]=0;      
  }
}

boolean Logger::putEvent(uint8_t module,  uint8_t level, uint8_t code, int16_t p0, int16_t p1, int16_t p2, int16_t p3, int16_t p4, int16_t p5, int16_t p6, int16_t p7) {
  //boolean ovf=false;
  //if(q[q_t].module && q[q_t].type) ovf=true;
  if(CfgDrv::Cfg.log_on<level) return false;
  q[q_t].id=++id;
  q[q_t].module=module;
  q[q_t].level=level;
  q[q_t].code=code;
  q[q_t].type=UMP_LOGGER_TYPE_I16;
  q[q_t].params.p[0]=p0;
  q[q_t].params.p[1]=p1;
  q[q_t].params.p[2]=p2;
  q[q_t].params.p[3]=p3;
  q[q_t].params.p[4]=p4;
  q[q_t].params.p[5]=p5;
  q[q_t].params.p[6]=p6;
  q[q_t].params.p[7]=p7;

  //Serial.print(F("PUT EVT ")); Serial.print(q[q_t].id); Serial.print(F("\t at  ")); Serial.println(q_t);   
  
  q_t++;
  if(q_t==UMP_LOGGER_NQUEUE) q_t=0;
  return true;  
}

boolean Logger::putEvent(uint8_t module,  uint8_t level, uint8_t code, const char* pa) {
  if(CfgDrv::Cfg.log_on<level) return false;
  q[q_t].id=++id;
  q[q_t].module=module;
  q[q_t].level=level;
  q[q_t].code=code;
  q[q_t].type=UMP_LOGGER_TYPE_S;
  strncpy(q[q_t].params.s, pa, UMP_LOGGER_NSSZ);
  q[q_t].params.s[UMP_LOGGER_NSSZ-1]=0;
  //Serial.print(F("PUT EVT ")); Serial.print(q[q_t].id); Serial.print(F("\t type  ")); Serial.print(q[q_t].type); Serial.print(F("\t at  ")); Serial.println(q_t);   
  q_t++;
  if(q_t==UMP_LOGGER_NQUEUE) q_t=0;
  return true;  
}

boolean Logger::flushEvents() {
  for(uint8_t i=0; i<UMP_LOGGER_NQUEUE && q[q_h].type!=UMP_LOGGER_TYPE_NONE; i++) { 
    //Serial.print(F("GET EVT ")); Serial.print(q[q_h].id);  Serial.print(F("\t type  ")); Serial.print(q[q_h].type); Serial.print(F("\t at  ")); Serial.println(q_h);
    if(q[q_h].type==UMP_LOGGER_TYPE_S) CmdProc::Cmd.sendEvent(q[q_h].id, q[q_h].module, q[q_h].level, q[q_h].code, q[q_h].params.s);
    else  CmdProc::Cmd.sendEvent(q[q_h].id, q[q_h].module, q[q_h].level, q[q_h].code, UMP_LOGGER_NPARAM, q[q_h].params.p);    
    q[q_h].type=UMP_LOGGER_TYPE_NONE;     
    q_h++;         
    if(q_h==UMP_LOGGER_NQUEUE) q_h=0;
  }
  return true;
}



  


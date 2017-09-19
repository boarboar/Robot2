//#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include "FS.h"
#include "logger.h"
#include "cfg.h"

#define MAX_CFG_LINE_SZ 80

const int NCFGS=4; 
const char *CFG_NAMES[NCFGS]={"DBG", "SYSL", "SPP_1", "SPP_2"};
enum CFG_ID {CFG_DBG=0, CFG_SYSL=1, CFG_PIDS_1=2, CFG_PIDS_2=3};

CfgDrv CfgDrv::Cfg; // singleton

CfgDrv::CfgDrv() : log_on(0), debug_on(0), log_port(0),  
bear_pid{8, 120, 4, 10, 100}, speed_pid{4, 20, 4, 150, 50}, 
fs_ok(false), dirty(false), last_chg(0)
 {
  }

int16_t CfgDrv::init() {
  fs_ok=SPIFFS.begin();
  if (!fs_ok) {
    Serial.println(F("Failed to mount FS"));
    return 0;
  }
  Serial.println(F("FS mounted"));
  return 1;
}

int16_t CfgDrv::load(const char* fname) {
  if (!fs_ok) return 0;
  
  char buf[MAX_CFG_LINE_SZ];
  uint32_t ms1=millis();
  File f = SPIFFS.open(fname, "r");
  if (!f) {
    Serial.println(F("Failed to open config file"));
    return 0;
  }
  size_t size = f.size();
  int c=0;

  while(c!=-1) { // while !EOF
    char *p = buf;
    while(1) { // new line
      c=f.read();
      //Serial.println((char)c);
      if(c==-1 || c=='\n'  || c=='\r') break;
      if(p-buf<MAX_CFG_LINE_SZ-1) *p++=c;       
    }
    if(p>buf) { //non-empty
      *p=0; 
      Serial.println(buf);  
      StaticJsonBuffer<200> jsonBuffer;
      JsonObject& json = jsonBuffer.parseObject(buf);  
      if (json.success()) {
        const char* cmd = json["C"];
        if(!strcmp(cmd, "SYSL")) setSysLog(json);
        else if(!strcmp(cmd, "SPP_1") || !strcmp(cmd, "SPP_2")) setPidParams(json);
        else {
          //Serial.println(F("Bad param or TODO"));
        }
      }
    } // new line
  } // while !EOF
  f.close();
  dirty=false;
  uint16_t t=millis()-ms1;
  Serial.print(F("Cfg sz ")); Serial.print(size); Serial.print(F(", read in ")); Serial.println(t);
  return 1;
}

int16_t CfgDrv::store(const char* fname) {
  if (!fs_ok) return 0;
  uint32_t ms1=millis();
  dirty=false; // to avoid multiple writes...
  File f = SPIFFS.open(fname, "w");
  if (!f) {
    Serial.println(F("Failed to open config file (w)"));
    return 0;
  }
  for(int i=0; i<NCFGS; i++) {
    StaticJsonBuffer<400> jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    Serial.print(F("Writing ")); Serial.println(CFG_NAMES[i]);
    json["C"]=CFG_NAMES[i];
    switch(i) {
      case CFG_DBG: json["ON"]=debug_on; break;
      case CFG_SYSL: {
        String addr = log_addr.toString();
        json["ON"]=log_on;
        json["PORT"]=log_port;
        json["ADDR"]=addr;
        break;
      }
      case CFG_PIDS_1:
      case CFG_PIDS_2:{
        //struct pid_params *p=NULL;
        int16_t *p=NULL;
        if(i==CFG_PIDS_1) {
          json["P"]=1;
          //p=&bear_pid;          
          p=bear_pid;          
        } else {
          json["P"]=2;
          //p=&speed_pid;
          p=speed_pid;
        }
        
        JsonArray& par = json.createNestedArray("PA");
        /*
        par.add(p->gain_p);
        par.add(p->gain_d);
        par.add(p->gain_i);
        par.add(p->gain_div);
        par.add(p->limit_i); 
        */
        for (int i=0; i<CFG_PID_NP; i++) par.add(p[i]);
        break;  
      }
      default:;    
    }
    
    json.printTo(f);
    f.write('\n');
    yield();
  }
  f.close();
  uint16_t t=millis()-ms1;
  Serial.print(F("Cfg written in ")); Serial.println(t);
  Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_SYS,  Logger::UMP_LOGGER_ALARM, -1, "CFGSTR");  
  return 1;
}

bool CfgDrv::needToStore() {
  return dirty && (millis()-last_chg)/1000>CfgDrv::LAZY_WRITE_TIMEOUT;
}

bool CfgDrv::setSysLog(JsonObject& root) {
  //uint8_t on = root["ON"] ? 1 : 0;
  uint8_t on = root["ON"];
  long port = root["PORT"];
  const char* addr = root["ADDR"];
  IPAddress newaddr;
  /*
  if(!on) { 
    if(!log_on) return true;
    Serial.println(F("SET_SYSL OFF")); 
    log_on=0;
    dirty=true; 
    last_chg=millis();
    return true;
  } 
  */ 
  if(on && !(port && addr && *addr && WiFi.hostByName(addr, newaddr))) return false;    
  if(log_addr==newaddr && log_port==port && log_on==on) return true; // nothing to change
  log_addr=newaddr;
  log_port=port;
  log_on=on;
  dirty=true;
  last_chg=millis();
  if(log_on) {
    Serial.print(F("SET_SYSL ")); Serial.print(log_on); Serial.print(":"); Serial.print(log_addr); Serial.print(":"); Serial.println(Cfg.log_port);     
  }
  else Serial.println(F("SET_SYSL OFF")); 
  return true;
}


bool CfgDrv::setPidParams(JsonObject& json) {
  JsonArray& par = json["PA"].asArray();
  /*
  struct pid_params *p=NULL;
  if(json["P"]==1) p=&bear_pid;
  else  if(json["P"]==2) p=&speed_pid;
  if(p==NULL) return false;
  p->gain_p=par[0];
  p->gain_d=par[1];
  p->gain_i=par[2];
  p->gain_div=par[3];
  p->limit_i=par[4];
  */
  int16_t *p=NULL;
  if(json["P"]==1) p=bear_pid;
  else  if(json["P"]==2) p=speed_pid;
  if(p==NULL) return false;
  for (int i=0; i<CFG_PID_NP; i++) p[i]=par[i];

  if(json["S"]==1) {
    dirty=true;
    last_chg=millis();
    Serial.println(F("SPP set to save"));
  }
  return true;  
}

void CfgDrv::printCtrlParams() {
  int16_t *p=NULL;
  Serial.print(F("PID B: "));
  p=bear_pid;
  for (int i=0; i<CFG_PID_NP; i++) { Serial.print(p[i]); Serial.print(F(" ")); }
  Serial.println();
  Serial.print(F("PID S: "));
  p=speed_pid;
  for (int i=0; i<CFG_PID_NP; i++) { Serial.print(p[i]); Serial.print(F(" ")); }
  Serial.println();
}
  
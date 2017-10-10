#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include "controller.h"
#include "cfg.h"
#include "logger.h"

const int M_POW_NORM=100;
    
Controller Controller::ControllerProc ; // singleton

Controller::Controller() : cready(false), reset_lvl(CTL_RST_NONE), nsens(0) {
  }

uint8_t Controller::getStatus() { return cready; }
uint8_t Controller::isNeedReset() { return reset_lvl; }
void    Controller::setReset(uint8_t reset_lvl) {  this->reset_lvl=reset_lvl; }

bool Controller::init() {
  cmgr.Init();
  cready=false;
  crd[0]=crd[1]=0;
  dist=0;
  yaw=0;
  imu_stat=0;
  for(int i=0; i<SENS_SIZE; i++) sensors[i]=-2;
  
  cready=_testConnection();
  if(cready) {
    Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL, Logger::UMP_LOGGER_EVENT, CTL_FAIL_INIT, "CTL_INT_OK");  
    nsens=_getNumSensors();    
    reset_lvl=CTL_RST_NONE;     
    yield();
  } else {
    Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL,  Logger::UMP_LOGGER_ALARM, CTL_FAIL_INIT, "CTL_INT_FL");  
    reset_lvl=CTL_RST_CTL; 
  }
  return cready;
}

bool Controller::reset() {
    switch(reset_lvl) {
      case CTL_RST_IMU :
        imu_stat=0;
        _resetIMU();
        reset_lvl=CTL_RST_CTL; 
        break;
      case CTL_RST_CTL :  
        init();
        break;
     default:;
    }
  return true;
}

bool Controller::process() {
  if(!cready) return false;
  if(!_getData_1()) return false;
  return true;
}

bool Controller::processAlarms() {
  uint16_t read_cnt=8; // do not read mor than that every time
  if(!cready) return false;
  while(0==cmgr.Get(REG_ALARM) && cmgr.GetResultCnt()!=0 && --read_cnt) {
      int n=cmgr.GetResultCnt();    
      const int16_t *va=cmgr.GetResultVal();
      Serial.print("ALR :");
      
      for(int i=0; i<n; i++) {
        Serial.print(va[i]);    
        Serial.print(" ");
      }
      Serial.println();
  }
  return true;
}


uint8_t Controller::getIMUStatus() { return imu_stat;}
uint8_t Controller::getNumSensors() { return nsens;}
int16_t Controller::getX_cm() { return crd[0];}
int16_t Controller::getY_cm() { return crd[1];}
int16_t Controller::getDist_cm() { return dist;}
int16_t Controller::getSpeed_cmps() { return speed;}
int16_t Controller::getYaw_grad() { return yaw;}
int16_t *Controller::getSensors() { return sensors;}
int16_t *Controller::getPower() { return pow;}

uint8_t Controller::_testConnection() {
  int res=cmgr.Get(REG_ID);
  if(res!=0 || cmgr.GetResultCnt()==0 || *(cmgr.GetResultVal())!=CM_ID ) return false;
  return true;
}

uint8_t Controller::_getNumSensors() {
  int res=cmgr.Get(REG_N_SENS);
  if(res!=0 || cmgr.GetResultCnt()==0 || *(cmgr.GetResultVal())>SENS_SIZE) {
    Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL, Logger::UMP_LOGGER_ALARM, CTL_FAIL_RD, REG_N_SENS, res, cmgr.GetResultCnt());
    return 0;
  }
  return *(cmgr.GetResultVal());
}

uint8_t Controller::_resetIMU() {
  int16_t v=reset_lvl;
  cmgr.Set(REG_RESET, &v, 1);  
  // do not expect answer
  return true;
}

uint8_t Controller::_getData_1() {
  int res=cmgr.Get(REG_STATUS);
  // St[0] yaw[1] X[2] Y[3] Dist[4] PW[5] PW[6] Speed[8] 
  if(res!=0 || cmgr.GetResultCnt()!=8) {
    Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL, Logger::UMP_LOGGER_ALARM, CTL_FAIL_RD, REG_ALL, res, cmgr.GetResultCnt());
    return 0;
  }

  //uint16_t cnt=cmgr.GetResultCnt();
  const int16_t *v=cmgr.GetResultVal();

  imu_stat=v[0]&0xFF;
  yaw=v[1];
  crd[0]=v[2];
  crd[1]=v[3];
  dist=v[4];
  pow[0]=v[5];
  pow[1]=v[6];
  speed = v[7];
  
  res=cmgr.Get(REG_SENS);
  if(res!=0 || cmgr.GetResultCnt()!=nsens) {
    Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL, Logger::UMP_LOGGER_ALARM, CTL_FAIL_RD, REG_MOTOR_POWER, res, cmgr.GetResultCnt());
    return true;
  } 
  
  v=cmgr.GetResultVal();
  for(int i=0; i<nsens; i++) sensors[i]=v[i];
  return true;
}


uint8_t Controller::_getData() {
  int res=cmgr.Get(REG_ALL);
  
  if(res!=0 || cmgr.GetResultCnt()==0) {
    Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL, Logger::UMP_LOGGER_ALARM, CTL_FAIL_RD, REG_ALL, res, cmgr.GetResultCnt());
    return 0;
  }

  uint16_t cnt=cmgr.GetResultCnt();
  const int16_t *v=cmgr.GetResultVal();

  // St[0] yaw[1] X[2] Y[3] Dist[4] Sens[5..]
  imu_stat=v[0]&0xFF;
  yaw=v[1];
  if(cnt<4) return true;  
  crd[0]=v[2];
  crd[1]=v[3];
  if(cnt<5) return true;
  dist=v[4];

  if(cnt<nsens+5) return true;
  for(int i=0; i<nsens; i++) sensors[i]=v[5+i];
  
  res=cmgr.Get(REG_MOTOR_POWER);
  if(res!=0 || cmgr.GetResultCnt()<2) {
    Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL, Logger::UMP_LOGGER_ALARM, CTL_FAIL_RD, REG_MOTOR_POWER, res, cmgr.GetResultCnt());
  } else {
    v=cmgr.GetResultVal();
    pow[0]=v[0];
    pow[1]=v[1];
  }

  return true;
}

bool Controller::setTargPower(float l, float r) {
  int16_t v[2];  
  v[0]=l*M_POW_NORM; v[1]=r*M_POW_NORM; // temp  
  Serial.print(F("DPOW=")); Serial.print(v[0]); Serial.print(F("\t ")); Serial.println(v[1]);
  if(0==cmgr.Set(REG_MOTOR_POWER, v, 2)) return true;    
  return false;
}

bool Controller::setTargSteering(int16_t s) {
  int16_t v;  
  v=s; 
  Serial.print(F("STR=")); Serial.println(v);
  if(0==cmgr.Set(REG_STEER, &v, 1)) return true;      
  return false; 
}

bool Controller::setTargBearing(int16_t s) {
  int16_t v;  
  v=s; 
  Serial.print(F("BER=")); Serial.println(v);
  if(0==cmgr.Set(REG_MOVE_BEAR, &v, 1)) return true;      
  return false; 
}

bool Controller::setTargSpeed(int16_t tspeed) {
  int16_t v;  
  v=tspeed; 
  Serial.print(F("MOV=")); Serial.println(v);
  if(0==cmgr.Set(REG_MOVE, &v, 1)) return true;      
  return false;
}


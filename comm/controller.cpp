#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include "controller.h"
#include "cfg.h"
#include "logger.h"

/*
const int DEV_ID=4;
const int M_POW_MIN=30; 
const int M_POW_MAX=200;
const int M_POW_NORM=100;
const int M_SPEED_NORM=200;

const int M_CTR_OBST_WARN_ON_DIST=45; //cm 
const int M_CTR_OBST_WARN_OFF_DIST=50; //cm 
const int M_CTR_OBST_STOP_DIST=12; //cm 
const int M_CTR_OBST_MAX_TURN=45;
//const int M_CTR_OBST_WARN_NREP=2;
*/
    
Controller Controller::ControllerProc ; // singleton

Controller::Controller() : cready(false), reset_lvl(CTL_RST_NONE), nsens(0) /*, act_advance{0}, act_power{0},sensors{0}*/ {
  }

uint8_t Controller::getStatus() { return cready; }

//uint8_t Controller::isDataReady() { return pready && data_ready; }

uint8_t Controller::isNeedReset() { return reset_lvl; }
void    Controller::setReset(uint8_t reset_lvl) {  this->reset_lvl=reset_lvl; }


bool Controller::init() {
  cmgr.Init();
  cready=false;
  crd[0]=crd[1]=0;
  dist=0;
  yaw=0;
  for(int i=0; i<SENS_SIZE; i++) sensors[i]=-2;
  
  /*
  data_ready=0;
  //fail_reason=0;
  nsens=0;
  //act_rot_rate[0]=act_rot_rate[1]=0;
  act_advance[0]=act_advance[1]=0;
  act_power[0]=act_power[1]=0;
  base_pow=0;
  delta_pow=0;
  targ_speed=0;
  rot_speed=0;
  proc_step=0;
  */
  //resetIntegrator();
  
  cready=_testConnection();
  if(cready) {
    Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL, Logger::UMP_LOGGER_EVENT, CTL_FAIL_INIT, "CTL_INT_OK");  
    nsens=_getNumSensors();    
    reset_lvl=CTL_RST_NONE;     
    yield();
    //setStart(0);  
  } else {
    Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL,  Logger::UMP_LOGGER_ALARM, CTL_FAIL_INIT, "CTL_INT_FL");  
    reset_lvl=CTL_RST_CTL; 
  }
  return cready;
}

bool Controller::reset() {
    switch(reset_lvl) {
      case CTL_RST_IMU :
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

/*
void Controller::resetIntegrator() {
  
  dist=0.0f;
  //angle=0.0f;
  r[0]=r[1]=0.0f;
  speed=0;
  for(int i=0; i<SPEED_R_SZ; i++) speed_r[i]=0;
  curr_yaw=0;
  targ_bearing=0;  
  err_bearing_p_0=err_bearing_i=0;
  act_advance_0[0]=act_advance[0];
  act_advance_0[1]=act_advance[1];
  err_speed_p_0=err_speed_i=0; 
  pid_cnt=0;
  qsum_err=0;

  last_obst=0xFF;
  sens_alarmed=false;
  
  Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL,  Logger::UMP_LOGGER_EVENT, CTL_LOG_PBPID, CfgDrv::Cfg.bear_pid.gain_p, CfgDrv::Cfg.bear_pid.gain_d, CfgDrv::Cfg.bear_pid.gain_i, CfgDrv::Cfg.bear_pid.gain_div, CfgDrv::Cfg.bear_pid.limit_i);  

}

*/

/*
bool Controller::start() {
  Serial.println(F("CTRL START"));
  setStart(1);
  return true;
}
*/

bool Controller::process(float yaw, uint32_t dt) {
  if(!cready) return false;

  if(!_getData()) return false;
  
/*  
    getActAdvance();
    
    if(abs(act_advance[0]-act_advance_0[0])>1024 || abs(act_advance[1]-act_advance_0[1])>1024) {
      Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL,  Logger::UMP_LOGGER_ALARM, CTL_FAIL_OVF, act_advance[0], act_advance[1], act_advance_0[0], act_advance_0[1]);    
      act_advance_0[0]=act_advance[0];
      act_advance_0[1]=act_advance[1];
      // RESET PERIPH ?
      //act_advance[0]=act_advance_0[0];
      //act_advance[1]=act_advance_0[1];
      return false;
      }


    getSensors();
    
  curr_yaw=yaw;
  
  float mov;
  float dist0=dist;
  int16_t avoid_obst_angle=0;
  dist=(float)(act_advance[0]+act_advance[1])*0.5f; // in mm;
  mov=dist-dist0;
  run_dist+=fabs(mov);

  act_advance_0[0]=act_advance[0];
  act_advance_0[1]=act_advance[1];
  
  // integrate
  r[0]+=mov*sin(yaw);
  r[1]+=mov*cos(yaw);

  //if(getSensors()) 
  {
    int8_t turn=checkObastacle();
    if(turn!=0) {
      if(turn==8) {
        setTargSpeed(0); // stop
      } else {
        Serial.print(F("Obst avoid turn: ")); Serial.println(turn);
        //adjustTargBearing(turn*10, false); // 10 degrees
        avoid_obst_angle=turn; // 60 degrees
      }
    }
  }
  
  //getActPower();

  if(!dt) return true;
  
  
  {
    //differnitaite
    // LPF
    int16_t raw_speed=(int16_t)(mov/(float)dt*1000.0f);
    int16_t sum_speed=0;
    for(int i=0; i<SPEED_R_SZ-1; i++) {
      speed_r[i]=speed_r[i+1];
      sum_speed+=speed_r[i];
    }
    speed_r[SPEED_R_SZ-1]=raw_speed;
    sum_speed+=raw_speed;
    speed=sum_speed/SPEED_R_SZ;
  }
*/
  return true;
}

uint8_t Controller::getNumSensors() { return nsens;}

int16_t Controller::getX_cm() { return crd[0];}
int16_t Controller::getY_cm() { return crd[1];}
int16_t Controller::getDist_cm() { return dist;}
int16_t Controller::getYaw_grad() { return yaw;}

int16_t *Controller::getSensors() { return sensors;}

/*
//int16_t *Controller::getTargPower() { return targ_pow;}
int16_t Controller::getTargSpeed() { return targ_speed/10;}
//int16_t *Controller::getCurPower() { return cur_pow;}
//float *Controller::getStoredRotRate() { return act_rot_rate;}
int32_t *Controller::getStoredAdvance() { return act_advance;}
int16_t *Controller::getStoredPower() { return act_power;}
//float Controller::getMovement() { return mov;}
//float Controller::getRotation() { return rot;}
float Controller::getDistance() { return dist*0.1f;} //cm
int16_t Controller::getSpeed() { return speed/10;} // cm/s
//float Controller::getAngle() { return angle;}
//float *Controller::getCoords() { return r;}
*/


/*
float Controller::getAVQErr() {
  if(!pid_cnt) return 0;
  return qsum_err/((uint32_t)pid_cnt*pid_cnt);
}
*/

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


uint8_t Controller::_getData() {
  int res=cmgr.Get(REG_ALL);
  
  if(res!=0 || cmgr.GetResultCnt()==0) {
    Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL, Logger::UMP_LOGGER_ALARM, CTL_FAIL_RD, REG_ALL, res, cmgr.GetResultCnt());
    return 0;
  }

  uint16_t cnt=cmgr.GetResultCnt();
  const int16_t *v=cmgr.GetResultVal();

  // yaw[0] X[1] Y[2] Dist[4] Sens[4..]

  yaw=v[0];

  if(cnt<3) return true;  
  crd[0]=v[1];
  crd[1]=v[2];
  if(cnt<4) return true;
  dist=v[3];

  if(cnt<nsens+4) return true;
  for(int i=0; i<nsens; i++) sensors[i]=v[4+i];
  
  return true;
}

bool Controller::setTargPower(float l, float r) {
  /*
  // init steering parameters
  //if(targ_rot_rate[0]==d[0] && targ_rot_rate[1]==d[1]) return true;
  setTargSteering(0);
  err_bearing_p_0=err_bearing_i=0;    
  
  cur_pow[0]=l*M_POW_NORM; cur_pow[1]=r*M_POW_NORM; // temp  
  targ_speed=(l+r)*0.5f*M_SPEED_NORM;
  pid_cnt=0;
    
  Serial.print(F("STP TV=")); Serial.print(targ_speed); Serial.print(F("ADV=")); Serial.print(cur_pow[0]); Serial.print(F("\t ")); Serial.println(cur_pow[1]);

  //raiseFail(CTL_LOG_POW, 1, round(l), round(r), 0, cur_pow[0], cur_pow[1]);

  if(!setPower(cur_pow)) return false;
  return true;
  */
  return false;
}

bool Controller::setTargSteering(int16_t s) {
  /*
  adjustTargBearing(s, true);
  if(!rot_speed && !targ_speed) return startRotate(M_SPEED_NORM);
  
  else
  */
  return true;  
}

bool Controller::setTargBearing(int16_t s) {
  /*
  adjustTargBearing(s, false);
  if(!rot_speed && !targ_speed) return startRotate(M_SPEED_NORM);
  else*/
  return true;  
}

bool Controller::setTargSpeed(int16_t tspeed) {
  /*
  if(targ_speed!=0 && tspeed==0) {
    // stop moving    
    Serial.print(F("Stop TV, AVQE=")); Serial.println(getAVQErr());     
  } else if(rot_speed!=0 && tspeed==0) {
    // stop rotating    
    rot_speed=0;
    Serial.println(F("Stop ROT")); 
  } else if(targ_speed==0 && tspeed!=0) { 
    // start moving
    pid_cnt=0;
    qsum_err=0;
    run_dist=0;
    //Serial.print(F("Start TV=")); Serial.println(tspeed);     
  } 

  //setTargSteering(0);
  adjustTargBearing(0, true);
  err_bearing_p_0=err_bearing_i=0;    
  targ_speed=tspeed*10; //mm
  base_pow=(int32_t)abs(targ_speed)*M_POW_NORM/M_SPEED_NORM; // temp
  delta_pow=0;
   
  int16_t cur_pow[2]={base_pow, base_pow};
  //Serial.print(F("STV TV=")); Serial.print(targ_speed); Serial.print(F("POW=")); Serial.print(cur_pow[0]); Serial.print(F("\t ")); Serial.println(cur_pow[1]);
  if(!setPowerStraight(targ_speed, cur_pow)) return false;
  */
  return true;
}

/*
bool Controller::startRotate(int16_t tspeed) {
  
  float a=targ_bearing-curr_yaw;
  if(a>PI) a-=PI*2.0f;
  else if(a<-PI) a+=PI*2.0f;    
  if(a>0.01) { 
    rot_speed=tspeed;  
    //Serial.println(F("Start ROT >>"));     
  }
  else if(a<-0.01) { 
    rot_speed=-tspeed;
    //Serial.println(F("Start ROT <<")); 
    }
  //else Serial.println(F("No ROT"));
  //if(tspeed==0)  Serial.println(F("Stop RROT"));
  err_bearing_p_0=((curr_yaw-targ_bearing)*180.0f/PI);     
  if(err_bearing_p_0<0) err_bearing_p_0=-err_bearing_p_0; 
  err_bearing_i=0;    
  base_pow=(int32_t)abs(rot_speed)*M_POW_NORM/M_SPEED_NORM; // temp
  delta_pow=0;  
  pid_cnt=0;    
  int16_t cur_pow[2]={base_pow, base_pow};
  //Serial.print(F("STR =")); Serial.print(rot_speed); Serial.print(F("POW=")); Serial.print(cur_pow[0]); Serial.print(F("\t ")); Serial.println(cur_pow[1]);  
  if(!setPowerRotate(rot_speed, cur_pow)) return false;    
  return true;
}
*/

/*

int8_t Controller::checkObastacle() {
  uint8_t obst=0xFF;
  uint8_t schk;
  uint16_t odist;
  int16_t turn=0;
  int16_t mdist=9999;
  //if(targ_speed==0) return 0; // turning. nocheck
  if(targ_speed<=0) return 0; // for fwd only

  // check head sens for straight 
  // else rear for back
  if(targ_speed>0) schk = nsens/4;
  else schk = nsens-1-nsens/4;

  if(this->last_obst==schk) odist=M_CTR_OBST_WARN_OFF_DIST; //debounce 
  else odist=M_CTR_OBST_WARN_ON_DIST; //if we move in other dir or no obst yet, to on_dist
    
  // check 3 readings
  uint8_t prox_count=0, stop_count=0;
  for(uint8_t i=0; i<3; i++) {
    uint8_t iss=schk-1+i;
    if(sensors[iss]>=0)  {
      if(sensors[iss]<=odist*(10+abs(i))/10) prox_count++; // 
      if(sensors[iss]<=M_CTR_OBST_STOP_DIST) { 
        stop_count++;  
        Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL,  Logger::UMP_LOGGER_ALARM, CTL_FAIL_OBST, 0, sensors[schk-1], sensors[schk], sensors[schk+1]);  
        //break;
        }
      if(sensors[iss]<mdist)
        mdist=sensors[iss];   
    } 
  }

  if(stop_count>0) {
    Serial.println(F("Stop!!!")); 
    //Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL,  Logger::UMP_LOGGER_ALARM, CTL_FAIL_OBST, (uint16_t)obst, M_CTR_OBST_STOP_DIST, this->speed, 8);  
    Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL,  Logger::UMP_LOGGER_ALARM, CTL_FAIL_OBST, "OSTOP");  
    return 8;      
  }

  if(prox_count==0) return 0;
  
  obst=schk; 
    
  if(obst !=0xFF) {
    //int16_t left=0, right=0;
    int16_t left=400, right=400;
    uint8_t nhsens=nsens/4;
    uint8_t dir=0;
    for(uint8_t i=0; i<nhsens; i++) {
      int16_t t;
      t=sensors[obst-nhsens+i+1];      
      //if(t>=0) left+=t; 
      if(t>=0 && t<left) left=t; 
      t=sensors[obst+i];
      //if(t>=0) right+=t; 
      if(t>=0 && t<right) right=t; 
    }

    
    if(mdist<=M_CTR_OBST_STOP_DIST) turn=M_CTR_OBST_MAX_TURN;
    else {
      turn=M_CTR_OBST_MAX_TURN-mdist+M_CTR_OBST_STOP_DIST;
      if(turn<0) turn = 0;
    }
    if(left>right) turn=-turn;
    
    //if(this->last_obst!=obst) 
    {
        Serial.print(F("Obstacle at ")); Serial.println(obst); 
        //Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL,  Logger::UMP_LOGGER_ALARM, CTL_FAIL_OBST, (uint16_t)obst, M_CTR_OBST_WARN_ON_DIST, this->speed, turn);        
        Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL,  Logger::UMP_LOGGER_ALARM, CTL_FAIL_OBST, turn, left, right, sensors[schk-1], sensors[schk], sensors[schk+1]);  
      }          
    //Serial.print(F("Obstacle turn ")); Serial.println(turn);   
    } 
  else {    
    if(this->last_obst!=obst) {
      //Serial.println(F("Clear obstacle")); 
      Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL,  Logger::UMP_LOGGER_ALARM, CTL_FAIL_OBST, "CLR");
     }
   }
  this->last_obst=obst;
    
  return turn;
}
*/

/*
bool Controller::getControllerStatus() { 
  bool res = I2Cdev::readBytes(DEV_ID, REG_STATUS, 2, sta); 
  if(!res) raiseFail(CTL_FAIL_RD, REG_STATUS);
  return res;
}
*/

/*
bool Controller::setPowerStraight(int16_t dir, int16_t *p) {
  bool res=false;
  if(dir>=0) res=writeInt16_2(REG_TARG_POW, p[0], p[1]);
  else  res=writeInt16_2(REG_TARG_POW, -p[0], -p[1]);
  if(!res) Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL, Logger::UMP_LOGGER_ALARM, CTL_FAIL_WRT, REG_TARG_POW);
  return res;
}

bool Controller::setPowerRotate(int16_t dir, int16_t *p) {
  bool res=false;
  if(dir>=0) res=writeInt16_2(REG_TARG_POW, p[0], -p[1]);
  else  res=writeInt16_2(REG_TARG_POW, -p[0], p[1]);
  if(!res) Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL, Logger::UMP_LOGGER_ALARM, CTL_FAIL_WRT, REG_TARG_POW);
  return res;
}
*/

/*
bool Controller::setSteering(int16_t s) {
  bool res = writeInt16(REG_STEERING, s); 
  if(!res) Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL, Logger::UMP_LOGGER_ALARM, CTL_FAIL_WRT, REG_STEERING);
  return res;
}

bool Controller::getActRotRate() {
  int16_t tmp[2];
  bool res = readInt16_2(REG_ACT_ROT_RATE, tmp, tmp+1);
  if(!res) {
    Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL, Logger::UMP_LOGGER_ALARM, CTL_FAIL_RD, REG_ACT_ROT_RATE);
    return false;
  }
  act_rot_rate[0]=(float)tmp[0]/V_NORM;
  act_rot_rate[1]=(float)tmp[1]/V_NORM;
  return res;
}
*/
/*
bool Controller::getActAdvance() {
  bool res = readInt32_2(REG_ACT_ADV_ACC, act_advance);  
  if(!res) Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL, Logger::UMP_LOGGER_ALARM, CTL_FAIL_RD, REG_ACT_ADV_ACC);
  return res;
}

bool Controller::getActPower() {
  bool res = readInt16_2(REG_ACT_POW, act_power, act_power+1); 
  if(!res) Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL, Logger::UMP_LOGGER_ALARM, CTL_FAIL_RD, REG_ACT_POW);
  return res;
}
*/
/*
bool Controller::getSensors() {
  ////bool res = readInt16_N(REG_SENSORS_ALL, 10, sensors);
  //bool res = readInt16_N(REG_SENSORS_ALL, nsens, sensors); 
  //if(!res) Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL, Logger::UMP_LOGGER_ALARM, CTL_FAIL_RD, REG_SENSORS_ALL);
  bool res = readInt16_N(REG_SENSORS_1H, nsens/2, sensors_buf); 
  if(!res) {
    Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL, Logger::UMP_LOGGER_ALARM, CTL_FAIL_RD, REG_SENSORS_1H);
    return false;
  }
  delay(25); // temporarily
  res = readInt16_N(REG_SENSORS_2H, nsens/2, sensors_buf+nsens/2); 
  if(!res) {
    Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL, Logger::UMP_LOGGER_ALARM, CTL_FAIL_RD, REG_SENSORS_2H);
    return false;
  }
  // check for validity
  uint8_t zero_count=0;
  for(uint8_t i=0; i<nsens; i++) {
    int16_t s=sensors_buf[i];
    if(s<-2 || s>511) {
      if(!sens_alarmed) {
        Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL, Logger::UMP_LOGGER_ALARM, CTL_FAIL_SENS_CHECK, i, s);
        sens_alarmed=true;
      }
      res=false;
      break;
    }
    if(s==0) zero_count++;
  }
  
  if(zero_count>=2) {
    if(!sens_alarmed) {
        Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL, Logger::UMP_LOGGER_ALARM, CTL_FAIL_SENS_CHECK, 0, 0, zero_count);
        sens_alarmed=true;
      }
    res=false;  
  }

  if(res) {
    for(uint8_t i=0; i<nsens; i++) sensors[i]= sensors_buf[i];
    sens_alarmed=false;
  }
  
  return res;
}
*/



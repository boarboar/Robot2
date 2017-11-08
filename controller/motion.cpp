#include <MapleFreeRTOS821.h>
#include "comm_mgr.h"
#include "log.h"
#include "motor.h"
#include "motion.h"

extern ComLogger xLogger;
extern CommManager xCommMgr;

const int16_t bear_pid_gain_p=8;
const int16_t bear_pid_gain_d=120;
const int16_t bear_pid_gain_i=4;
const int16_t bear_pid_gain_div=10;
const int16_t bear_pid_limit_i=100;

const int16_t speed_pid_gain_p=2;
const int16_t speed_pid_gain_d=4;
const int16_t speed_pid_gain_i=1;
const int16_t speed_pid_gain_div=100;
const int16_t speed_pid_limit_i=10;

const int M_POW_MIN=30; 
const int M_POW_MAX=100;
const int M_POW_NORM=60;
const int M_SPEED_NORM=250;

const int M_ROT_TARG_DEV=1;  //rotation target deviation

void Motion::Init(Motor *m) {
  vSemaphoreCreateBinary(xMotionFree);        
  bReady = false;      
  pxMotor=m;
  fCurrYaw = 0.0f;
  Reset();
  Serial.println("Motion init OK");
}

bool Motion::Acquire() {
  return xSemaphoreTake( xMotionFree, ( portTickType ) 10 ) == pdTRUE;
}


void Motion::Release() {
  xSemaphoreGive( xMotionFree );
}

void Motion::Start() {
  bReady=true;
  //xRunTime=xTaskGetTickCount();     
  xLogger.vAddLogMsg("Motion module ready");      
}

void Motion::Reset() {
  iTargBearing=0;
  iTargSpeed=0;
  iTargRot=0;
  lPIDCnt=0;
  for(int i=0; i<2; i++ ) {
    lAdvance[i]=0;
    lAdvance0[i]=0;
    fCrd[i]=0.0f;    
    run_pow[i]=0;
  }
  fAdvanceTot=0.0f;
  err_bearing_p_0=0;
  err_bearing_i=0;
  err_speed_i=err_speed_p_0=0;
  base_pow=0;
  delta_pow=0;    
  speed = 0;
  collision = 0;
}

bool Motion::HasTask() {
  return iTargSpeed || iTargRot;
}

void Motion::DoCycle(float yaw, int16_t dt, int16_t *vmeas, int16_t nmeas) 
{
  float mov; //mm
  int8_t dir[2];
  pxMotor->DoCycle();
  if(!bReady) return;
  fCurrYaw = yaw;
  
  //uint32_t dt=(xTaskGetTickCount()-xRunTime)*portTICK_PERIOD_MS;
  //xRunTime=xTaskGetTickCount(); 
  if(pxMotor->Acquire()) {      
    pxMotor->GetDirDistMM(lAdvance, dir);
    pxMotor->Release();
  }
  // Advance - with direction
  mov=((int32_t)((lAdvance[0]-lAdvance0[0]))*dir[0]+(int32_t)((lAdvance[1]-lAdvance0[1]))*dir[1])*0.5f;
  fAdvanceTot+=mov;
  lAdvance0[0]=lAdvance[0];
  lAdvance0[1]=lAdvance[1];

  // diff
  // LPF 1:1
  //speed = (speed+(int16_t)((int32_t)mov*1000/dt))/2; //mm_s; 
  // LPF 4:1
  speed = (speed*4+(int16_t)((int32_t)mov*1000/dt))/5; //mm_s; 

  // integrate

  fCrd[0]+=mov*sin(yaw);
  fCrd[1]+=mov*cos(yaw);


  if(iTargSpeed || iTargRot) 
  { // movement
    if(lPIDCnt>0) 
    {
      //int16_t err_bearing_p, err_bearing_d;
      err_bearing_p = (int16_t)(yaw*180.0f/PI-iTargBearing);      
      while(err_bearing_p>180) err_bearing_p-=360;
      while(err_bearing_p<-180) err_bearing_p+=360;   

      if(iTargSpeed) { // straight 
        if(iTargSpeed<0) err_bearing_p=-err_bearing_p;                                             
        err_bearing_i=err_bearing_i+(int32_t)err_bearing_p*dt/100;             
        if(err_bearing_i>bear_pid_limit_i) err_bearing_i=bear_pid_limit_i;
        if(err_bearing_i<-bear_pid_limit_i) err_bearing_i=-bear_pid_limit_i;      
        //if(run_dist>=100) //100mm
        //  qsum_err+=err_bearing_p*err_bearing_p;
      } else { // rot
        if((err_bearing_p<0 && iTargRot<0) || (err_bearing_p>0 && iTargRot>0)) { 
          iTargRot=-iTargRot;          
        }
        if(err_bearing_p<0) err_bearing_p=-err_bearing_p;        
        if(err_bearing_p<M_ROT_TARG_DEV) { 
          iTargRot=0;          
        }        
      }

      err_bearing_d=err_bearing_p-err_bearing_p_0;      
      if(err_bearing_d>180) err_bearing_d-=360;
      else if(err_bearing_d<-180) err_bearing_d+=360;
      err_bearing_d=(int32_t)err_bearing_d*100/dt;
        
      err_bearing_p_0=err_bearing_p;
      // use 32 bit math?
      delta_pow=-(int16_t)((err_bearing_p*bear_pid_gain_p+err_bearing_d*bear_pid_gain_d+err_bearing_i*bear_pid_gain_i)/bear_pid_gain_div);

      int16_t cur_pow[2];
      if(iTargSpeed) {              
        // do collision avoidance here
        collision = DoCollisionCheck(speed, vmeas, nmeas);
        switch(collision) {
          case 1 :
            Move(0);
            xCommMgr.vAddAlarm(CommManager::CM_ALARM, CommManager::CM_MODULE_CTL, CTL_FAIL_OBST, 0); 
            return;
            break; // stop
          case 2 : break; // slow-down
          case 3 : 
            delta_pow = -base_pow/2;
            break; // turn left
          case 4 : 
            delta_pow = base_pow/2;
            break; // turn right;
          default:;  // no action
        }  

        cur_pow[0]=base_pow+delta_pow;
        cur_pow[1]=base_pow-delta_pow;     
        
        if(lPIDCnt>25) { // 500 ms
          // speed PID; 
          err_speed_p = abs(speed)-abs(iTargSpeed);        
          err_speed_d=err_speed_p-err_speed_p_0;
          err_speed_d=(int32_t)err_speed_d*100/dt;
          err_speed_i=err_speed_i+(int32_t)err_speed_p*dt/100;             
          if(err_speed_i>speed_pid_limit_i) err_speed_i=speed_pid_limit_i;
          if(err_speed_i<-speed_pid_limit_i) err_speed_i=-speed_pid_limit_i;    
          delta_pow=-(int16_t)((err_speed_p*speed_pid_gain_p+err_speed_d*speed_pid_gain_d+err_speed_i*speed_pid_gain_i)/speed_pid_gain_div);              
          cur_pow[0]+=delta_pow;
          cur_pow[1]+=delta_pow;
        }
      } else {
        cur_pow[0]=base_pow-delta_pow;
        cur_pow[1]=base_pow-delta_pow;
      }
      
      for(int i=0; i<2; i++) {
        if(cur_pow[i]<M_POW_MIN) cur_pow[i]=M_POW_MIN; 
        if(cur_pow[i]>M_POW_MAX) cur_pow[i]=M_POW_MAX; 
      // maybe a better idea would be to make limits proportional to the target?
      }
      if(iTargSpeed){
        //xLogger.vAddLogMsg("MV YTB,P*:", (int16_t)(yaw*180.0f/PI), iTargBearing, cur_pow[0], cur_pow[1]);
        SetPowerStraight(iTargSpeed, cur_pow);       
      }
      else {
        if(!iTargRot) StartRotate(0); // stop rotate
        else SetPowerRotate(iTargRot, cur_pow);              
      }     
    }
    lPIDCnt++;
  }
}

void Motion::Move(int16_t tspeed)
{
  if(!bReady) return;
  //xLogger.vAddLogMsg("MOV V:", tspeed);
  if(iTargSpeed!=0 && tspeed==0) {
    // stop moving    
    //Serial.print(F("Stop TV, AVQE=")); Serial.println(getAVQErr());     
  } else if(iTargRot!=0 && tspeed==0) {
    // stop rotating    
    iTargRot=0;
    //Serial.println(F("Stop ROT")); 
  } else if(iTargSpeed!=0 && tspeed!=0) { 
    // start moving
    lPIDCnt=0;
    //qsum_err=0;
    //run_dist=0;
    //Serial.print(F("Start TV=")); Serial.println(tspeed);     
  } 


  AdjustTargBearing(0, true);
  err_bearing_p_0=err_bearing_i=0;  
  err_speed_i=err_speed_p_0=0;  
  iTargSpeed=tspeed*10; //mm
  base_pow=(int32_t)abs(iTargSpeed)*M_POW_NORM/M_SPEED_NORM; // temp
  delta_pow=0;
  collision=0;
  int16_t cur_pow[2]={base_pow, base_pow};
  //cur_pow[0]=cur_pow[1]=base_pow;
  //xLogger.vAddLogMsg("MV TVB,P*:", iTargSpeed, iTargBearing, cur_pow[0], cur_pow[1]);
  SetPowerStraight(iTargSpeed, cur_pow);
  return;
}

void Motion::Steer(int16_t angle)
{
  if(!bReady) return;
  //xLogger.vAddLogMsg("ROT A:", angle);
  AdjustTargBearing(angle, true);
  if(!iTargRot && !iTargSpeed) return StartRotate(M_SPEED_NORM);
}

void Motion::MoveBearing(int16_t angle)
{
  if(!bReady) return;
  //xLogger.vAddLogMsg("BER A:", angle);
  AdjustTargBearing(angle, false);
  if(!iTargRot && !iTargSpeed) return StartRotate(M_SPEED_NORM);
}

void Motion::AdjustTargBearing(int16_t s, bool absolute) {
  iTargBearing = s;
  if(absolute) iTargBearing+=fCurrYaw*180.0/PI;
  if(iTargBearing>180) iTargBearing-=360;
  else if(iTargBearing<-180) iTargBearing+=360;    
}


void Motion::StartRotate(int16_t tspeed) {
  int16_t a = iTargBearing-fCurrYaw*180.0/PI;
  if(a>180) a-=360;
  else if(a<-180) a+=360;    
  
  if(a>1) { 
    iTargRot=tspeed;  
    //Serial.println(F("Start ROT >>"));     
  }
  else if(a<-1) { 
    iTargRot=-tspeed;
    //Serial.println(F("Start ROT <<")); 
    }

  //xLogger.vAddLogMsg("R TBYR:", iTargBearing, (int16_t)(fCurrYaw*180.0/PI), iTargRot);
  
  err_bearing_p_0=-a;     
  if(err_bearing_p_0<0) err_bearing_p_0=-err_bearing_p_0; 
  err_bearing_i=0;  
  err_speed_i=err_speed_p_0=0;  
  base_pow=(int32_t)abs(iTargRot)*M_POW_NORM/M_SPEED_NORM; // temp
  delta_pow=0;  
  lPIDCnt=0;    
  collision=0;
  int16_t cur_pow[2]={base_pow, base_pow};
  //Serial.print(F("STR =")); Serial.print(rot_speed); Serial.print(F("POW=")); Serial.print(cur_pow[0]); Serial.print(F("\t ")); Serial.println(cur_pow[1]);  
  SetPowerRotate(iTargRot, cur_pow);    
}

void Motion::SetPowerStraight(int16_t dir, int16_t *p) {
  //xLogger.vAddLogMsg("POWS:", dir, p[0], p[1]);
  if(dir>0) SetMotors(p[0], p[1]);
  else SetMotors(-p[0], -p[1]);  
}

void Motion::SetPowerRotate(int16_t dir, int16_t *p) {
  //xLogger.vAddLogMsg("POWR:", dir, p[0], p[1]);
  if(dir>0) SetMotors(p[0], -p[1]);
  else SetMotors(-p[0], p[1]);  
}


void Motion::SetMotors(int16_t dp1, int16_t dp2) // in %%
{
  if(!bReady) return;
  //xLogger.vAddLogMsg("SETM:", dp1, dp2, 0);
  if(dp1<-100) dp1=-100; 
  else if(dp1>100) dp1=100;
  if(dp2<-100) dp2=-100; 
  else if(dp2>100) dp2=100;
  if(pxMotor->Acquire()) {
    pxMotor->SetMotors((int8_t)dp1, (int8_t)dp2);   
    run_pow[0]=dp1;  
    run_pow[1]=dp2;  
    pxMotor->Release();
  }
}

int16_t Motion::DoCollisionCheck(int16_t speed, int16_t *vmeas, int16_t nmeas, int16_t *act_val) 
{
  const int STOP_DIST=40;
  const int AVOID_DIST=60;
  if(!vmeas || nmeas<10) return 0;
  if(speed>0 && (vmeas[2]>0 && vmeas[2]<STOP_DIST)) return 1; //stop 
  if(speed<0 && (vmeas[7]>0 && vmeas[7]<STOP_DIST)) return 1; //stop

  if(speed>0 && (vmeas[2]>0 && vmeas[2]<AVOID_DIST) && (vmeas[1]>0 && vmeas[1]<AVOID_DIST)) {
    return 4; //right
  }  
  if(speed>0 && (vmeas[2]>0 && vmeas[2]<AVOID_DIST) && (vmeas[3]>0 && vmeas[3]<AVOID_DIST)) {
    return 3; //left
  }  
  if(speed>0 && (vmeas[2]>0 && vmeas[2]<AVOID_DIST)) {
    return 4; //right
  }  
  if(speed>0 && (vmeas[1]>0 && vmeas[1]<AVOID_DIST)) {
    return 4; //right
  }  
  if(speed>0 && (vmeas[3]>0 && vmeas[3]<AVOID_DIST)) {
    return 3; //left
  }  
  return 0;
}

/*
void Motion::GetAdvance(uint32_t *dst_dist) 
{
   dst_dist[0]=lAdvance[0];
   dst_dist[1]=lAdvance[1];        
}
*/

void Motion::GetCrdCm(int16_t *crd) 
{
   crd[0]=(int16_t)(fCrd[0]/10.0f);
   crd[1]=(int16_t)(fCrd[1]/10.0f);
}

int16_t Motion::GetAdvanceCm() {  
  return (int16_t)(fAdvanceTot/10.0f);
}

int16_t Motion::GetSpeedCmpS() {
  return speed/10;
}

void Motion::GetPower(int16_t *pow) 
{
  pow[0]=run_pow[0];
  pow[1]=run_pow[1];
}

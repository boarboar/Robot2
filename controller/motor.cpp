#include <MapleFreeRTOS821.h>
#include "motor.h"
#include "log.h"

#define   V_NORM 10000
#define   V_NORM_MAX 30000
#define   V_NORM_PI2 62832L
#define   WHEEL_CHGSTATES 40
#define   WHEEL_RAD_MM   33 // measured 32
#define   CHGST_TO_MM(CNT)  ((uint64_t)(CNT)*V_NORM_PI2*WHEEL_RAD_MM/WHEEL_CHGSTATES/V_NORM)
#define   WHEEL_MAX_CHGST_TIME  5 //5 ms = 200 chst sec = 1 m/s

extern ComLogger xLogger;

static Motor *motor_instance=NULL;  

static void encodeInterrupt_1() { 
  //static portBASE_TYPE xHigherPriorityTaskWoken1;
  //xHigherPriorityTaskWoken1 = pdFALSE;
  if(motor_instance) motor_instance->encInterrupt(0); 
  //portEND_SWITCHING_ISR( xHigherPriorityTaskWoken1 );
  }
  
static void encodeInterrupt_2() { 
  //static portBASE_TYPE xHigherPriorityTaskWoken2;
  //xHigherPriorityTaskWoken2 = pdFALSE;
  if(motor_instance) motor_instance->encInterrupt(1); 
  //portEND_SWITCHING_ISR( xHigherPriorityTaskWoken2 );
  }

void Motor::Init(int m_1_1_pin, int m_1_2_pin, int m_1_enab_pin, int m_1_enc_pin, int m_2_1_pin, int m_2_2_pin, int m_2_enab_pin, int m_2_enc_pin) {
    vSemaphoreCreateBinary(xMotorFree);    
    
    m[0].pin_1=m_1_1_pin;
    m[0].pin_2=m_1_2_pin;
    m[0].pin_enab=m_1_enab_pin;
    m[0].pin_enc=m_1_enc_pin;

    m[1].pin_1=m_2_1_pin;
    m[1].pin_2=m_2_2_pin;
    m[1].pin_enab=m_2_enab_pin;
    m[1].pin_enc=m_2_enc_pin;
    
    for(int i=0; i<2; i++) {
      pinMode(m[i].pin_1, OUTPUT);     
      pinMode(m[i].pin_2, OUTPUT);
      pinMode(m[i].pin_enab, PWM);
      //pinMode(m[i].pin_enc, INPUT_PULLUP);
      pinMode(m[i].pin_enc, INPUT);
      m[i].enc_count=0;
      m[i].enc_accum=0;
      //m[i].dist=0;
      m[i].enc_prev_st=digitalRead(m[i].pin_enc);
      m[i].dir=0;
      m[i].power=0;
      m[i].xLastWakeTime=0;
      Low_Drive(i);
    }
    
    motor_instance = this;

    attachInterrupt(m[0].pin_enc, encodeInterrupt_1, RISING);
    attachInterrupt(m[1].pin_enc, encodeInterrupt_2, RISING);    
    //running = false;      
    Serial.println("Motor OK");
}


bool Motor::Acquire() {
  return xSemaphoreTake( xMotorFree, ( portTickType ) 10 ) == pdTRUE;
}


void Motor::Release() {
  xSemaphoreGive( xMotorFree );
}

void Motor::DoCycle() {
//  if ( xSemaphoreTake( xMotorFree, ( portTickType ) 10 ) == pdTRUE )
    {
      for(int i=0; i<2; i++) {
        m[i].enc_accum+=m[i].enc_count; // should be locked internally and done by intermediate var
        m[i].enc_count=0;       
      }
//      xSemaphoreGive( xMotorFree );
    }
}

// 0-100 
void Motor::SetMotors(int8_t dp1, int8_t dp2) {

  //if(!running) return;
  
  int8_t dp[2];
  int8_t d[2];
  uint16_t p[2];  
  int i;
  dp[0]=dp1;
  dp[1]=dp2;
  
  for(i=0; i<2; i++) {
    if(dp[i]<-100 || dp[i]>100) return;    
    if(dp[i]==0) {d[i]=0; p[i]=0; }
    else if(dp[i]>0) {d[i]=1; p[i]=dp[i]; }
    else {d[i]=-1; p[i]=-dp[i]; }
    p[i]*=655;
    if(m[i].dir!=d[i] || m[i].power!=p[i]) {
       m[i].dir=d[i]; 
       m[i].power=p[i];
       Low_Drive(i); 
       //xLogger.vAddLogMsg("M:", i, d[i], p[i]);
     }

  }      
  
}

bool Motor::GetEncDist(uint16_t *dst_enc, uint32_t *dst_dist) {  
  for(int i=0; i<2; i++) {
     if(dst_enc) dst_enc[i]=m[i].enc_accum;
     if(dst_dist) dst_dist[i]= (CHGST_TO_MM(m[i].enc_accum));
    }
  return true;
}


void Motor::encInterrupt(uint16_t i)  { 
  static TickType_t xLastWakeTime[2];
  static uint8_t v[2];
  xLastWakeTime[i]=xTaskGetTickCountFromISR(); 
  //uint8_t v=digitalRead(m[i].pin_enc);
  v[i]=digitalRead(m[i].pin_enc);
  // separate semaphores

/*
  xSemaphoreTakeFromISR
      (
        SemaphoreHandle_t xSemaphore,
        signed BaseType_t *pxHigherPriorityTaskWoken
      )
  */
      
  //if ( xSemaphoreTake( xMotorFree, ( portTickType ) 10 ) == pdTRUE )
    {
      // add debounces
      if (v[i]!=m[i].enc_prev_st && (xLastWakeTime[i] >= m[i].xLastWakeTime+WHEEL_MAX_CHGST_TIME)) 
      {
        m[i].enc_prev_st=v[i]; 
        m[i].enc_count++;
        m[i].xLastWakeTime=xLastWakeTime[i];
      }     
 //     xSemaphoreGive( xMotorFree );
    }
 }

void Motor::Low_Drive(uint8_t i) 
{
  if(m[i].dir==0 || m[i].power==0) {
    pwmWrite(m[i].pin_enab, 0);
    digitalWrite(m[i].pin_1, LOW); digitalWrite(m[i].pin_2, LOW); 
    return;
  }
  else if(m[i].dir==1) {
    digitalWrite(m[i].pin_1, LOW); digitalWrite(m[i].pin_2, HIGH); 
  }
  else {
    digitalWrite(m[i].pin_1, HIGH); digitalWrite(m[i].pin_2, LOW); 
  } 
  pwmWrite(m[i].pin_enab, m[i].power);
} 


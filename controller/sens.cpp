#include <MapleFreeRTOS821.h>
#include "sens.h"
#include "log.h"

#define SERVO_STEP    36
#define SERVO_ZERO_SHIFT    -3
//#define SERVO_ZERO_SHIFT    10
//#define SERVO_CORR    2
#define SERVO_CORR    0
#define SERVO_WAIT    150
//#define SERVO_WAIT    2000 //test

#define USENS_BASE    7

#define USENS_DIVISOR 58
#define PING_OVERHEAD 5
#define PING_WAIT_MS  40 

#define USENS_BASE_D    (float)USENS_BASE*USENS_DIVISOR

// TEST DIST 245

extern ComLogger xLogger;

static Sensor *sensor_instance=NULL;  

static void echoInterrupt_0() {
  if(sensor_instance) sensor_instance->echoInterrupt(0); 
}

static void echoInterrupt_1() {
  if(sensor_instance) sensor_instance->echoInterrupt(1); 
}

void Sensor::echoInterrupt(uint16_t i) {

  static BaseType_t xHigherPriorityTaskWoken;
  static uint16_t v;
  
  if(xTaskToNotify == NULL || wait_sens!=i) return;
  xHigherPriorityTaskWoken = pdFALSE;
  v=digitalRead(this->sens_in_pin[i]);
  if(v==HIGH) { //raise = start echo
    this->t0[i]=micros();
    this->di[i]=0;
    this->sens_state[i]=1;
  } else { // FALL = stop echo
    if(this->sens_state[i]==1) {
      this->sens_state[i]=2;
      this->di[i]=micros()-this->t0[i]-PING_OVERHEAD;
      /* Notify the task that the transmission is complete. */
      vTaskNotifyGiveFromISR( xTaskToNotify, &xHigherPriorityTaskWoken );
      /* There are no transmissions in progress, so no tasks to notify. */
    }
    xTaskToNotify = NULL;
  }
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void Sensor::Init(int servo_pin, int sens_in_pin_0, int sens_out_pin_0, int sens_in_pin_1, int sens_out_pin_1) {
    uint8_t i;
    this->sens_in_pin[0]=sens_in_pin_0;
    this->sens_out_pin[0]=sens_out_pin_0;
    this->sens_in_pin[1]=sens_in_pin_1;
    this->sens_out_pin[1]=sens_out_pin_1;
    vSemaphoreCreateBinary(xSensFree);    
    xServo.attach(servo_pin);
    xServo.write(90-SERVO_ZERO_SHIFT);
    for(i=0; i<2; i++) {
      digitalWrite(sens_in_pin[i], LOW);
      pinMode(sens_out_pin[i], OUTPUT);           
      pinMode(sens_in_pin[i], INPUT); 
      t0[i]=0;
      sens_state[i]=0;
    }
    for(i=0; i<M_SENS_N; i++) 
      value[i]=-2;
    for(i=0; i<=SERVO_NSTEPS; i++)   
      fCosines[i] = cos((float)i*SERVO_STEP*PI/180);
    sservo_pos=0; //90
    sservo_step=1; 
    sensor_instance = this;
    xTaskToNotify = NULL;
    wait_sens=100;
    attachInterrupt(sens_in_pin[0], echoInterrupt_0, CHANGE);
    attachInterrupt(sens_in_pin[1], echoInterrupt_1, CHANGE);    
    running = false;
    Serial.println("Sensor OK");
}

bool Sensor::Acquire() {
  return xSemaphoreTake( xSensFree, ( portTickType ) 50 ) == pdTRUE;
}


void Sensor::Release() {
  xSemaphoreGive( xSensFree );
}

void Sensor::Start() {
    xLogger.vAddLogMsg("Sens module run.");
    running=true;
}

/*
N=6:  (NSTEPS=1)
      Sens_s |
Servo_p   \  | 0  | 1  |
------------ +----+----+  
  -1         | 2  | 5  |  
------------ +----+----+  
   0         | 1  | 4  |  
------------ +----+----+  
   1         | 0  | 3  |  
------------ +----+----+  

N=10:  (NSTEPS=2)
      Sens_s |
Servo_p   \  | 0  | 1  |
------------ +----+----+  
  -2         | 4  | 9  |  
------------ +----+----+  
  -1         | 3  | 8  |  
------------ +----+----+  
   0         | 2  | 7  |  
------------ +----+----+  
   1         | 1  | 6  |  
------------ +----+----+  
   2         | 0  | 5  |  
------------ +----+----+    
*/

void Sensor::DoCycle() {
    if(!running) return;    
    if((sservo_step>0 && sservo_pos>=SERVO_NSTEPS) || (sservo_step<0 && sservo_pos<=-SERVO_NSTEPS)) sservo_step=-sservo_step;
    sservo_pos+=sservo_step;
    int16_t sservo_angle=90-SERVO_ZERO_SHIFT+sservo_pos*SERVO_STEP+abs(sservo_pos)*SERVO_CORR;
    //vTaskDelay(1);
    taskENTER_CRITICAL();
    xServo.write(sservo_angle);
    taskEXIT_CRITICAL();
    vTaskDelay(SERVO_WAIT);
    for(uint16_t sens_step=0; sens_step<2; sens_step++) {  
      int8_t current_sens=-sservo_pos+SERVO_NSTEPS+sens_step*(SERVO_NSTEPS*2+1); 
      int16_t pin_in=sens_in_pin[sens_step];
      int16_t pin_out=sens_out_pin[sens_step];
      di[sens_step]=0;
      sens_state[sens_step]=0;
      wait_sens=sens_step;
      xTaskToNotify = xTaskGetCurrentTaskHandle();      
      taskENTER_CRITICAL();
      digitalWrite(pin_out, LOW);
      delayMicroseconds(2);
      digitalWrite(pin_out, HIGH);
      delayMicroseconds(10);
      digitalWrite(pin_out, LOW);
      taskEXIT_CRITICAL();
      int32_t d=0;
   
      if(digitalRead(pin_in)==HIGH) {
        // not completed
        xLogger.vAddLogMsg("SENC UNC==", sens_step); 
      } else {             
        uint32_t ulNotificationValue = ulTaskNotifyTake( pdTRUE, pdMS_TO_TICKS( PING_WAIT_MS ) );
        if( ulNotificationValue == 1 && sens_state[sens_step]==2 && di[sens_step]<1000L*PING_WAIT_MS) 
            d=di[sens_step];
        else d=0;              
      }
      xTaskToNotify = NULL;

      if(d>0 && USENS_BASE!=0) {  // base shift
        if(sens_step==0) { //fwd
          if(sservo_pos==0) d += USENS_BASE_D;
          else d=sqrt(2.0f*d*USENS_BASE_D*fCosines[abs(sservo_pos)]+(float)d*d+USENS_BASE_D*USENS_BASE_D);
        } else { //bck
          if(sservo_pos==0) d -= USENS_BASE_D;
          else d=sqrt(-2.0f*d*USENS_BASE_D*fCosines[abs(sservo_pos)]+(float)d*d+USENS_BASE_D*USENS_BASE_D);
          if(d<1) d=1;
        }
      }

      if(Acquire()) 
      {
        // + BASE is wrong;   
        // actual shoud be 
        // sqrt(dist*dist+b*b+2*b*dist*cos(a)**2) = 
        // sqrt((dist+b)**2-2*b*dist*sin(a)**2) = 
        //if(d>0) value[current_sens]=(int16_t)(d/USENS_DIVISOR+USENS_BASE);
        //else value[current_sens] = -2;
        if(d>0) value[current_sens]=(int16_t)(d/USENS_DIVISOR);
        
        else value[current_sens] = -2;
        Release();  
      }
      vTaskDelay(1);
    }
}

int16_t Sensor::GetNMeas() {
  return M_SENS_N;
}

int16_t Sensor::Get() {
  return value[0];
}

void Sensor::Get(int16_t *v, int16_t n) {
  if(n>M_SENS_N) n=M_SENS_N;
  for(int i=0; i<n; i++)
    v[i]=value[i];
}


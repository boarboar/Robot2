
#include <MapleFreeRTOS821.h>
#include <Wire.h>
#include "comm_mgr.h"
#include "log.h"
#include "mpu.h"
#include "sens.h"
#include "motor.h"
#include "motion.h"

/*

 ENC1[3]: Vcc(5v), Gnd, In(5vT)
 ENC2[3]: Vcc(5v), Gnd, In(5vT)
 
 SENS[7]: Vcc(5v), Gnd, UI1(5vT), UO1(5vT), UI2(5vT), UO2(5vT), Srv 
 MOTO[8]: Vcc(5v), Gnd, In1, In2, In3, In4, EN1, EN2
 
 */

#define BOARD_LED_PIN PC13

// Serial MCU comm
#define SER3_TX_PIN PB10
#define SER3_RX_PIN PB11

// I2C ports fixed
#define SCL_PIN PB6
#define SDA_PIN PB7

// PWM ports (check!)
#define SERVO_1_PIN PA8 
#define MOTOR_EN_1_PIN PA2 
#define MOTOR_EN_2_PIN PA1 

// GPIO
#define MOTOR_OUT_1_1_PIN PA6
#define MOTOR_OUT_1_2_PIN PA5
#define MOTOR_OUT_2_1_PIN PA3
#define MOTOR_OUT_2_2_PIN PA2

// 5v Tolerant ports (check!)
#define MOTOR_ENC_1_PIN  PB9
#define MOTOR_ENC_2_PIN  PB8

#define US_IN_1_PIN   PB12
#define US_OUT_1_PIN  PB13

#define US_IN_2_PIN   PB14
#define US_OUT_2_PIN  PB15


CommManager xCommMgr;
ComLogger xLogger;
Sensor xSensor;
Motor xMotor;
Motion xMotion;

static void vSerialOutTask(void *pvParameters) {
    Serial.println("Serial Out Task started.");
    for (;;) {
       xLogger.Process();
       vTaskDelay(20);
    }
}

static void vCommTask(void *pvParameters) {
    xLogger.vAddLogMsg("Comm Task started.");
    for (;;) {
        vTaskDelay(10);        
        if(xCommMgr.ReadSerialCommand()) {       
          digitalWrite(BOARD_LED_PIN, LOW);        
          xLogger.vAddLogMsg(xCommMgr.GetBuffer());    
          xCommMgr.ProcessCommand();
          xLogger.vAddLogMsg(xCommMgr.GetDbgBuffer());  // debug        
          xCommMgr.Complete();
          //xCommMgr.Respond(xCmd.GetResponce());          
          digitalWrite(BOARD_LED_PIN, HIGH);
        }        
    }
}

static void vLazyTask(void *pvParameters) {
    float yaw; 
    //nt16_t val;
    uint16_t enc[2];
    uint8_t needReset=0;
    int8_t imu_status=0;
    int16_t vcnt=0;
    int16_t val[16];
    xLogger.vAddLogMsg("Lazy Task started.");
    for (;;) {       
        vTaskDelay(2000);  
        yaw=0.f;              
        if(MpuDrv::Mpu.Acquire()) {
          needReset=MpuDrv::Mpu.isNeedReset();
          MpuDrv::Mpu.copyAlarms();     
          yaw=MpuDrv::Mpu.getYaw(); 
          imu_status=MpuDrv::Mpu.getStatus(); 
          MpuDrv::Mpu.Release();
        } 
        MpuDrv::Mpu.flushAlarms();   
        if(imu_status==MpuDrv::ST_READY) {  
            //int16_t vcnt=0;
            //int16_t val[16];
            vcnt=0;
            if(xSensor.Acquire()) {
              vcnt=xSensor.GetNMeas();
              xSensor.Get(val, vcnt);
              xSensor.Release();  
            }
            if(vcnt>0) {
               xLogger.vAddLogMsg("S[1-3]", val[0], val[1], val[2]);           
               //xLogger.vAddLogMsg("S[4-6]", val[3], val[4], val[5]);           
               xLogger.vAddLogMsg("S[7-9]", val[6], val[7], val[8]);            
               //xLogger.vAddLogMsg("S[10]", val[9]);          
            }
            //val = yaw*180.0/PI;
            xLogger.vAddLogMsg("Y", (int16_t)yaw*180.0/PI);    
        }
        /*      
        if(xSensor.Acquire()) {
          xLogger.vAddLogMsg("Y", val, "S", xSensor.Get());           // BAD!!!
          xSensor.Release();  
        }
        */
        if (xMotor.Acquire()) { 
          xMotor.GetEncDist(enc, NULL);
          xMotor.Release();
          xLogger.vAddLogMsg("E1", enc[0], "E2", enc[1]);           
        }
        

       if(needReset) {
          xCommMgr.vAddAlarm(CommManager::CM_ALARM, CommManager::CM_MODULE_SYS, 101);
          xLogger.vAddLogMsg("IMURST");           
          vTaskDelay(2000);                
          nvic_sys_reset();          
       }
    }
}

static void vIMU_Task(void *pvParameters) {
    int16_t mpu_res=0;    
    xLogger.vAddLogMsg("IMU Task started.");
    for (;;) { 
      vTaskDelay(3); 
      if(MpuDrv::Mpu.Acquire()) {
        mpu_res = MpuDrv::Mpu.cycle_dt();       
        MpuDrv::Mpu.Release();
      } else continue;
      if(mpu_res==2) {
        // IMU settled
        xLogger.vAddLogMsg("Activate motion!");
        if(xSensor.Acquire()) {
          xSensor.Start();    
          xSensor.Release();
        }
        if(xMotion.Acquire()) {
          xMotion.Start();             
          xMotion.Release();
        }
      }
    }
}

static void vSensorTask(void *pvParameters) {
    //xSensor.Start();
    xLogger.vAddLogMsg("Sensor Task started.");
    for (;;) {
        vTaskDelay(10);
        xSensor.DoCycle();     
    }
}


static void vMotionTask(void *pvParameters) {
    xLogger.vAddLogMsg("Motion Task started.");    
    for (;;) { 
      vTaskDelay(100); // should be less
      float yaw=0;
      if(MpuDrv::Mpu.Acquire()) {
        MpuDrv::Mpu.process();           
        yaw=MpuDrv::Mpu.getYaw();
        MpuDrv::Mpu.Release();
      }
      else continue;
      if(xMotion.Acquire()) {
         xMotion.DoCycle(yaw);
         xMotion.Release();
      } 
    }
}


void setup() {
  
    digitalWrite(BOARD_LED_PIN, LOW);
    pinMode(BOARD_LED_PIN, OUTPUT);
    
    delay(5000);
    //disableDebugPorts(); // disable JTAG debug, enable PB3,4 for usage
      
    Serial.begin(115200); 
    Serial.print("Tick = ");
    Serial.println(portTICK_PERIOD_MS);
    
    xLogger.Init();
    xCommMgr.Init(115200);
    xSensor.Init(SERVO_1_PIN, US_IN_1_PIN, US_OUT_1_PIN, US_IN_2_PIN, US_OUT_2_PIN); 
    xMotor.Init(MOTOR_OUT_1_1_PIN, MOTOR_OUT_1_2_PIN, MOTOR_EN_1_PIN, MOTOR_ENC_1_PIN,
        MOTOR_OUT_2_1_PIN, MOTOR_OUT_2_2_PIN, MOTOR_EN_2_PIN, MOTOR_ENC_2_PIN); 
    xMotion.Init(&xMotor);
      
    Serial.println("Init Wire...");
    //Wire.begin(SCL_PIN, SDA_PIN);
    Wire.begin();
    MpuDrv::Mpu.init();
     
    Serial.println("Starting...");
    digitalWrite(BOARD_LED_PIN, HIGH);
       
    xTaskCreate(vSerialOutTask,
                "TaskSO",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 1, // low
                NULL);

    xTaskCreate(vCommTask,
                "TaskCom",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 1, // low
                NULL);
                
    xTaskCreate(vSensorTask,
                "TaskSens",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2, // max
                NULL);
                  
    xTaskCreate(vIMU_Task,
                "TaskIMU",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 3, // max
                NULL);

    xTaskCreate(vMotionTask,
                "TaskMotion",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2, // med
                NULL);
          
    xTaskCreate(vLazyTask,
                "TaskLazy",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 0, // min
                NULL);
                            
    vTaskStartScheduler();
}

void loop() {
    // Insert background code here
}



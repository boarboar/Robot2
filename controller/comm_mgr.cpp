#include <MapleFreeRTOS821.h>
#include "log.h"
#include "mpu.h"
#include "sens.h"
#include "motor.h"
#include "motion.h"
#include "comm_mgr.h"

extern ComLogger xLogger;
extern Sensor xSensor;
extern Motor xMotor;
extern Motion xMotion;
 
void CommManager::Init(uint32_t comm_speed) {
  vSemaphoreCreateBinary(xAlarmFree);
  xAlarmQueue = xQueueCreate( ALR_Q_SZ, sizeof( struct Alarm ) );

  if( xAlarmQueue == NULL )
    {
        /* Queue was not created and must not be used. */
        Serial.println("Couldn't create AQ");
        return;
    }

  txAlarm.uAlarmID=0;
    
  bytes = 0;
  buf[bytes] = 0;
  Serial3.begin(comm_speed); 
  while (Serial3.available()) Serial3.read();  // eat garbage
}

// read serial data into buffer
boolean CommManager::ReadSerialCommand()
{
  while (Serial3.available() && bytes < CM_BUF_SIZE)
  {
    buf[bytes] = Serial3.read();
    if (buf[bytes] == 10 || buf[bytes] == 13)
    {
      if (bytes > 0) { 
        buf[bytes]=0;         
        return true; 
      } 
      return false; // skip 10 or 13 left         
    }
    bytes++;
  }
  if(bytes>=CM_BUF_SIZE) { 
    bytes=0; //overflow, probably caused hang up at start...    
    buf[bytes]=0; 
    //return false;     
  }
  return false;
}

// process command
// cmd:= setcmd | getcmd
// getcmd := verb [,] reg
// setcmd := verb [,] reg [,] val
// val := int | list
// list := '[' int [,] ... ']'
boolean CommManager::ProcessCommand()
{
  RetCodes rc=CM_RC_OK;
  *msgdbg=0;  
  // check crc
  uint8_t crc=CRC();
  if(buf[pos]=='%') {
    pos++;
    while(isspace(buf[pos])) pos++;
    uint8_t mcrc=(uint8_t)ReadInt();    
    if(crc!=mcrc) {
      Respond(CM_RC_FAIL_CRC, 0, "CRC FAIL");
      return false;    
    } else strcpy(msgdbg, "% ");
  }
  
  pos=0;
  verb=buf[0];
  switch(verb) {
    case 'g': 
      verb='G';
    case 'G': // get cmd 
      strcat(msgdbg, "G");
      break;
    case 's':  
      verb='S'; 
    case 'S': // get cmd 
      strcat(msgdbg, "S");
      break;
    default:  
      Respond(CM_RC_FAIL_BADCMD, 0, "BAD CMD");
      return false;
  }

  while(isspace(buf[pos]) || buf[pos]==',' ) pos++;     
  
  pos=1;
  reg=ReadInt();
  if(reg==0) {    
    Respond(CM_RC_FAIL_BADREG, 0, "BAD REG");    
    return false;
  }
  
  strcat(msgdbg, " ");
  itoa_cat(reg, msgdbg);

  vcnt=0;
  if(verb=='G') {    
    switch(reg) {
      case REG_ID:
        vcnt=1;
        val[0]=CM_ID;
        break;
      case REG_N_SENS:
        vcnt=1;
        val[0]=xSensor.GetNMeas();
        break;        
      case REG_STATUS:
        vcnt=1;
        if(MpuDrv::Mpu.Acquire()) {
          val[0]=MpuDrv::Mpu.getStatus();
          MpuDrv::Mpu.Release();
        }
        break;  
      case REG_ENC:
        vcnt=2;
        if (xMotor.GetEncDistMM((uint16_t*)val, NULL)) { //BAD !!!
          ;
        }
        break;    
      case REG_SENS:
        if(xSensor.Acquire()) {
          vcnt=xSensor.GetNMeas();
          xSensor.Get(val, vcnt);
          xSensor.Release();
        }
        break;
      case REG_ALL:
         // St[0] yaw[1] X[2] Y[3] Dist[4] Sens[5..14]
        for(uint16_t i=0; i<CM_NVAL; i++) val[i]=0; 
         
        vcnt=0;
        if(MpuDrv::Mpu.Acquire()) {
          val[0]=MpuDrv::Mpu.getStatus();
          val[1]=MpuDrv::Mpu.getYaw()*180.0/PI;
          //vcnt++;
          MpuDrv::Mpu.Release();
        }
        if(xMotion.Acquire()) {
          xMotion.GetCrdCm(val+2); 
          val[4]=xMotion.GetAdvanceCm();
          //vcnt+=3;
          xMotion.Release();
        }
        vcnt=5;
        if(xSensor.Acquire()) {
          uint16_t nmeas=xSensor.GetNMeas();          
          xSensor.Get(val+vcnt, nmeas);
          vcnt+=nmeas;          
          xSensor.Release();
        }
        break;  
      case REG_ALARM:
        if( xQueueReceive( xAlarmQueue, &rxAlarm, ( TickType_t ) 1 ) )
        {
          vcnt=7;
          val[0]=rxAlarm.level;
          val[1]=rxAlarm.module;
          val[2]=rxAlarm.code;
          val[3]=rxAlarm.iData[0];
          val[4]=rxAlarm.iData[1];
          val[5]=rxAlarm.iData[2];
          val[6]=rxAlarm.iData[3];
        }  
        break;  
      default:;
        rc=CM_RC_FAIL_BADREG;
        vcnt=0;
    }
    /*
    if(CM_RC_FAIL_BADREG==rc) Respond(CM_RC_FAIL_BADREG, 0, "BAD REG");    
    else Respond(CM_RC_OK, vcnt);    
    */

    Respond(rc, vcnt, CM_RC_FAIL_BADREG==rc ? "BAD REG" : NULL);    
    return true;
  }

  // set 
  while(isspace(buf[pos]) || buf[pos]==',' ) pos++;     

  if(!buf[pos]) {
    Respond(CM_RC_FAIL_BADSYN, 0, "SYN");
    return false;
  }
 
  if(buf[pos]=='[') {
    // array    
    vcnt=0;
    pos++;
    while(1) { 
        while(isspace(buf[pos])) pos++;     
        if(!buf[pos]) {
          Respond(CM_RC_FAIL_BADSYN, 0, "SYN");
          return false;
        }
        if(buf[pos]==']') break;        
        val[vcnt]=ReadInt(); // add to array
        vcnt++;
        if(vcnt>CM_NVAL) {
          // too many vals
          Respond( CM_RC_FAIL_TOOMANYVALS, 0, "TOOMANY");
          return false;
        }        
        while(isspace(buf[pos])) pos++;     
        if(buf[pos]==',') pos++;
    }     
    strcat(msgdbg, " [@");    
    itoa_cat(vcnt, msgdbg);    
  } else {
    vcnt=1;
    val[0]=ReadInt();
    strcat(msgdbg, " ");
    itoa_cat(val[0], msgdbg);
  }

  //int8_t rc=0;
  switch(reg) {    
    //case REG_ID:
    //    break;    
    case REG_MOTOR_POWER:
        if(vcnt<2) { rc=CM_RC_FAIL_TOOLESSVALS; break; }
        xMotion.SetMotors(val[0], val[1]);
        break;
    case REG_MOVE:
        if(vcnt<1) { rc=CM_RC_FAIL_TOOLESSVALS; break; }
        xMotion.Move(val[0]);
        break;    
    case REG_STEER:
        if(vcnt<1) { rc=CM_RC_FAIL_TOOLESSVALS; break; }
        xMotion.Steer(val[0]);
        break;    
    case REG_MOVE_BEAR:
        if(vcnt<1) { rc=CM_RC_FAIL_TOOLESSVALS; break; }
        xMotion.MoveBearing(val[0]);
        break;   
    case REG_RESET:
        // requires one parameter with fixed value of 100
        if(vcnt<1 || (val[0]!=1 && val[0]!=2)) { rc=CM_RC_FAIL_TOOLESSVALS; break; }        
        if(val[0]==2) {
          // full delay
          vAddAlarm(CM_ALARM, CM_MODULE_SYS, 100);
          xLogger.vAddLogMsg("RST");           
          vTaskDelay(2000);                
          nvic_sys_reset();
        } else {
          // integrator rest
          if(MpuDrv::Mpu.Acquire()) {
            MpuDrv::Mpu.resetIntegrator();
            MpuDrv::Mpu.Release();
          }      
          if(xMotion.Acquire()) {
            xMotion.Reset();
            val[3]=xMotion.GetAdvanceCm();
            vcnt+=3;
            xMotion.Release();
          }
        }
        break;             
    default:;        
  }

  Respond(rc, 0, CM_RC_FAIL_TOOLESSVALS==rc ? "TOO_LESS_VALS" : NULL);   
  return true;
}


void CommManager::Respond(uint8_t rc, uint8_t vcnt, const char* msg) {
  uint8_t crc;
  strcpy(buf, "R ");
  itoa_cat(rc, buf);
  if(vcnt) {
      if(vcnt==1) 
      {
        strcat(buf, ",");
        itoa_cat(val[0], buf);
        strcat(buf, "%");
      } else 
      {
        strcat(buf, ",[");
        for(uint8_t i=0; i<vcnt; i++) {
          itoa_cat(val[i], buf);
          if(i<vcnt-1) strcat(buf, ",");
        }
        strcat(buf, "]%");
      }    
      crc=CRC();
      itoa_cat(crc, buf);
    }
  Serial3.println(buf);
  if(msg) {
    strcat(msgdbg, " - ");
    strcat(msgdbg, msg);
  } 
  else {
    strcat(msgdbg, "->");
    strcat(msgdbg, buf);    
  }
}

int16_t CommManager::ReadInt() {
  int16_t val=0;
  boolean sign=false;
  while(isspace(buf[pos])) pos++;
  if(buf[pos]=='+') pos++;
  else if(buf[pos]=='-') { 
    sign=true; 
    pos++;
  }
  while (isdigit(buf[pos]))
  {
    val *= 10;
    val += buf[pos] - '0';
    pos++;
  }
  if(sign) val=-val;
  return val;
}

uint8_t CommManager::CRC()
{
  uint8_t crc=0, i;
  pos=0;
  while(buf[pos] && buf[pos]!='%')
    {      
      crc = crc ^ buf[pos];
      for (i=0; i<8; i++) {
        if (crc & 1) {
            crc = (crc >> 1) ^0x8c;
        }
        else {
            crc = (crc >> 1);
        }
      }
      pos++;
    }
  return crc;
}  
    
const char *CommManager::GetBuffer() {
  return buf;
}

const char *CommManager::GetDbgBuffer() {
  return msgdbg;
}

void CommManager::Complete() {
  bytes=0;
  *buf=0;
  *msgdbg=0;
  verb=reg=vcnt=0;
}

void CommManager::vAddAlarm(uint8_t level, uint8_t module, uint8_t code, int16_t p0, int16_t p1, int16_t p2, int16_t p3)
{
   if ( xSemaphoreTake( xAlarmFree, ( portTickType ) 10 ) == pdTRUE )
    {
      txAlarm.uAlarmID++;     
      txAlarm.level=level;
      txAlarm.module=module;
      txAlarm.code=code;
      txAlarm.iData[0]=p0;
      txAlarm.iData[1]=p1;
      txAlarm.iData[2]=p2;
      txAlarm.iData[3]=p3;
      xQueueSendToBack( xAlarmQueue, ( void * ) &txAlarm, ( TickType_t ) 0 );          
      xSemaphoreGive( xAlarmFree );
    }
}




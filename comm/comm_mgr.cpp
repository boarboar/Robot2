#include <Arduino.h>
#include <SoftwareSerial.h>
#include "comm_mgr.h"

// enum RetCodes { CM_RC_OK=0, CM_RC_FAIL_CRC=-2, CM_RC_FAIL_BADSYN=-10, CM_RC_FAIL_BADCMD=-11, CM_RC_FAIL_BADREG=-12, CM_RC_FAIL_TOOMANYVALS=-13 };

void itoa(int n, char s[]);
void ltoa(int32_t n, char s[]);

inline void itoa_cat(int n, char s[]) { itoa(n, s+strlen(s)); }
inline void ltoa_cat(int n, char s[]) { ltoa(n, s+strlen(s)); }

SoftwareSerial swSer(12, 13, false, 256);

int CommManager::GetResultCnt() {
  return vcnt;  
}

const int16_t *CommManager::GetResultVal() {
  return val;
}

uint32_t CommManager::GetLastTimeMs() {
  return tmo;
}

void CommManager::Init(uint32_t comm_speed, int16_t timeout) {
  bytes = 0;
  buf[bytes] = 0;
  this->timeout=timeout;
  swSer.begin(comm_speed);
  while (swSer.available() > 0)  swSer.read();
  Serial.println("\nSoftware serial started");
}

int CommManager::Get(uint16_t reg) {
  if(reg==0) return -7;
  strcpy(snd_buf, "G ");
  itoa_cat(reg, snd_buf);
  return CommandRetr(snd_buf);  
}

int CommManager::Set(uint16_t reg, int16_t *va, uint16_t nval) 
{
  if(reg==0 || NULL==va || nval==0) return -7;
  strcpy(snd_buf, "S ");
  itoa_cat(reg, snd_buf);
  strcat(snd_buf, ",");
  if(nval==1) itoa_cat(*va, snd_buf);
  else {
    strcat(snd_buf, "[");
    for(uint8_t i=0; i<nval; i++) {
      itoa_cat(va[i], snd_buf);
      if(i<nval-1) strcat(snd_buf, ",");
    }
    strcat(snd_buf, "]");
  }
  return CommandRetr(snd_buf);  
}
    
int CommManager::CommandRetr(char *cmd, uint16_t nretr)
{
  int rc=0;
  while((rc=Command(cmd)) && nretr--) {
      yield();
      Serial.print("Retry "); Serial.println(nretr);
  }
  return rc;
}
    
int CommManager::Command(char *cmd)
{  
  if(NULL!=cmd) strcpy(buf, cmd);
  strcat(buf, "%");
  itoa_cat(CRC(), buf);
  Serial.print(">");
  Serial.println(buf);
  while (swSer.available() > 0)  swSer.read();
  swSer.println(buf);
  long t=millis();
  boolean res=false;
  resp_val=-100;
  bytes=0;   
  buf[bytes]=0; 
  vcnt=0;
  while (!res && millis()<t+timeout && bytes<CM_BUF_SIZE) // WRAPAROUND!!!
  {
    while(!res && swSer.available()) 
    {
      buf[bytes] = swSer.read();
      if (buf[bytes] == 10 || buf[bytes] == 13)
      {
        if (bytes > 0) { 
          buf[bytes]=0;        
        } 
        res=true; 
     }
      else
        bytes++;
    }    
    if(!res) yield();
  }

  tmo=millis()-t;
  
  if(bytes>=CM_BUF_SIZE) { 
    //Serial.println("OVERFLOW");
    bytes=0; //overflow, probably caused hang up at start...    
    buf[bytes]=0; 
    return -2;     
  }

  if(!res) {
    //Serial.println("TIMEOUT");
    return -1;    
  }

  
  Serial.print("<");
  Serial.println(buf);
  
  uint8_t crc=CRC();

  if(buf[pos]=='%') {
    pos++;
    while(isspace(buf[pos])) pos++;
    uint8_t mcrc=(uint8_t)ReadInt();    
    if(crc!=mcrc) {
      //Serial.println("CRC FAIL");
      return -3;    
    } 
  }
  pos=0;
  while(isspace(buf[pos])) pos++;
  if(buf[pos]!='R') {
    //Serial.println("SYNTAX FAIL");
    return -7;    
  }
  pos++;
  
  while(isspace(buf[pos]) || buf[pos]==',' ) pos++;     

  if(!buf[pos]) {
    //Serial.println("SYNTAX FAIL");
    return -7;    
  }

  resp_val = ReadInt();

  if(resp_val) {
    //Serial.println("BAD CODE");
    return -6;    
  }

  while(isspace(buf[pos]) || buf[pos]==',' ) pos++;     

  if(buf[pos]=='[') {
    // array    
    vcnt=0;
    pos++;
    while(1) { 
        while(isspace(buf[pos])) pos++;     
        if(!buf[pos]) {
          //Serial.println("SYNTAX FAIL");
          return -7;
        }
        if(buf[pos]==']') break;        
        val[vcnt]=ReadInt(); // add to array
        vcnt++;
        if(vcnt>CM_NVAL) {
          // too many vals
          //Serial.println("SYNTAX FAIL");
          return -7;
        }        
        while(isspace(buf[pos])) pos++;     
        if(buf[pos]==',') pos++;
    }        
  } else {
    vcnt=1;
    val[0]=ReadInt();    
  }
  
  //Serial.print("OK (");
  //Serial.print(millis()-t);
  //Serial.println(" ms) ");
  
  return 0;
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



/* reverse:  reverse string s in place */
 void reverse(char s[])
 {
     int i, j;
     char c;

     for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
         c = s[i];
         s[i] = s[j];
         s[j] = c;
     }
}  

/* itoa:  convert n to characters in s */
void itoa(int n, char s[])
 {
     int i, sign;

     if ((sign = n) < 0)  /* record sign */
         n = -n;          /* make n positive */
     i = 0;
     do {       /* generate digits in reverse order */
         s[i++] = n % 10 + '0';   /* get next digit */
     } while ((n /= 10) > 0);     /* delete it */
     if (sign < 0)
         s[i++] = '-';
     s[i] = '\0';
     reverse(s);
}  


/* itoa:  convert n to characters in s */

void ltoa(int32_t n, char s[])
 {
     int32_t i, sign;

     if ((sign = n) < 0)  /* record sign */
         n = -n;          /* make n positive */
     i = 0;
     do {       /* generate digits in reverse order */
         s[i++] = n % 10 + '0';   /* get next digit */
     } while ((n /= 10) > 0);     /* delete it */
     if (sign < 0)
         s[i++] = '-';
     s[i] = '\0';
     reverse(s);
}  





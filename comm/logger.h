#ifndef _UMP_LOGGER_H_
#define _UMP_LOGGER_H_

#define UMP_LOGGER_NQUEUE  6 
#define UMP_LOGGER_NPARAM  8 
#define UMP_LOGGER_NSSZ   16

class Logger {
public:
  enum ParamTypes { UMP_LOGGER_TYPE_NONE=0, UMP_LOGGER_TYPE_I16=1, UMP_LOGGER_TYPE_S=2 };
  enum Modules { UMP_LOGGER_MODULE_SYS=1, UMP_LOGGER_MODULE_IMU=1, UMP_LOGGER_MODULE_CTL=2 };
  enum Levels { UMP_LOGGER_ALARM=1, UMP_LOGGER_EVENT=2, UMP_LOGGER_INFO=3 };
  static  Logger Instance; // singleton  
  boolean putEvent(uint8_t module,  uint8_t level, uint8_t code,  int16_t p0=0, int16_t p1=0, int16_t p2=0, int16_t p3=0, int16_t p4=0, int16_t p5=0, int16_t p6=0, int16_t p7=0);
  boolean putEvent(uint8_t module,  uint8_t level, uint8_t code, const char* pa);
  boolean flushEvents();
protected:  
  struct q_s {
    uint8_t module;
    uint8_t level;
    uint8_t code;
    uint8_t type;
    uint16_t id;
    union {
      int16_t p[UMP_LOGGER_NPARAM];
      char s[UMP_LOGGER_NSSZ];
    } params;
  };

  Logger();
  struct q_s q[UMP_LOGGER_NQUEUE];
  uint8_t q_h, q_t;
  uint8_t id;
};

#endif //_UMP_LOGGER_H_

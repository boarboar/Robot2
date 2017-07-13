
#define CM_BUF_SIZE 64
#define CM_NVAL     16

#define CM_ID 94

#define ALR_DATA_SZ 4
#define ALR_Q_SZ 16

class CommManager {
  public:
    void Init(uint32_t comm_speed);
    boolean ReadSerialCommand();
    void Complete();
    const char *GetBuffer();
    const char *GetDbgBuffer();
    boolean ProcessCommand();    
    void Respond(uint8_t code, uint8_t vcnt=0, const char* msg=NULL);
    void vAddAlarm(uint8_t level, uint8_t module, uint8_t code, int16_t p0=0, int16_t p1=0, int16_t p2=0, int16_t p3=0);
    
    enum Modules { CM_MODULE_SYS=1, CM_MODULE_IMU=2, CM_MODULE_CTL=3 };
    enum Levels { CM_ALARM=1, CM_EVENT=2, CM_INFO=3 };
    enum RetCodes { CM_RC_OK=0, CM_RC_FAIL_CRC=-2, CM_RC_FAIL_BADSYN=-10, CM_RC_FAIL_BADCMD=-11, CM_RC_FAIL_BADREG=-12, CM_RC_FAIL_TOOMANYVALS=-13 };
  protected:
    struct Alarm
    {
      uint16_t uAlarmID;      
      int16_t iData[ ALR_DATA_SZ ];
      uint8_t level, module, code;
    };
    struct Alarm txAlarm;
    struct Alarm rxAlarm;
    QueueHandle_t xAlarmQueue;
    xSemaphoreHandle xAlarmFree;
  
    enum Regs {REG_None=0, REG_ID=1, REG_STATUS=2, REG_SENS=3, REG_ALL=4, REG_MOTOR_POWER=5, REG_MOVE=6, REG_STEER=7, REG_MOVE_BEAR=8, REG_ALARM=9, REG_ENC=10, 
      REG_RESET=100, REG_N_SENS=101};
    uint8_t CRC();
    int16_t ReadInt() ;        
    char buf[CM_BUF_SIZE];
    char msgdbg[CM_BUF_SIZE*2];
    uint8_t bytes;
    uint8_t pos;
    uint8_t verb;
    uint8_t reg;
    uint8_t vcnt;
    int16_t val[CM_NVAL];    
};






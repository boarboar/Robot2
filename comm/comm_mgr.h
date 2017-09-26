
#define CM_BUF_SIZE 128
#define CM_NVAL     16

class CommManager {
  public:
    void Init(uint32_t comm_speed=115200, int16_t timeout=500);
    int Command(char *cmd);
    int CommandRetr(char *cmd, uint16_t nretr=5);
    int Get(uint16_t reg);
    int Set(uint16_t reg, int16_t *va, uint16_t nval);
    int GetResultCnt();
    const int16_t *GetResultVal();
    uint32_t GetLastTimeMs();
  protected:
    uint8_t CRC();
    int16_t ReadInt() ;    
    
    char buf[CM_BUF_SIZE];
    char snd_buf[CM_BUF_SIZE];
    uint8_t bytes;
    uint8_t pos;
    uint16_t timeout;

    int16_t resp_val;
    uint8_t verb;
    uint8_t reg;
    uint8_t vcnt;
    int16_t val[CM_NVAL];
    uint32_t tmo;
   
};






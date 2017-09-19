#ifndef _UMP_CFG_H_
#define _UMP_CFG_H_

#define CFG_PID_NP  5
  
class CfgDrv {
  /*
  struct pid_params {
    int16_t gain_p;
    int16_t gain_d;
    int16_t gain_i;
    int16_t gain_div;
    int16_t limit_i;
  };
*/
public:
  const unsigned int LAZY_WRITE_TIMEOUT=10; //seconds
  static CfgDrv Cfg; // singleton
  int16_t init();  
  int16_t load(const char* fname);
  int16_t store(const char* fname);
  void printCtrlParams();
  //bool isDirty() { return dirty; }
  bool needToStore();
  bool setSysLog(JsonObject& root);
  bool setPidParams(JsonObject& root);
public:
  uint8_t log_on, debug_on;  
  IPAddress log_addr;
  uint16_t log_port;
  //struct pid_params bear_pid, speed_pid;
  int16_t bear_pid[CFG_PID_NP], speed_pid[CFG_PID_NP];
protected:  
  CfgDrv();
  bool fs_ok;
  bool dirty;
  uint32_t  last_chg;
};

#endif //_UMP_CFG_H_


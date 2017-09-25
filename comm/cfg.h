#ifndef _UMP_CFG_H_
#define _UMP_CFG_H_

#define CFG_PID_NP  5
#define CFG_PID_IDX_GAIN_P  0
#define CFG_PID_IDX_GAIN_D  1
#define CFG_PID_IDX_GAIN_I  2
#define CFG_PID_IDX_GAIN_DIV  3
#define CFG_PID_IDX_LIM_I  4
  
class CfgDrv {
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
  int16_t bear_pid[CFG_PID_NP], speed_pid[CFG_PID_NP];
protected:  
  CfgDrv();
  bool fs_ok;
  bool dirty;
  uint32_t  last_chg;
};

#endif //_UMP_CFG_H_


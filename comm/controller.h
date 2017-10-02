#ifndef _UMP_CONTROLLER_H_
#define _UMP_CONTROLLER_H_

#include "comm_mgr.h"

#define SENS_SIZE 10


class Controller {
public:

  enum FailReason {CTL_FAIL_NONE=0, CTL_FAIL_INIT=1, CTL_FAIL_WRT=2, CTL_FAIL_RD=3 };  
  enum ResetLevels {CTL_RST_NONE=0, CTL_RST_CTL=1, CTL_RST_IMU=2};  
  static Controller ControllerProc; // singleton  
  
  bool init();
  bool reset();
  
  uint8_t getStatus();
  
  void setReset(uint8_t reset_lvl);
  uint8_t isNeedReset();
  bool process();  
  bool processAlarms();
  
  bool setTargPower(float l, float r);
  bool setTargSteering(int16_t s);
  bool setTargSpeed(int16_t s);
  bool setTargBearing(int16_t s);

  uint8_t getIMUStatus();
  uint8_t getNumSensors();

  int16_t getX_cm();
  int16_t getY_cm();
  int16_t getDist_cm();
  int16_t getYaw_grad();
  int16_t *getSensors();  
  int16_t *getPower();  
  
protected:  
  enum Regs {REG_None=0, REG_ID=1, REG_STATUS=2, REG_SENS=3, REG_ALL=4, REG_MOTOR_POWER=5, REG_MOVE=6, REG_STEER=7, REG_MOVE_BEAR=8, REG_ALARM=9, REG_ENC=10, 
    REG_RESET=100, REG_N_SENS=101};
  const int16_t CM_ID=94;    
  Controller();
  uint8_t _testConnection();
  uint8_t _getNumSensors();
  uint8_t _getData();
  uint8_t _resetIMU();

private:  
  CommManager cmgr;
  uint8_t cready;
  uint8_t reset_lvl;
  uint8_t nsens;
  int16_t imu_stat;
  int16_t crd[2];
  int16_t pow[2];
  int16_t dist;
  int16_t yaw;
  int16_t sensors[SENS_SIZE];
};

#endif //_UMP_CONTROLLER_H_


#ifndef _UMP_CONTROLLER_H_
#define _UMP_CONTROLLER_H_

#include "comm_mgr.h"

/*
#define REG_WHO_AM_I         0xFF  // 1 unsigned byte
#define REG_STATUS           0x01  // 2 unsigned bytes
#define REG_START            0x02  // 1 unsigned bytes
#define REG_TARG_ROT_RATE    0x03  // 2 signed ints (4 bytes)
#define REG_ACT_ROT_RATE     0x06  // 2 signed ints (4 bytes)
#define REG_ACT_ADV_ACC      0x09  // 2 signed ints (4 bytes)
#define REG_ACT_POW          0x0A  // 2 signed ints (4 bytes)
#define REG_TARG_POW         0x0B  // 2 signed ints (4 bytes)
#define REG_STEERING         0x0C  // 1 signed int (2 bytes)
#define REG_SENSORS_CNT      0x20  // 8 unsigned ints
#define REG_SENSORS_1H       0x21  // up to 6 unsigned ints
#define REG_SENSORS_2H       0x22  // up to 6 unsigned ints
#define REG_SENSORS_ALL      0x28  // 8 unsigned ints

#define M_OWN_ID 0x53
//#define M_MAGIC_ID 0x4C
#define SPEED_R_SZ 8

#define V_NORM 10000

#define WHEEL_BASE_MM 130
*/

#define SENS_SIZE 10


class Controller {
public:
/*
  enum FailReason {CTL_FAIL_NONE=0, CTL_FAIL_INIT=1, CTL_FAIL_WRT=2, CTL_FAIL_RD=3, CTL_FAIL_OVF=4, CTL_FAIL_ALR=5, CTL_FAIL_OBST=6, CTL_FAIL_SENS_CHECK=7,
    CTL_LOG_PID=100, CTL_LOG_POW=101, CTL_LOG_PBPID=102};
    */
  enum FailReason {CTL_FAIL_NONE=0, CTL_FAIL_INIT=1, CTL_FAIL_WRT=2, CTL_FAIL_RD=3 };  
  enum ResetLevels {CTL_RST_NONE=0, CTL_RST_CTL=1, CTL_RST_IMU=2};  
  static Controller ControllerProc; // singleton  
  
  
  bool init();
  bool reset();
  //bool start();
  
  uint8_t getStatus();
  
  void setReset(uint8_t reset_lvl);
  uint8_t isNeedReset();
  bool process(/*float yaw, uint32_t dt*/);  
  bool processAlarms();
  
  /*
  uint8_t isDataReady();
  void resetIntegrator();
  */
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

  /*
   int16_t getTargSpeed();
  
  //bool setSteering(int16_t s);
  //float *getStoredRotRate();
  int32_t *getStoredAdvance();
  int16_t *getStoredPower();
  //int16_t *getCurPower();
  int16_t *getStoredSensors();  
  //float getMovement();
  //float getRotation();
  float getDistance();
  int16_t getSpeed();
  //float getAngle();
  float getX();
  float getY();
  bool getActPower();   
  float getAVQErr();
*/
protected:  
  enum Regs {REG_None=0, REG_ID=1, REG_STATUS=2, REG_SENS=3, REG_ALL=4, REG_MOTOR_POWER=5, REG_MOVE=6, REG_STEER=7, REG_MOVE_BEAR=8, REG_ALARM=9, REG_ENC=10, 
    REG_RESET=100, REG_N_SENS=101};
  const int16_t CM_ID=94;    
  Controller();
  uint8_t _testConnection();
  uint8_t _getNumSensors();
  uint8_t _getData();
  uint8_t _resetIMU();
  /*
  //bool getControllerStatus();
  //bool getActRotRate(); // in RPS
  bool getActAdvance(); // in MMs
  bool getSensors(); 
  //bool setStart(uint8_t p); 
  bool startRotate(int16_t tspeed);
  bool setPowerRotate(int16_t dir, int16_t *p);
  bool setPowerStraight(int16_t dir, int16_t *p);
  //void adjustTargBearing(int16_t s, bool absolute);
  int8_t checkObastacle();
*/
private:  
  CommManager cmgr;
  uint8_t cready;
  uint8_t reset_lvl;
  uint8_t nsens;
  int16_t imu_stat;
  int16_t crd[2];
  int16_t dist;
  int16_t yaw;
  int16_t sensors[SENS_SIZE];
  
/*
  uint8_t data_ready;
  
  uint8_t buf[20];  
  //uint8_t sta[2];
  //float mov, rot;
  //uint8_t proc_step;
  //uint8_t _sens_half;
  uint8_t last_obst;
  //float act_rot_rate[2];
  //int16_t targ_rot_rate[2];
  int32_t act_advance[2];
  int32_t act_advance_0[2];
  int16_t act_power[2];
  int16_t sensors[SENS_SIZE];
  //int16_t sensors_buf[SENS_SIZE];
  //int16_t cur_pow[2];
  int16_t base_pow;
  int16_t delta_pow;
  int16_t targ_speed; //mm_s
  int16_t rot_speed; //mm_s
  
  float dist;
  //float angle;
  float curr_yaw;
  float r[2];
  float targ_bearing;
  int16_t speed; //mm/s
  int16_t speed_r[SPEED_R_SZ]; // raw speed readings mm/s
  int16_t err_bearing_p_0, err_bearing_i;
  float err_speed_p_0, err_speed_i;
  uint32_t pid_cnt;
  float qsum_err;
  float run_dist;

  bool sens_alarmed;
  */
};

#endif //_UMP_CONTROLLER_H_


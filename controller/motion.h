class Motor;

class Motion {  
  public:
    enum FailReason {CTL_FAIL_NONE=0, CTL_FAIL_INIT=1, CTL_FAIL_WRT=2, CTL_FAIL_RD=3, CTL_FAIL_OVF=4, CTL_FAIL_ALR=5, CTL_FAIL_OBST=6, CTL_FAIL_SENS_CHECK=7,
      CTL_LOG_PID=100, CTL_LOG_POW=101, CTL_LOG_PBPID=102};
    void Init(Motor *m);    
    void Start();
    void Reset();
    void DoCycle(float yaw, int16_t dt, int16_t *vmeas, int16_t nmeas); 
    void SetMotors(int16_t dp1, int16_t dp2);
    void Move(int16_t tspeed);
    void Steer(int16_t angle);
    void MoveBearing(int16_t angle);
    //void GetAdvance(uint32_t *dst_dist);
    void GetCrdCm(int16_t *crd);
    void GetPower(int16_t *pow);
    int16_t GetSpeedCmpS();
    int16_t GetAdvanceCm();
    bool HasTask();
    bool Acquire();
    void Release();
  protected:     
    void AdjustTargBearing(int16_t s, bool absolute);
    void StartRotate(int16_t tspeed);
    void SetPowerStraight(int16_t dir, int16_t *p);
    void SetPowerRotate(int16_t dir, int16_t *p);
    int16_t DoCollisionCheck(int16_t speed, int16_t *vmeas, int16_t nmeas, int16_t *act_val=NULL);

    xSemaphoreHandle xMotionFree;
    //TickType_t xRunTime;
    Motor *pxMotor;
    bool bReady;
    int16_t iTargBearing; // in grads
    int16_t iTargSpeed; //mm_s
    int16_t iTargRot; //mm_s ??
    uint32_t lPIDCnt;
    int16_t err_bearing_p, err_bearing_d;
    int16_t err_speed_p, err_speed_d;
    int16_t err_bearing_p_0, err_bearing_i;
    int16_t err_speed_p_0, err_speed_i;
    int16_t delta_pow;
    int16_t base_pow;
    int16_t run_pow[2];
    int16_t speed; //mm_s
    uint32_t lAdvance0[2], lAdvance[2]; // in mm    
    float fAdvanceTot;
    float fCrd[2];
    float fCurrYaw;    
};


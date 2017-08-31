class Motor;

class Motion {  
  public:
    void Init(Motor *m);    
    void Start();
    void Reset();
    void DoCycle(float yaw); 
    void SetMotors(int16_t dp1, int16_t dp2);
    void Move(int16_t tspeed);
    void Steer(int16_t angle);
    void MoveBearing(int16_t angle);
    void GetAdvance(uint32_t *dst_dist);
    void GetCrdCm(int16_t *crd);
    int16_t GetAdvanceCm();
    bool HasTask();
    bool Acquire();
    void Release();
  protected:     
    void AdjustTargBearing(int16_t s, bool absolute);
    void StartRotate(int16_t tspeed);
    void SetPowerStraight(int16_t dir, int16_t *p);
    void SetPowerRotate(int16_t dir, int16_t *p);

    xSemaphoreHandle xMotionFree;
    TickType_t xRunTime;
    Motor *pxMotor;
    bool bReady;
    int16_t iTargBearing; // in grads
    int16_t iTargSpeed; //mm_s
    int16_t iTargRot; //mm_s ??
    uint32_t lPIDCnt;
    int16_t err_bearing_p_0, err_bearing_i;
    int16_t delta_pow;
    int16_t base_pow;
    uint32_t lAdvance0[2], lAdvance[2]; // in mm
    float fCrd[2];
    float fCurrYaw;    
};


class Motor {  
  public:
    void Init(int m_1_1_pin, int m_1_2_pin, int m_1_enab_pin, int m_1_enc_pin, int m_2_1_pin, int m_2_2_pin, int m_2_enab_pin, int m2_enc_pin);    
    //void Start();
    void DoCycle(); 
    void SetMotors(int8_t dp1, int8_t dp2);
    bool GetEncDist(uint16_t *dst_enc, uint32_t *dst_dist);
    void encInterrupt(uint16_t i);
    bool Acquire();
    void Release();
  protected: 
    void Low_Drive(uint8_t i); 
    struct {
      uint16_t pin_1, pin_2, pin_enab, pin_enc;
      uint16_t enc_count;
      uint32_t enc_accum;
      //uint32_t dist;
      uint8_t enc_prev_st;
      int8_t dir;
      uint16_t power;
      TickType_t xLastWakeTime;
    } m[2];
    xSemaphoreHandle xMotorFree;
    //bool running;
    
};


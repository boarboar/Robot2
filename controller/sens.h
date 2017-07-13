#include <Servo.h>

#define SERVO_NSTEPS  2
#define M_SENS_N      (SERVO_NSTEPS*4+2) // number of readings

class Sensor {  
  public:
    void Init(int servo_pin, int sens_in_pin_0, int sens_out_pin_0, int sens_in_pin_1, int sens_out_pin_1);    
    void Start();
    void DoCycle();    
    int16_t GetNMeas();
    int16_t Get();
    void Get(int16_t *v, int16_t n);
    bool Acquire();
    void Release();
    void echoInterrupt(uint16_t i);
  protected: 
    int16_t sens_in_pin[2];
    int16_t sens_out_pin[2];
    uint32_t t0[2], di[2];
    uint8_t sens_state[2];
    int16_t value[M_SENS_N];
    int8_t sservo_pos;
    int8_t sservo_step;
    uint8_t wait_sens;
    Servo xServo;
    xSemaphoreHandle xSensFree;
    TaskHandle_t xTaskToNotify;
    bool running;
};


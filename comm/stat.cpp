#include <Arduino.h>
#include "stat.h"

Stat Stat::StatStore ; // singleton

Stat::Stat() : cycle_delay_cnt{0}, cycle_mpu_dry_cnt(0), 
  mpu_owfl_cnt(0), mpu_gup_cnt(0), mpu_exc_cnt(0), mpu_ndt_cnt(0)      
  {
  }


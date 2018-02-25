#ifndef OBSERVER_H
#define OBSERVER_H

#include "Arduino.h"

class Observer
{
public:
  Observer();
  // Observer();
  virtual ~Observer();
  void Init(float gyro_sen, float acc_sen);
  float* ComplementaryFilter(uint16_t *gyrData, uint16_t *accData, float dt);

protected:

private:
  float GYROSCOPE_SENSITIVITY,
        ACCELEROMETER_SENSITIVITY,
        rad_to_deg,
        *Angles; // Pitch and Roll
};

#endif // OBSERVER_H

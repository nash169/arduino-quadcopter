#include "observer.h"

// Constructor
Observer::Observer() { }

// Default Constructor
// Observer::Observer() { }

// Destructor
Observer::~Observer() {

}

void Observer::Init(float gyro_sen, float acc_sen) {
  GYROSCOPE_SENSITIVITY = gyro_sen;
  ACCELEROMETER_SENSITIVITY = acc_sen;
  Angles = new float[2];
  Angles[0] = 0;
  Angles[1] = 0;
  rad_to_deg = 180/3.14159265359;
}

float* Observer::ComplementaryFilter(uint16_t *gyrData, uint16_t *accData, float dt)
{
  float pitchAcc, rollAcc;

  // Integrate the gyroscope data -> int(angularSpeed) = angle
  Angles[0] += ((float)gyrData[0] / GYROSCOPE_SENSITIVITY) * dt; // Angle around the X-axis
  Angles[1] += ((float)gyrData[1] / GYROSCOPE_SENSITIVITY) * dt;    // Angle around the Y-axis

  // Compensate for drift with accelerometer data if !bullshit
  // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
  int forceMagnitudeApprox = abs(accData[0]) + abs(accData[1]) + abs(accData[2]);

  if (forceMagnitudeApprox > ACCELEROMETER_SENSITIVITY && forceMagnitudeApprox < ACCELEROMETER_SENSITIVITY*4) {
	   // Turning around the X axis results in a vector on the Y-axis
     rollAcc = atan2f((float)accData[1], sqrt(pow((float)accData[0], 2) + pow((float)accData[2], 2))) * rad_to_deg;
     Angles[0] = Angles[0] * 0.98 + rollAcc * 0.00;

	   // Turning around the Y axis results in a vector on the X-axis
     pitchAcc = atan2f((float)accData[0], sqrt(pow((float)accData[1], 2) + pow((float)accData[2], 2))) * rad_to_deg;
     Angles[1] = Angles[1] * 0.98 + pitchAcc * 0.00;
  }

  return Angles;
}

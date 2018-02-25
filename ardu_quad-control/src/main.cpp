#include "Arduino.h"

#include "radio_decoder.h"
#include "transmitter.h"
#include "observer.h"
#include "imu.h"

#define SERIAL_PORT_SPEED 115200

/*==== SIGNALS FLOWS FROM COMMAND TO SERVOS ==========================
RightSide-RightLeft     -> Ch. 1 -> A2
RightSide-UpDown        -> Ch. 2 -> A1
LeftSide-UpDown         -> Ch. 3 -> A0
LeftSide-RightLeft      -> Ch. 4 -> A3
Left-Knob (VRA)         -> Ch. 5 -> ?
Right-Knob (VRB)        -> Ch. 6 -> ?
Left-Lever (SWA)        -> Ch. 7
MiddleLeft-Lever (SWB)  ->
MiddleRight-Lever (SWC) ->
Right-Lever (SWD)       -> Ch. 8
=====================================================================*/
// Arduino Analog & Digital pins setting
enum {
  PWM,
  PPM
};
/*---PWM---*/
/* Max 4 channnels with Arduino Uno since A4 and A5 are used by the MPU6050 */
// uint8_t const inPins [] = {A0, A1, A2, A3};
// uint8_t const inPins_num = sizeof(inPins)/sizeof(inPins[0]);
// float const minAnalog_out [] = {-15.0, -15.0, -15.0, -15.0};
// float const maxAnalog_out [] = {15.0, 15.0, 15.0, 15.0};

/*---PPM---*/
/* Max 8 channnels. It looks like the limit for the flysky TX/RX */
const uint8_t inPins [] = {A0};
const uint8_t inPins_num = 8;
const float minAnalog_out [] = {-15.0, -15.0, -15.0, -15.0, -15.0, -15.0, -15.0, -15.0};
const float maxAnalog_out [] = {15.0, 15.0, 15.0, 15.0, 15.0, 15.0, 15.0, 15.0};

// Output
const uint8_t outPins[] = {6, 9, 10, 11};
const uint8_t outPins_num = sizeof(outPins)/sizeof(outPins[0]);
const float minDigital_in [] = {-15.0, -15.0, -15.0, -15.0};
const float maxDigital_in [] = {15.0, 15.0, 15.0, 15.0};
const int minDigital_out [] = {1000, 1000, 1000, 1000};
const int maxDigital_out [] = {2000, 2000, 2000, 2000};

// Radio Input Boundaries
bool CALIBRATE = false;

// MPU 6050 & Observer
IMU imuSens(0);
dmp_IMU* State = new dmp_IMU;
float time, timePrev, elapsedTime,
      gyro_sens, acc_sens,
      *angles;

// Pilot & Control arrays
float u_p[inPins_num],
      u_c[outPins_num],
      g[] = {1, 1, 1, 1, 0.001, 0.001};

// Decoder & Transmitter objects creation
RadioDecoder radio(inPins, inPins_num, PPM);
Transmitter myPilot(outPins, outPins_num);

// Observer kalman;

void setup() {
  Serial.begin(SERIAL_PORT_SPEED);

  imuSens.Init();
  // gyro_sens = imuSens.SetGyroSens(0);
  // acc_sens = imuSens.SetAccSens(0);

  radio.Initialize(minAnalog_out, maxAnalog_out, CALIBRATE);
  myPilot.Initialize(minDigital_in, maxDigital_in, minDigital_out, maxDigital_out);

  // kalman.Init(gyro_sens, acc_sens);

  time = millis();
}

void loop() {

  imuSens.Run();
  State = imuSens.GetDMP();

  timePrev = time;
  time = millis();
  elapsedTime = (time - timePrev)/1000;

  // angles = kalman.ComplementaryFilter(State->angularVel, State->linearAcc, elapsedTime);

  /*----RADIO RECEIVING----*/
  radio.rc_read_values();
  for (int i = 0; i < inPins_num; ++i)
    u_p[i] = radio.CtrInput(i);

  for(int i=0; i<outPins_num; i++)
    u_c[i] = -15.0;

  /*----CONTROLLING SERVO----*/

  // Pilot Inputs
  // if(u_p[6] > 0){
    u_c[0] = g[0]*u_p[2] - g[1]*u_p[3] - g[2]*u_p[1] + g[3]*u_p[0];
    u_c[1] = g[0]*u_p[2] + g[1]*u_p[3] - g[2]*u_p[1] - g[3]*u_p[0];
    u_c[2] = g[0]*u_p[2] - g[1]*u_p[3] + g[2]*u_p[1] - g[3]*u_p[0];
    u_c[3] = g[0]*u_p[2] + g[1]*u_p[3] + g[2]*u_p[1] + g[3]*u_p[0];
  // }

  // SAS Inputs
  // if(u_p[7] > 0) {
  //   u_c[0] = u_c[0] - g[4]*1.0 - g[5]*1.0;
  //   u_c[1] = u_c[1] + g[4]*1.0 - g[5]*1.0;
  //   u_c[2] = u_c[2] + g[4]*1.0 + g[5]*1.0;
  //   u_c[3] = u_c[3] - g[4]*1.0 + g[5]*1.0;
  // }

  for(int i=0; i<outPins_num; i++)
      myPilot.SetServo(u_c[i], i);

  Serial.print(State->ypr[1]);
  Serial.print(" - ");
  Serial.println(State->ypr[2]);

  // delay(200);
}

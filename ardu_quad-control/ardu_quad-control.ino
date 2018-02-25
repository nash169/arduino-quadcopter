#include "libs/radio_decoder.h"
#include "libs/transmitter.h"
// #include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
// #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//     #include "Wire.h"
// #endif
#define LIBCALL_ENABLEINTERRUPT
#include <EnableInterrupt.h>


#define SERIAL_PORT_SPEED 9600

/*==== SIGNALS FLOWS FROM COMMAND TO SERVOS ==========================
RightSide-RightLeft -> Ch. 1 -> A2
RightSide-UpDown    -> Ch. 2 -> A1
LeftSide-UpDown     -> Ch. 3 -> A0
LeftSide-RightLeft  -> Ch. 4 -> A3
Left-Knob           -> Ch. 5 -> ?
Right-Knob          -> Ch. 6 -> ?
=====================================================================*/
// Arduino Analog & Digital pins setting
uint8_t const inPins [] = {A0};
uint8_t const outPins[] = {6, 9, 10, 11};
uint8_t const inPins_num = sizeof(inPins)/sizeof(inPins[0]);
uint8_t const outPins_num = sizeof(outPins)/sizeof(outPins[0]);

// Arduino Output analog boundaries
float const minAnalog_out [] = {-15.0};
float const maxAnalog_out [] = {15.0};

// Arduino Input/Output digital boundaries
float const minDigital_in [] = {-15.0, -15.0, -15.0, -15.0};
float const maxDigital_in [] = {15.0, 15.0, 15.0, 15.0};
int const minDigital_out [] = {1000, 1000, 1000, 1000};
int const maxDigital_out [] = {2000, 2000, 2000, 2000};

// Radio Input Boundaries
bool CALIBRATE = false;

// MPU control/status vars
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' }; // packet structure for InvenSense teapot demo
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// Pilot & Control arrays
float u_p[inPins_num],
      u_c[outPins_num],
      g[] = {1, 1, 1, 1, 0, 0};

// Decoder & Transmitter objects creation
RadioDecoder radio(inPins, inPins_num);
Transmitter myPilot(outPins, outPins_num);

MPU6050 mpu;

void setup() {
  Serial.begin(SERIAL_PORT_SPEED);
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);
      // enable Arduino interrupt detection
      Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
      enableInterrupt(0, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();
      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;
      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }
  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  radio.Initialize(minAnalog_out, maxAnalog_out, CALIBRATE);
  myPilot.Initialize(minDigital_in, maxDigital_in, minDigital_out, maxDigital_out);
}

void loop() {
  // if (!dmpReady) return;
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;
      // Get Yaw Pitch Roll
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      // display quaternion values in InvenSense Teapot demo format:
      // teapotPacket[2] = fifoBuffer[0];
      // teapotPacket[3] = fifoBuffer[1];
      // teapotPacket[4] = fifoBuffer[4];
      // teapotPacket[5] = fifoBuffer[5];
      // teapotPacket[6] = fifoBuffer[8];
      // teapotPacket[7] = fifoBuffer[9];
      // teapotPacket[8] = fifoBuffer[12];
      // teapotPacket[9] = fifoBuffer[13];
      // Serial.write(teapotPacket, 14);
      // teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
  }

  /*----RADIO RECEIVING----*/
  radio.rc_read_values();
  for (int i = 0; i < inPins_num; ++i)
    u_p[i] = radio.CtrInput(i);

  // radio.PrintOut();

  // u_c[0] = g[0]*u_p[2] - g[1]*u_p[3] - g[2]*u_p[1] + g[3]*u_p[0] - g[4]*ypr[2] - g[5]*ypr[1];
  // u_c[1] = g[0]*u_p[2] + g[1]*u_p[3] - g[2]*u_p[1] - g[3]*u_p[0] + g[4]*ypr[2] - g[5]*ypr[1];
  // u_c[2] = g[0]*u_p[2] - g[1]*u_p[3] + g[2]*u_p[1] - g[3]*u_p[0] + g[4]*ypr[2] + g[5]*ypr[1];
  // u_c[3] = g[0]*u_p[2] + g[1]*u_p[3] + g[2]*u_p[1] + g[3]*u_p[0] - g[4]*ypr[2] + g[5]*ypr[1];

  /*----CONTROLLING SERVO----*/
  for(int i=0; i<outPins_num; i++){
      u_c[i] = u_p[0];
      myPilot.SetServo(u_c[i], i);
  }
  myPilot.PrintOut();

  Serial.println(F("Testing changes"));
}

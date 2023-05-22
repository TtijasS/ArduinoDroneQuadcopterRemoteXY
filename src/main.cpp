#include <Arduino.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <QuickPID.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation is used in I2Cdev.h

/////////////////////////////////////////////
//                 RemoteXY                //
/////////////////////////////////////////////
// RemoteXY select connection mode and include library
#define REMOTEXY_MODE__HARDSERIAL

#include <RemoteXY.h>

// RemoteXY connection settings
#define REMOTEXY_SERIAL Serial
#define REMOTEXY_SERIAL_SPEED 115200

// RemoteXY configurate
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =  // 80 bytes
    {255, 8, 0, 25, 0, 73, 0, 16, 31, 0, 1, 0, 26, 8, 7, 7, 120, 31, 0, 3,
     138, 37, 8, 63, 7, 2, 26, 5, 1, 0, 23, 40, 40, 2, 26, 31, 10, 48, 0, 0,
     10, 10, 4, 26, 31, 79, 78, 0, 31, 79, 70, 70, 0, 5, 0, 60, 23, 40, 40, 2,
     26, 31, 1, 0, 19, 12, 7, 7, 35, 31, 0, 67, 4, 13, 0, 83, 7, 2, 26, 25};

// this structure defines all the variables and events of your control interface
struct {
    // input variables
    uint8_t btnGreen;   // =1 if button pressed, else =0
    uint8_t btnSelect;  // =0 if select position A, =1 if position B, =2 if position C, ...
    int8_t joyRP_x;     // from -100 to 100
    int8_t joyRP_y;     // from -100 to 100
    uint8_t onOff;      // =1 if state is ON, else =0
    int8_t joyYT_x;     // from -100 to 100
    int8_t joyYT_y;     // from -100 to 100
    uint8_t btnRed;     // =1 if button pressed, else =0

    // output variables
    char textField[25];  // string UTF8 end zero

    // other variable
    uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY;
#pragma pack(pop)
/////////////////////////////////////////////
//                 Battery                 //
/////////////////////////////////////////////
// 4.2 V ~ full battery
// 4.2 V ~ 5 V on analogRead(A0) pin
// on Nano A0 to A7 hav max resolution 10 bit
// Voltage values between 0 and 5 V are mapped on spectrum from 0 to 1023 bit
// map(0, 5, 0, 1023)
// 4.2 V equals 4200 mV => multiplier calculation
const float bat_alpha{0.05};  // complementary filter alpha for noise reduction (example 0.9 % old reading + 0.1 % new)
const float multiplier{4.3};  // 4200 / 1023 = 4.10557 (mine was 2 V short, so i raised multiplier by .2)
float incrementPID{0.01f};    // how much the PID parameter will be incremented/ decremented

float batteryVoltage{0.0f};  // the smoothed value that we output
bool computeBattery{0};
unsigned long timerBattery{0};   // computer smooth battery value every x millis, so RemoteXY doesn't update edit field every loop
unsigned long timerSetpoint{0};  // read battery every x seconds
unsigned long timerPID{0};       // timer for PID calculations
bool armDisarm{0};               // enable or dissable drone throttle

// buffers for converting floats to string
char buf0[10];
char buf1[10];
char buf2[10];
char buf3[10];
/////////////////////////////////////////////
//                  PID                    //
/////////////////////////////////////////////

#define CAL_N 10  // number of calibration cicles (1 = 100 samples)
// how to connect the motrs (pin numbers ; PCB connector number)
// (3)	 (11)	(CN1)	(CN4)
//     X			  X
// (9)	 (10)	(CN2)	(CN3)
static const int FLmotor{3};
static const int FRmotor{11};
static const int BLmotor{9};
static const int BRmotor{10};

// Stores the PWM output value
int FLspeed{0};
int FRspeed{0};
int BLspeed{0};
int BRspeed{0};

const uint32_t sampleTimeUs{6000};  // 1000 us = 1 ms
const float prOutLimits{20.0f};     // pitch and roll PID output limits
const float yawOutLimits{20.0f};    // yaw PID output limits

// JOYSTICK LIMITS
// Y, P, R and throttle controller limits
float yawLimit{0.7f}, pitchLimit{0.25f}, rollLimit{0.25f}, throtleLimit{200};  // map(input, 0, 100, 0, limit)
int yawDeadzone{15};                                                           // from -x to x controller does nothing
// because pitchPID output limit is 25, roll is 25 and yaw is 10 == 25 + 25 + 10
// joystick throttle goes from 0 to 100. 100 * 1.95 = 195 throttle + 60 in worst case scenario for PID corrections
int throttle{0};  // RemoteXY.joyYT_y * throttleLimit
float rollSetpoint{0.0f}, rollInput{0.0f}, rollOutput{0.0f};
float pitchSetpoint{0.0f}, pitchInput{0.0f}, pitchOutput{0.0f};
float yawSetpoint{0.0f}, yawInput{0.0f}, yawOutput{0.0f};
bool yawInvertOutput{0};  // invert yawOutput when yawDeg < 0 so we never encounter transition from 180 to -180, which makes PID go crazy

/////////////////////////////////////////////
//               PID TUNINGS               //
/////////////////////////////////////////////
float prP{0.38f}, prI{0.028f}, prD{0.11f};  // Pitch&Roll kp, ki, kd .38, .03, .11
float yP{0.65f}, yI{0.0f}, yD{0.0f};        // Yaw kp ki kd	.65, 0, 0
int trimRoll{0}, trimPitch{0};              // trim values (if drone is leaning, you can correct with theese)

// for turning off I term when error is too big
bool pitchIswitch{0}, rollIswitch{0};
const int errorThreshold{20};  // max error in degrees before I term is turned off

// -Left trimRoll Right+	(direction of the nose)	+Up trimPitch Down-
// Turn on the drone and sellect YPR output... test

/////////////////////////////////////////////
//                   MPU                   //
/////////////////////////////////////////////
MPU6050 mpu;
// MPU control/status vars
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;                         // [w, x, y, z]			quaternion container
VectorInt16 rot;                      // [x, y, z]				angular velocity
VectorInt16 aa;                       // [x, y, z]				accel sensor measurements
VectorInt16 aaReal;                   // [x, y, z]				gravity-free accel sensor measurements
VectorInt16 aaWorld;                  // [x, y, z]				world-frame accel sensor measurements
VectorFloat gravity;                  // [x, y, z]				gravity vector
float euler[3];                       // [psi, theta, phi]		Euler angle container
float ypr[3];                         // [yaw, pitch, roll]		yaw/pitch/roll container and gravity vector
float pitchDeg{0.0f}, rollDeg{0.0f};  // yawDeg{0.0f},
float yawDps{0};                      // yaw in degrees per second

// ===               INTERRUPT DETECTION ROUTINE                ===
volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

QuickPID yawPID(&yawDps, &yawOutput, &yawSetpoint);
QuickPID pitchPID(&pitchDeg, &pitchOutput, &pitchSetpoint);
QuickPID rollPID(&rollDeg, &rollOutput, &rollSetpoint);

// /////////////////////////////////////////////
//               Custom functions          //
/////////////////////////////////////////////

void smoothBat() {
    // reads the new battery value and smoothes it with bat_alpha param
    // float batReading = analogRead(A0) / 1023.0 * multiplier;
    batteryVoltage = (1.0f - bat_alpha) * batteryVoltage + (bat_alpha * analogRead(A0) / 1023.0f * multiplier);
    // if (millis() - timerBattery >= 253) {
    //     timerBattery = millis();
    // }
}

// Calculates motor PWM values by taking throttle and +- PID outputs
// values are stored in FRspeed, FLspeed, ...
void calculateELMspeed() {
    if (armDisarm == 1) {
        FLspeed = throttle + pitchOutput + rollOutput + yawOutput;  // front left (CW)
        FRspeed = throttle + pitchOutput - rollOutput - yawOutput;  // back	left (CCW)
        BLspeed = throttle - pitchOutput + rollOutput - yawOutput;  // front right (CCW)
        BRspeed = throttle - pitchOutput - rollOutput + yawOutput;  // back	right (CW)

        if (FLspeed < 6) FLspeed = 6;
        if (FRspeed < 6) FRspeed = 6;
        if (BLspeed < 6) BLspeed = 6;
        if (BRspeed < 6) BRspeed = 6;

        if (FLspeed > 255) FLspeed = 255;
        if (FRspeed > 255) FRspeed = 255;
        if (BLspeed > 255) BLspeed = 255;
        if (BRspeed > 255) BRspeed = 255;

    } else {
        FLspeed = 0;
        FRspeed = 0;
        BLspeed = 0;
        BRspeed = 0;
    }
};

// Update desired motor speeds from FLspeed, FRspeed,...
void setELMspeed() {
    analogWrite(FLmotor, FLspeed);
    analogWrite(FRmotor, FRspeed);
    analogWrite(BLmotor, BLspeed);
    analogWrite(BRmotor, BRspeed);
}

void calculateSetpoint() {
    if (millis() - timerSetpoint > 50) {
        rollSetpoint = (RemoteXY.joyRP_x * rollLimit) + trimRoll;
        pitchSetpoint = (RemoteXY.joyRP_y * (-pitchLimit)) + trimPitch;
        throttle = map(RemoteXY.joyYT_y, -100, 100, 0, throtleLimit);
        if ((RemoteXY.joyYT_x <= -yawDeadzone) || (RemoteXY.joyYT_x >= yawDeadzone)) {
            yawSetpoint = RemoteXY.joyYT_x * (-yawLimit);
        } else {
            yawSetpoint = 0;
        }
        timerSetpoint = millis();
    }
}

void pitchITermSwitch(int maxErr) {
    float pitchErr = abs(pitchSetpoint - pitchDeg);
    // when pitchIswitch == 0 - normal pitchPID mode (all terms in use)
    // when pitchIswitch == 1 - no I term pitchPID mode
    if (pitchIswitch == 0 && pitchErr > maxErr) {
        pitchPID.SetTunings(prP, 0.0f, prD);
        pitchIswitch = 1;
    } else if (pitchIswitch == 1 && pitchErr <= maxErr) {
        pitchPID.SetTunings(prP, prI, prD);
        pitchIswitch = 0;
    }
}

void rollITermSwitch(int maxErr) {
    float rollErr = abs(rollSetpoint - rollDeg);
    // when rollIswitch == 0 - normal pitchPID mode (all terms in use)
    // when rollIswitch == 1 - no I term pitchPID mode
    if (rollIswitch == 0 && rollErr > maxErr) {
        rollPID.SetTunings(prP, 0.0f, prD);
        rollIswitch = 1;
    } else if (rollIswitch == 1 && rollErr <= maxErr) {
        rollPID.SetTunings(prP, prI, prD);
        rollIswitch = 0;
    }
}

void computePID() {
    if (micros() - timerPID >= sampleTimeUs) {
        pitchITermSwitch(errorThreshold);  // turn off pitch I term when error is bigger than maxErr value
        rollITermSwitch(errorThreshold);   // turn off roll I term when error is bigger than maxErr value
        yawPID.Compute();
        pitchPID.Compute();
        rollPID.Compute();
        calculateELMspeed();
        setELMspeed();
        timerPID = micros();
    }
}

// If throttle stick equals 100, PID increments will change by 1.0,
// if it's -100 (lowest position) they change by 0.001,
// else (middle position) by 0.01
void setPIDincrements() {
    if (RemoteXY.joyYT_y == 100)
        incrementPID = 1.0f;
    else if (RemoteXY.joyYT_y == -100)
        incrementPID = 0.001f;
    else
        incrementPID = 0.01;
}

/////////////////////////////////////////////
//                  SETUP                  //
/////////////////////////////////////////////
void setup() {
    // Setup motor PWM frequency from 490 Hz to 31.4 kHz
    // TCCR1A = 0b00000001;  // Pins D9, D10 to 8bit phase correct
    // TCCR1B = 0b00000001;  //
    // TCCR2B = 0b00000001;  // Pins D3, D11 to 8bit phase correct
    // TCCR2A = 0b00000001;  // phase correct -> count up 8-bit, count down 8-bit (2*256)

    // Setup PWM frequency from 490 Hz to 62.5 kHz
    TCCR1A = 0b00000001;  // Pins D9 and D10 to 8-bit
    TCCR1B = 0b00001001;  // x1 fast pwm

    TCCR2A = 0b00000011;  // Pins D3 and D11 to 8-bit
    TCCR2B = 0b00000001;  // x1 fast pwm

    // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24;  // 400kHz I2C clock
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif
    Serial.begin(115200);

    // Battery setup
    pinMode(A0, INPUT);
    analogReference(INTERNAL);
    batteryVoltage = 4.2;  // default starting voltage (predicted full batt)

    // // Set PID modes 1U == automatic QuickPID::Control::automatic
    rollPID.SetMode(QuickPID::Control::timer);
    pitchPID.SetMode(QuickPID::Control::timer);
    yawPID.SetMode(QuickPID::Control::timer);

    rollPID.SetSampleTimeUs(sampleTimeUs);
    pitchPID.SetSampleTimeUs(sampleTimeUs);
    yawPID.SetSampleTimeUs(sampleTimeUs);

    // Set PID limits
    pitchPID.SetOutputLimits(-prOutLimits, prOutLimits);
    rollPID.SetOutputLimits(-prOutLimits, prOutLimits);
    yawPID.SetOutputLimits(-yawOutLimits, yawOutLimits);

    // Set PID tunings
    pitchPID.SetTunings(prP, prI, prD);
    rollPID.SetTunings(prP, prI, prD);
    yawPID.SetTunings(yP, yI, yD);

    // Set up ports for PWM, then in the loop we
    // directly manipulate the compare value registers (OCR1A/B OCR2A/B)
    // This makes a loop cycle a bit faster
    // analogWrite(FLmotor, FLspeed);
    // analogWrite(FRmotor, FRspeed);
    // analogWrite(BLmotor, BLspeed);
    // analogWrite(BRmotor, BRspeed);

    // MPU6050 init, setup and calibration
    devStatus = mpu.dmpInitialize();
    mpu.setXAccelOffset(127);  // replace these with your own offsets
    mpu.setYAccelOffset(-2497);
    mpu.setZAccelOffset(1773);
    mpu.setXGyroOffset(-93);
    mpu.setYGyroOffset(16);
    mpu.setZGyroOffset(60);
    // Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    if (devStatus == 0) {
        // Serial.println(F("Calibrating"));
        // mpu.CalibrateAccel(CAL_N);
        // mpu.CalibrateGyro(CAL_N);

        /*
                -- READ THIS --
                On the first run set CAL_N to 100.
        Then uncomment the line 'mpu.PrintActiveOffsets() and run.
                It will print out new active offsets. Replace printed out offsets with Accel and Gyro offsets with the printed ones.
        After that revert CAL_N to around 6 (more means longer calibration time but higher accuracy)
                */
        // mpu.PrintActiveOffsets();

        // Serial.println(F("Calibration completed"));
        // Serial.println(F("Enabling DMP"));
        mpu.setDMPEnabled(true);
        // Serial.println(F("Enabling interrupt"));
        attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        // Serial.println(F("DMP Ready, waiting for first interrupt"));
        dmpReady = true;

        // Get expected packed size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        // Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        // Serial.println(F(")"));
    }

    Serial.end();
    delay(200);

    RemoteXY_Init();
    RemoteXY.joyYT_y = -100;
    timerBattery = millis();
    timerSetpoint = millis();
    timerPID = micros();
}

/////////////////////////////////////////////
//                  LOOP                   //
/////////////////////////////////////////////

void loop() {
    // if (!dmpReady) return;
    computePID();  // we check computePID so many times so there is a lesser chance timer will be missed
    // perform remoteXY HW serial data transfer
    RemoteXY_Handler();
    computePID();
    // MPU get data and calculate yaw pitch roll position
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        computePID();
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.dmpGetGyro(&rot, fifoBuffer);
        yawDps = (yawDps * 0.8f) + (rot.z * 0.2f);  // complementary filter
        // yawDeg = ypr[0] * RAD_TO_DEG;
        pitchDeg = ypr[1] * RAD_TO_DEG;
        rollDeg = ypr[2] * RAD_TO_DEG;
    }
    computePID();

    calculateSetpoint();

    computePID();

    smoothBat();

    /*
    RemoteXY select buttons
    A = battery
        B,C,D,E,F,G = prP, prI, prD, yP, yI, yD
        H = apply new tuning params
    */

    if (armDisarm == 0) {
        // don't allow arming the drone if the throttle isn't == 0 (joystick value == -100)
        // it's inside this if statement, so we don't check armDisarm == 0 more than once
        if (RemoteXY.onOff == 1) {
            if (RemoteXY.joyYT_y > -100) {
                RemoteXY.onOff = 0;
                return;
            } else {
                armDisarm = 1;
                // Calculate I only when drone is armed, so I term doesn't saturate too much on the uneaven terrain
                yawPID.SetTunings(yP, yI, yD);
                pitchPID.SetTunings(prP, prI, prD);
                rollPID.SetTunings(prP, prI, prD);
            }
        }
        if (batteryVoltage > 1 && batteryVoltage < 3.8) {
            dtostrf(batteryVoltage, 1, 1, buf0);
            snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField), PSTR("%sV Battery low!"), buf0);

            // We check if 200 ms has passed, so buttons work more consistent
        } else if (millis() - timerBattery > 100) {
            setPIDincrements();
            switch (RemoteXY.btnSelect) {
                case 0:
                    dtostrf(batteryVoltage, 1, 1, buf0);
                    snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField), PSTR("%sV"), buf0);
                    break;
                case 1:
                    snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField), PSTR("trim pitch:%d"), trimPitch);
                    if (RemoteXY.btnGreen)
                        ++trimPitch;
                    else if (RemoteXY.btnRed)
                        --trimPitch;
                    break;
                case 2:
                    snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField), PSTR("trim roll:%d"), trimRoll);
                    if (RemoteXY.btnGreen)
                        ++trimRoll;
                    else if (RemoteXY.btnRed)
                        --trimRoll;
                    break;
                case 3:
                    dtostrf(prP, 1, 3, buf0);
                    snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField), PSTR("prP:%s"), buf0);
                    if (RemoteXY.btnGreen)
                        prP += incrementPID;
                    else if (RemoteXY.btnRed)
                        prP -= incrementPID;
                    break;
                case 4:
                    dtostrf(prI, 1, 3, buf0);
                    snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField), PSTR("prI:%s"), buf0);
                    if (RemoteXY.btnGreen)
                        prI += incrementPID;
                    else if (RemoteXY.btnRed)
                        prI -= incrementPID;
                    break;
                case 5:
                    dtostrf(prD, 1, 3, buf0);
                    snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField), PSTR("prD:%s"), buf0);
                    if (RemoteXY.btnGreen)
                        prD += incrementPID;
                    else if (RemoteXY.btnRed)
                        prD -= incrementPID;
                    break;
                case 6:
                    dtostrf(yP, 1, 3, buf0);
                    snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField), PSTR("yP:%s"), buf0);
                    if (RemoteXY.btnGreen)
                        yP += incrementPID;
                    else if (RemoteXY.btnRed)
                        yP -= incrementPID;
                    break;
                case 7:
                    dtostrf(yI, 1, 3, buf0);
                    snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField), PSTR("yI:%s"), buf0);
                    if (RemoteXY.btnGreen)
                        yI += incrementPID;
                    else if (RemoteXY.btnRed)
                        yI -= incrementPID;
                    break;
                case 8:
                    dtostrf(yD, 1, 3, buf0);
                    snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField), PSTR("yD:%s"), buf0);
                    if (RemoteXY.btnGreen)
                        yD += incrementPID;
                    else if (RemoteXY.btnRed)
                        yD -= incrementPID;
                    break;
                case 9:
                    // Don't allow negative Kp, Ki and Kd values
                    if (RemoteXY.btnGreen) {
                        if (prP < 0) {
                            prP = 0.0f;
                        }
                        if (prI < 0) {
                            prI = 0.0f;
                        }
                        if (prD < 0) {
                            prD = 0.0f;
                        }
                        if (yP < 0) {
                            yP = 0.0f;
                        }
                        if (yI < 0) {
                            yI = 0.0f;
                        }
                        if (yD < 0) {
                            yD = 0.0f;
                        }
                        rollPID.SetTunings(static_cast<double>(prP), static_cast<double>(prI), static_cast<double>(prD));
                        pitchPID.SetTunings(static_cast<double>(prP), static_cast<double>(prI), static_cast<double>(prD));
                        yawPID.SetTunings(static_cast<double>(yP), static_cast<double>(yI), static_cast<double>(yD));
                        snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField), PSTR("--OK--"));
                    } else {
                        snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField), PSTR("Tune PID params?"));
                    }
                    break;
                default:
                    break;
            }
            timerBattery = millis();
        }
    } else {
        // disarm the drone if it's overturning or onOff is switched off
        if (abs(pitchDeg) > 80 || abs(rollDeg) > 80 || RemoteXY.onOff == 0) {
            armDisarm = 0;
            RemoteXY.onOff = 0;
            // dissable I term when drone isn't armed to prevent unwanted satturation
            yawPID.SetTunings(yP, 0.0f, yD);
            pitchPID.SetTunings(prP, 0.0f, prD);
            rollPID.SetTunings(prP, 0.0f, prD);
            return;
        }
        // if (batteryVoltage > 1 && batteryVoltage < 3.8) {
        //     dtostrf(batteryVoltage, 1, 1, buf0);
        //     snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField), PSTR("%s Battery low!"), buf0);
        // } else {
        switch (RemoteXY.btnSelect) {
            case 0:
                dtostrf(batteryVoltage, 1, 1, buf0);
                snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField), PSTR("%sV"), buf0);
                break;
            case 1:
                dtostrf(yawDps, 3, 1, buf0);
                dtostrf(pitchDeg, 3, 1, buf1);
                dtostrf(rollDeg, 3, 1, buf2);
                snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField), PSTR("Y:%s;P:%s;R:%s"), buf0, buf1, buf2);
                break;
            case 2:
                dtostrf(yawSetpoint, 2, 2, buf0);
                dtostrf(pitchSetpoint, 2, 2, buf1);
                dtostrf(rollSetpoint, 2, 2, buf2);
                dtostrf(throttle, 2, 2, buf3);
                snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField), PSTR("joyY:%s;P:%s;R:%s;T:%d"), buf0, buf1, buf2, throttle);
                break;
            case 3:
                dtostrf(yawPID.GetPterm(), 1, 2, buf0);
                dtostrf(pitchPID.GetPterm(), 1, 2, buf1);
                dtostrf(rollPID.GetPterm(), 1, 2, buf2);
                snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField), PSTR("yP:%s;pP:%s;rP:%s"), buf0, buf1, buf2);
                break;
            case 4:
                dtostrf(pitchPID.GetIterm(), 1, 2, buf0);
                dtostrf(rollPID.GetIterm(), 1, 2, buf1);
                snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField), PSTR("pI:%s;rI:%s"), buf0, buf1);
                break;
            case 5:
                dtostrf(yawPID.GetDterm(), 1, 2, buf0);
                dtostrf(pitchPID.GetDterm(), 1, 2, buf1);
                dtostrf(rollPID.GetDterm(), 1, 2, buf2);
                snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField), PSTR("yD:%s;pD:%s;rD:%s"), buf0, buf1, buf2);
                break;
            case 6:
                dtostrf(pitchOutput, 1, 1, buf0);
                dtostrf(rollOutput, 1, 1, buf1);
                dtostrf(yawOutput, 1, 1, buf3);
                snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField), PSTR("outP:%s;R:%s;Y:%s"), buf0, buf1, buf3);
                break;
            case 9:
                if (RemoteXY.textField[0] != 0)
                    strcpy(RemoteXY.textField, "");
                break;
            default:
                snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField), PSTR("FL:%d;FR:%d;BL:%d;BR:%d"), FLspeed, FRspeed, BLspeed, BRspeed);
                break;
        }
    }

    computePID();
}

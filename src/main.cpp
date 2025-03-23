//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// #define REMOTEXY__DEBUGLOG
#define REMOTEXY_MODE__HARDSERIAL

#define REMOTEXY_SERIAL Serial
#define REMOTEXY_SERIAL_SPEED 115200

#include <RemoteXY.h>

#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =
{
  255,8,0,25,0,83,0,19,0,0,0,0,31,1,200,84,1,1,7,0,
  1,56,10,9,9,0,120,31,0,3,72,9,126,14,138,2,26,5,
  14,31,53,53,1,2,26,31,10,3,1,13,13,48,4,26,31,
  79,78,0,31,79,70,70,0,5,134,31,53,53,0,2,26,31,
  1,42,14,9,9,0,35,31,0,67,23,0,166,9,68,2,26,25
};

struct
{
  // Input variables
  uint8_t btnGreen;
  uint8_t btnSelect;
  int8_t  joyRP_x;
  int8_t  joyRP_y;
  uint8_t onOff;
  int8_t  joyYT_x;
  int8_t  joyYT_y;
  uint8_t btnRed;

  // Output variable
  char textField[25];

  // Other
  uint8_t connect_flag;

} RemoteXY;
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////

#include <Arduino.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <QuickPID.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

////////////////////////////////////////////////
//           GLOBAL CONSTANTS & DEFINES       //
////////////////////////////////////////////////

// Controls how much the PID parameters will be incremented/decremented
float pid_increment { 0.0f };

/*
 * Battery reading explanation:
 *   1) Read raw ADC from A0 (0..1023).
 *   2) Convert ADC value to voltage by multiplying with AREF_VOLTAGE_STEP.
 *   3) Because of the external voltage divider, multiply by VOLT_DIV_SCALING_FACTOR.
 *   4) Filter with alpha = 0.05 for noise reduction.
 */
const float BATTERY_FILTER_ALPHA   { 0.05f };
const float AREF_OFFSET            { -0.03f };
const float AREF_VOLTAGE           { 1.1f + AREF_OFFSET };
const float AREF_VOLTAGE_STEP      = AREF_VOLTAGE / 1023.0f;
const float VOLT_DIV_SCALING_FACTOR= (100.0f + 33.0f) / 33.0f;

/*
 * Motor PWM limits. If your ESCs won't spin the motor below 6 or if you want 
 * some margin, set MOTOR_PWM_MIN > 0. MOTOR_PWM_MAX = 255 for 8-bit output.
 */
static const int MOTOR_PWM_MIN { 6 };
static const int MOTOR_PWM_MAX { 255 };

/*
 * Battery failsafe threshold. If battery < CRITICAL_BATTERY_VOLTAGE, 
 * we forcibly disarm (if armed) or won't allow arming (if disarmed).
 */
static const float CRITICAL_BATTERY_VOLTAGE { 3.5f };

// How often to compute PID (microseconds)
static const uint32_t CALC_PID_DELTA_MICROS { 6000 };

// Separate pitch & roll I-term thresholds
static const int PITCH_ERROR_THRESHOLD { 20 };
static const int ROLL_ERROR_THRESHOLD  { 20 };

// PID tunings
float pr_p { 0.38f }, pr_i { 0.028f }, pr_d { 0.11f };  // Pitch & Roll
float y_p  { 0.65f }, y_i  { 0.0f   }, y_d  { 0.0f   };  // Yaw

static const float PR_OUTPUT_LIMITS { 20.0f };
static const float YAW_OUTPUT_LIMITS{ 20.0f };

// Joystick
float yaw_limit      { 0.7f  };
float pitch_limit    { 0.25f };
float roll_limit     { 0.25f };
float throttle_limit { 200.0f };
int   yaw_deadzone   { 15 };

// MPU calibration
static const int CAL_ITERATIONS { 10 }; // how many times to calibrate

////////////////////////////////////////////////
//             GLOBAL VARIABLES              //
////////////////////////////////////////////////

// Battery / timers
float g_battery_voltage { 0.0f };
unsigned long g_timer_battery  { 0 };
unsigned long g_timer_setpoint { 0 };
unsigned long g_timer_pid      { 0 };

// Arming
bool g_is_armed { false };

// Buffers for float-to-string
char g_buffer0[10];
char g_buffer1[10];
char g_buffer2[10];
char g_buffer3[10];

// Motor pins
const int PIN_FL_MOTOR { 3 };
const int PIN_FR_MOTOR { 11 };
const int PIN_BL_MOTOR { 9 };
const int PIN_BR_MOTOR { 10 };

// Motor speeds
int fl_speed { 0 };
int fr_speed { 0 };
int bl_speed { 0 };
int br_speed { 0 };

// Switches for turning off I-term
bool pitch_i_switch { false };
bool roll_i_switch  { false };

// Joystick setpoints
int   g_throttle     { 0 };
float yaw_setpoint   { 0.0f };
float pitch_setpoint { 0.0f };
float roll_setpoint  { 0.0f };
float yaw_output     { 0.0f };
float pitch_output   { 0.0f };
float roll_output    { 0.0f };
int   trim_pitch     { 0 };
int   trim_roll      { 0 };

// MPU / DMP
MPU6050 mpu;
bool     g_mpu_interrupt { false };
bool     g_dmp_ready     { false };
uint8_t  g_mpu_int_status;
uint8_t  g_dev_status;
uint16_t g_packet_size;
uint16_t g_fifo_count;
uint8_t  g_fifo_buffer[64];

// Orientation / motion
Quaternion  g_q;
VectorInt16 g_rot;
VectorInt16 g_aa;
VectorInt16 g_aa_real;
VectorInt16 g_aa_world;
VectorFloat g_gravity;
float       g_ypr[3];
float       g_pitch_deg { 0.0f };
float       g_roll_deg  { 0.0f };
float       g_yaw_dps   { 0.0f };

// QuickPID objects
QuickPID yaw_pid   (&g_yaw_dps,   &yaw_output,   &yaw_setpoint);
QuickPID pitch_pid (&g_pitch_deg, &pitch_output, &pitch_setpoint);
QuickPID roll_pid  (&g_roll_deg,  &roll_output,  &roll_setpoint);

////////////////////////////////////////////////
//          INTERRUPT SERVICE ROUTINE        //
////////////////////////////////////////////////
void on_dmp_data_ready()
{
  g_mpu_interrupt = true;
}

////////////////////////////////////////////////
//            CUSTOM HELPER FUNCTIONS        //
////////////////////////////////////////////////

/**
 * @brief Smooth the battery reading with a complementary filter.
 */
void update_battery_voltage()
{
  float battery_reading = analogRead(A0) * AREF_VOLTAGE_STEP * VOLT_DIV_SCALING_FACTOR;
  g_battery_voltage = g_battery_voltage * (1.0f - BATTERY_FILTER_ALPHA) 
                      + battery_reading * BATTERY_FILTER_ALPHA;
}

/**
 * @brief Calculate individual motor speeds by combining throttle & PID outputs,
 *        then constrain them to MOTOR_PWM_MIN..MOTOR_PWM_MAX.
 */
void calculate_motor_speed()
{
  if (g_is_armed)
  {
    fl_speed = g_throttle + pitch_output + roll_output + yaw_output;
    fr_speed = g_throttle + pitch_output - roll_output - yaw_output;
    bl_speed = g_throttle - pitch_output + roll_output - yaw_output;
    br_speed = g_throttle - pitch_output - roll_output + yaw_output;

    // Constrain using Arduino's constrain function
    fl_speed = constrain(fl_speed, MOTOR_PWM_MIN, MOTOR_PWM_MAX);
    fr_speed = constrain(fr_speed, MOTOR_PWM_MIN, MOTOR_PWM_MAX);
    bl_speed = constrain(bl_speed, MOTOR_PWM_MIN, MOTOR_PWM_MAX);
    br_speed = constrain(br_speed, MOTOR_PWM_MIN, MOTOR_PWM_MAX);
  }
  else
  {
    fl_speed = 0;
    fr_speed = 0;
    bl_speed = 0;
    br_speed = 0;
  }
}

/**
 * @brief Write current motor speeds to their pins.
 */
void set_motor_speed()
{
  analogWrite(PIN_FL_MOTOR, fl_speed);
  analogWrite(PIN_FR_MOTOR, fr_speed);
  analogWrite(PIN_BL_MOTOR, bl_speed);
  analogWrite(PIN_BR_MOTOR, br_speed);
}

/**
 * @brief Refresh setpoints (roll, pitch, yaw, throttle) from RemoteXY controls.
 */
void update_control_setpoints()
{
  if (millis() - g_timer_setpoint > 50)
  {
    roll_setpoint  = (RemoteXY.joyRP_x * roll_limit) + trim_roll;
    pitch_setpoint = (RemoteXY.joyRP_y * (-pitch_limit)) + trim_pitch;
    g_throttle     = map(RemoteXY.joyYT_y, -100, 100, 0, throttle_limit);

    if ((RemoteXY.joyYT_x <= -yaw_deadzone) || (RemoteXY.joyYT_x >= yaw_deadzone))
      yaw_setpoint = RemoteXY.joyYT_x * (-yaw_limit);
    else
      yaw_setpoint = 0;

    g_timer_setpoint = millis();
  }
}

/**
 * @brief Turn off pitch PID I-term when the pitch error is large.
 */
void pitch_i_term_switch(int max_err)
{
  float pitch_err = fabs(pitch_setpoint - g_pitch_deg);
  if (!pitch_i_switch && pitch_err > max_err)
  {
    pitch_pid.SetTunings(pr_p, 0.0f, pr_d);
    pitch_i_switch = true;
  }
  else if (pitch_i_switch && pitch_err <= max_err)
  {
    pitch_pid.SetTunings(pr_p, pr_i, pr_d);
    pitch_i_switch = false;
  }
}

/**
 * @brief Turn off roll PID I-term when the roll error is large.
 */
void roll_i_term_switch(int max_err)
{
  float roll_err = fabs(roll_setpoint - g_roll_deg);
  if (!roll_i_switch && roll_err > max_err)
  {
    roll_pid.SetTunings(pr_p, 0.0f, pr_d);
    roll_i_switch = true;
  }
  else if (roll_i_switch && roll_err <= max_err)
  {
    roll_pid.SetTunings(pr_p, pr_i, pr_d);
    roll_i_switch = false;
  }
}

/**
 * @brief Compute new PID outputs if enough time has elapsed.
 */
void compute_pid()
{
  if (micros() - g_timer_pid >= CALC_PID_DELTA_MICROS)
  {
    // Use separate thresholds for pitch & roll
    pitch_i_term_switch(PITCH_ERROR_THRESHOLD);
    roll_i_term_switch(ROLL_ERROR_THRESHOLD);

    yaw_pid.Compute();
    pitch_pid.Compute();
    roll_pid.Compute();

    calculate_motor_speed();
    set_motor_speed();
    g_timer_pid = micros();
  }
}

/**
 * @brief Adjust how much we increment/decrement PID parameters based on throttle.
 */
void set_pid_increments()
{
  if (RemoteXY.joyYT_y == 100)
    pid_increment = 1.0f;
  else if (RemoteXY.joyYT_y == -100)
    pid_increment = 0.001f;
  else
    pid_increment = 0.01f;
}

/**
 * @brief Read new MPU/DMP data if available, and filter yaw rate.
 */
void read_mpu_data()
{
  if (mpu.dmpGetCurrentFIFOPacket(g_fifo_buffer))
  {
    mpu.dmpGetQuaternion(&g_q, g_fifo_buffer);
    mpu.dmpGetGravity(&g_gravity, &g_q);
    mpu.dmpGetYawPitchRoll(g_ypr, &g_q, &g_gravity);
    mpu.dmpGetGyro(&g_rot, g_fifo_buffer);

    // Complementary filter for yaw DPS
    g_yaw_dps   = (g_yaw_dps * 0.8f) + (g_rot.z * 0.2f);
    g_pitch_deg = g_ypr[1] * RAD_TO_DEG;
    g_roll_deg  = g_ypr[2] * RAD_TO_DEG;
  }
}

/**
 * @brief Logic when the drone is disarmed (g_is_armed == false).
 * 
 * - We only arm if onOff == 1 AND the throttle is fully down.
 * - If battery is below CRITICAL_BATTERY_VOLTAGE, we refuse to arm.
 */
void handle_disarmed_state()
{
  // Hard failsafe: if battery below critical threshold, refuse to arm
  if (g_battery_voltage < CRITICAL_BATTERY_VOLTAGE)
  {
    RemoteXY.onOff = 0; // stay disarmed
    snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField),
               PSTR("Battery too low!"));
    return;
  }

  // Don’t allow arming if throttle isn’t fully down
  if (RemoteXY.onOff == 1)
  {
    if (RemoteXY.joyYT_y > -100)
    {
      RemoteXY.onOff = 0;
      return;
    }
    else
    {
      g_is_armed = true;
      // Re-enable full PID only upon arming
      yaw_pid.SetTunings(y_p, y_i, y_d);
      pitch_pid.SetTunings(pr_p, pr_i, pr_d);
      roll_pid.SetTunings(pr_p, pr_i, pr_d);
    }
  }

  // Battery low check (soft warning at e.g. 3.8f)
  if (g_battery_voltage > 1 && g_battery_voltage < 3.8f)
  {
    dtostrf(g_battery_voltage, 1, 1, g_buffer0);
    snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField),
               PSTR("%sV Battery low!"), g_buffer0);
  }
  else if (millis() - g_timer_battery > 100)
  {
    set_pid_increments();

    switch (RemoteXY.btnSelect)
    {
      case 0:
        dtostrf(g_battery_voltage, 1, 1, g_buffer0);
        snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField),
                   PSTR("%sV"), g_buffer0);
        break;

      case 1:
        snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField),
                   PSTR("trim pitch:%d"), trim_pitch);
        if (RemoteXY.btnGreen) ++trim_pitch;
        else if (RemoteXY.btnRed) --trim_pitch;
        break;

      case 2:
        snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField),
                   PSTR("trim roll:%d"), trim_roll);
        if (RemoteXY.btnGreen) ++trim_roll;
        else if (RemoteXY.btnRed) --trim_roll;
        break;

      case 3:
        dtostrf(pr_p, 1, 3, g_buffer0);
        snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField),
                   PSTR("prP:%s"), g_buffer0);
        if (RemoteXY.btnGreen) pr_p += pid_increment;
        else if (RemoteXY.btnRed) pr_p -= pid_increment;
        break;

      case 4:
        dtostrf(pr_i, 1, 3, g_buffer0);
        snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField),
                   PSTR("prI:%s"), g_buffer0);
        if (RemoteXY.btnGreen) pr_i += pid_increment;
        else if (RemoteXY.btnRed) pr_i -= pid_increment;
        break;

      case 5:
        dtostrf(pr_d, 1, 3, g_buffer0);
        snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField),
                   PSTR("prD:%s"), g_buffer0);
        if (RemoteXY.btnGreen) pr_d += pid_increment;
        else if (RemoteXY.btnRed) pr_d -= pid_increment;
        break;

      case 6:
        dtostrf(y_p, 1, 3, g_buffer0);
        snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField),
                   PSTR("yP:%s"), g_buffer0);
        if (RemoteXY.btnGreen) y_p += pid_increment;
        else if (RemoteXY.btnRed) y_p -= pid_increment;
        break;

      case 7:
        dtostrf(y_i, 1, 3, g_buffer0);
        snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField),
                   PSTR("yI:%s"), g_buffer0);
        if (RemoteXY.btnGreen) y_i += pid_increment;
        else if (RemoteXY.btnRed) y_i -= pid_increment;
        break;

      case 8:
        dtostrf(y_d, 1, 3, g_buffer0);
        snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField),
                   PSTR("yD:%s"), g_buffer0);
        if (RemoteXY.btnGreen) y_d += pid_increment;
        else if (RemoteXY.btnRed) y_d -= pid_increment;
        break;

      case 9:
        // Disallow negative Kp, Ki, Kd
        if (RemoteXY.btnGreen)
        {
          if (pr_p < 0) pr_p = 0.0f;
          if (pr_i < 0) pr_i = 0.0f;
          if (pr_d < 0) pr_d = 0.0f;
          if (y_p  < 0) y_p  = 0.0f;
          if (y_i  < 0) y_i  = 0.0f;
          if (y_d  < 0) y_d  = 0.0f;

          roll_pid.SetTunings(pr_p, pr_i, pr_d);
          pitch_pid.SetTunings(pr_p, pr_i, pr_d);
          yaw_pid.SetTunings(y_p, y_i, y_d);
          snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField),
                     PSTR("--OK--"));
        }
        else
        {
          snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField),
                     PSTR("Tune PID params?"));
        }
        break;

      default:
        // do nothing
        break;
    }
    g_timer_battery = millis();
  }
}

/**
 * @brief Logic when the drone is armed (g_is_armed == true).
 * 
 * - If pitch or roll angles exceed ±80°, or user toggles onOff=0, we disarm.
 * - If battery is below CRITICAL_BATTERY_VOLTAGE, also disarm immediately.
 */
void handle_armed_state()
{
  // Battery failsafe if below critical voltage
  if (g_battery_voltage < CRITICAL_BATTERY_VOLTAGE)
  {
    g_is_armed = false;
    RemoteXY.onOff = 0;
    snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField),
               PSTR("Battery too low!"));

    // Disable I term
    yaw_pid.SetTunings(y_p, 0.0f, y_d);
    pitch_pid.SetTunings(pr_p, 0.0f, pr_d);
    roll_pid.SetTunings(pr_p, 0.0f, pr_d);
    return;
  }

  // Disarm if overturning or onOff toggled off
  if (fabs(g_pitch_deg) > 80 || fabs(g_roll_deg) > 80 || RemoteXY.onOff == 0)
  {
    g_is_armed = false;
    RemoteXY.onOff = 0;
    // Disable I term
    yaw_pid.SetTunings(y_p, 0.0f, y_d);
    pitch_pid.SetTunings(pr_p, 0.0f, pr_d);
    roll_pid.SetTunings(pr_p, 0.0f, pr_d);
    return;
  }

  // Show data in textField
  switch (RemoteXY.btnSelect)
  {
    case 0:
      dtostrf(g_battery_voltage, 1, 1, g_buffer0);
      snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField),
                 PSTR("%sV"), g_buffer0);
      break;

    case 1:
      dtostrf(g_yaw_dps,    3, 1, g_buffer0);
      dtostrf(g_pitch_deg, 3, 1, g_buffer1);
      dtostrf(g_roll_deg,  3, 1, g_buffer2);
      snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField),
                 PSTR("Y:%s;P:%s;R:%s"), g_buffer0, g_buffer1, g_buffer2);
      break;

    case 2:
      dtostrf(yaw_setpoint,   2, 2, g_buffer0);
      dtostrf(pitch_setpoint, 2, 2, g_buffer1);
      dtostrf(roll_setpoint,  2, 2, g_buffer2);
      dtostrf(g_throttle,     2, 2, g_buffer3);
      snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField),
                 PSTR("joyY:%s;P:%s;R:%s;T:%d"), 
                 g_buffer0, g_buffer1, g_buffer2, g_throttle);
      break;

    case 3:
      dtostrf(yaw_pid.GetPterm(),   1, 2, g_buffer0);
      dtostrf(pitch_pid.GetPterm(), 1, 2, g_buffer1);
      dtostrf(roll_pid.GetPterm(),  1, 2, g_buffer2);
      snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField),
                 PSTR("yP:%s;pP:%s;rP:%s"), g_buffer0, g_buffer1, g_buffer2);
      break;

    case 4:
      dtostrf(pitch_pid.GetIterm(), 1, 2, g_buffer0);
      dtostrf(roll_pid.GetIterm(),  1, 2, g_buffer1);
      snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField),
                 PSTR("pI:%s;rI:%s"), g_buffer0, g_buffer1);
      break;

    case 5:
      dtostrf(yaw_pid.GetDterm(),   1, 2, g_buffer0);
      dtostrf(pitch_pid.GetDterm(), 1, 2, g_buffer1);
      dtostrf(roll_pid.GetDterm(),  1, 2, g_buffer2);
      snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField),
                 PSTR("yD:%s;pD:%s;rD:%s"), g_buffer0, g_buffer1, g_buffer2);
      break;

    case 6:
      dtostrf(pitch_output, 1, 1, g_buffer0);
      dtostrf(roll_output,  1, 1, g_buffer1);
      dtostrf(yaw_output,   1, 1, g_buffer3);
      snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField),
                 PSTR("outP:%s;R:%s;Y:%s"), g_buffer0, g_buffer1, g_buffer3);
      break;

    case 9:
      if (RemoteXY.textField[0] != 0)
        strcpy(RemoteXY.textField, "");
      break;

    default:
      snprintf_P(RemoteXY.textField, sizeof(RemoteXY.textField),
                 PSTR("FL:%d;FR:%d;BL:%d;BR:%d"), 
                 fl_speed, fr_speed, bl_speed, br_speed);
      break;
  }
}

////////////////////////////////////////////////
//                   SETUP                    //
////////////////////////////////////////////////
void setup()
{
  // Increase PWM frequencies:
  // Pins D9,D10 => Timer1
  TCCR1A = 0b00000001; // 8-bit
  TCCR1B = 0b00001001; // x1 fast PWM

  // Pins D3,D11 => Timer2
  TCCR2A = 0b00000011; // 8-bit
  TCCR2B = 0b00000001; // x1 fast PWM

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);

  pinMode(A0, INPUT);
  analogReference(INTERNAL);
  g_battery_voltage = 4.2f; // default starting voltage

  // Set PID modes & sample times
  roll_pid.SetMode(QuickPID::Control::timer);
  pitch_pid.SetMode(QuickPID::Control::timer);
  yaw_pid.SetMode(QuickPID::Control::timer);

  roll_pid.SetSampleTimeUs(CALC_PID_DELTA_MICROS);
  pitch_pid.SetSampleTimeUs(CALC_PID_DELTA_MICROS);
  yaw_pid.SetSampleTimeUs(CALC_PID_DELTA_MICROS);

  // PID output limits
  pitch_pid.SetOutputLimits(-PR_OUTPUT_LIMITS, PR_OUTPUT_LIMITS);
  roll_pid.SetOutputLimits(-PR_OUTPUT_LIMITS,  PR_OUTPUT_LIMITS);
  yaw_pid.SetOutputLimits(-YAW_OUTPUT_LIMITS,  YAW_OUTPUT_LIMITS);

  // PID tunings
  pitch_pid.SetTunings(pr_p, pr_i, pr_d);
  roll_pid.SetTunings(pr_p, pr_i, pr_d);
  yaw_pid.SetTunings(y_p,  y_i,  y_d);

  // Initialize MPU
  g_dev_status = mpu.dmpInitialize();

  // Replace these with your calibration offsets if needed:
  mpu.setXAccelOffset(127);
  mpu.setYAccelOffset(-2497);
  mpu.setZAccelOffset(1773);
  mpu.setXGyroOffset(-93);
  mpu.setYGyroOffset(16);
  mpu.setZGyroOffset(60);

  Serial.print(mpu.testConnection() ? F("MPU6050 connection successful")
                                    : F("MPU6050 connection failed"));

  if (g_dev_status == 0)
  {
    mpu.setRate(0);
    mpu.setDLPFMode(MPU6050_DLPF_BW_256);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);

    // If you want to calibrate the MPU, uncomment:
    // mpu.CalibrateAccel(CAL_ITERATIONS);
    // mpu.CalibrateGyro(CAL_ITERATIONS);

    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(2), on_dmp_data_ready, RISING);
    g_mpu_int_status = mpu.getIntStatus();
    g_dmp_ready      = true;
    g_packet_size    = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    Serial.print(F("DMP Initialization failed with code: "));
    Serial.print(g_dev_status);
    exit(g_dev_status);
  }

  Serial.end();
  RemoteXY_delay(20);

  // Initialize RemoteXY and set throttle to min
  RemoteXY_Init();
  RemoteXY.joyYT_y = -100;
  g_timer_battery  = millis();
  g_timer_setpoint = millis();
  g_timer_pid      = micros();
}

////////////////////////////////////////////////
//                    LOOP                    //
////////////////////////////////////////////////
void loop()
{
  if (!g_dmp_ready) return;

  // 1) Handle RemoteXY data
  RemoteXY_Handler();

  // 2) Read MPU data
  read_mpu_data();

  // 3) Update setpoints & battery
  update_control_setpoints();
  update_battery_voltage();

  // 4) PID computations
  compute_pid();

  // 5) Run logic depending on armed state
  if (!g_is_armed)
    handle_disarmed_state();
  else
    handle_armed_state();
}
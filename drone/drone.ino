#include <Wire.h>
#include <MPU6050_tockn.h>
#include <Adafruit_BMP280.h>
#include <PID_v1.h>

// ----- PIN CONSTANTS -----

const int IMU_SDA = 23;
const int IMU_SCL = 22;

const int BAR_SDA = 21;
const int BAR_SCL = 19;

const int MOTOR_PINS[4] = {
  32, // front left
  33, // front right
  25, // rear left
  26  // rear right
};

const int ARM_PIN = 27;

// ----- PWM CONSTANTS -----

const int PWM_FREQ = 400; // Hz
const int PWM_RES = 16;

// pulse width (μs)
const int PWM_MIN = 1000;
const int PWM_MAX = 2000;

// ----- PID CONSTANTS -----

// Minimum throttle that spins the motors
const double THROTTLE_MIN = 0.11;

// Minimum throttle that lifts the drone off the ground
// TODO: find this value
const double THROTTLE_HOVER = 0.2;

// TODO: tune PID constants, add trim if needed

// Attitude (pitch and roll) correction
const double KP_ATT = 1.0;
const double KI_ATT = 0.0;
const double KD_ATT = 0.01;
const double PID_ATT_LIMIT = THROTTLE_HOVER * 0.4;

// Altitude correction
const double KP_ALT = 1.0;
const double KI_ALT = 0.0;
const double KD_ALT = 0.01;
const double PID_ALT_LIMIT = THROTTLE_HOVER * 0.5;

const int LOOP_PERIOD = 4000; // μs, 250 Hz

// ----- SENSOR OBJECTS -----

TwoWire imu_i2c = TwoWire(0);
MPU6050 imu(imu_i2c);

TwoWire bar_i2c = TwoWire(1);
Adafruit_BMP280 bar(&bar_i2c);

double groundAltitude = 0.0;

// ----- PID OBJECTS -----

double pitchInput, pitchOutput, pitchSetpoint = 0.0;
double rollInput, rollOutput, rollSetpoint = 0.0;
double altInput, altOutput, altSetpoint = 0.0;

PID pidPitch(&pitchInput, &pitchOutput, &pitchSetpoint, KP_ATT, KI_ATT, KD_ATT, DIRECT);
PID pidRoll(&rollInput, &rollOutput, &rollSetpoint, KP_ATT, KI_ATT, KD_ATT, DIRECT);
PID pidAlt(&altInput, &altOutput, &altSetpoint, KP_ALT, KI_ALT, KD_ALT, DIRECT);

unsigned long lastLoop = 0;

// ----- CODE -----

void setup() {
  Serial.begin(115200);

  // Initialize ESCs
  Serial.println("Initializing ESCs");
  for (int i = 0; i < 4; i++) {
    ledcAttachChannel(MOTOR_PINS[i], PWM_FREQ, PWM_RES, i);
    setESC(i, 0.0);
  }
  delay(5000); // allow ESCs to arm

  // Initialize IMU
  Serial.println("Initializing IMU");
  imu_i2c.begin(IMU_SDA, IMU_SCL);
  imu.begin();

  // Calibrate IMU
  Serial.println("Calibrating IMU");
  imu.calcGyroOffsets();

  // Initialize barometer
  Serial.println("Initializing barometer");
  bar_i2c.begin(BAR_SDA, BAR_SCL);
  bar.begin(0x76);

  // Calibrate barometer
  Serial.println("Calibrating barometer");
  groundAltitude = bar.readAltitude();

  // Initialize PID
  Serial.println("Initializing PID");
  pidPitch.SetMode(AUTOMATIC);
  pidPitch.SetOutputLimits(-PID_ATT_LIMIT, PID_ATT_LIMIT);
  pidPitch.SetSampleTime(LOOP_PERIOD / 1000);
  pidRoll.SetMode(AUTOMATIC);
  pidRoll.SetOutputLimits(-PID_ATT_LIMIT, PID_ATT_LIMIT);
  pidRoll.SetSampleTime(LOOP_PERIOD / 1000);
  pidAlt.SetMode(AUTOMATIC);
  pidAlt.SetOutputLimits(-PID_ALT_LIMIT, PID_ALT_LIMIT);
  pidAlt.SetSampleTime(LOOP_PERIOD / 1000);
}

void loop() {
  // Ensure fixed-rate loop
  if (micros() - lastLoop >= LOOP_PERIOD) {
    lastLoop += LOOP_PERIOD;

    // imu.update performs attitude (pitch, roll, yaw) sensor fusion
    imu.update();

    // TODO: add altitude fusion or smoothing if needed

    // Compute PID outputs (note that this is angle only, not rate)
    pitchInput = imu.getAngleX();
    rollInput = imu.getAngleY();
    altInput = bar.readAltitude() - groundAltitude;
    pidPitch.Compute();
    pidRoll.Compute();
    pidAlt.Compute();

    // Motor mixing
    double baseThrottle = THROTTLE_HOVER + altOutput;
    setESC(0, constrain(baseThrottle + pitchOutput + rollOutput, THROTTLE_MIN, 1.0));  // front left
    setESC(1, constrain(baseThrottle + pitchOutput - rollOutput, THROTTLE_MIN, 1.0));  // front right
    setESC(2, constrain(baseThrottle - pitchOutput + rollOutput, THROTTLE_MIN, 1.0));  // rear left
    setESC(3, constrain(baseThrottle - pitchOutput - rollOutput, THROTTLE_MIN, 1.0));  // rear right

    // Print telemetry
    Serial.print("Pitch: "); Serial.print(imu.getAngleX());
    Serial.print("\tRoll: "); Serial.print(imu.getAngleY());
    Serial.print("\tYaw: "); Serial.print(imu.getAngleZ());
    Serial.print("\tAltitude: "); Serial.println(bar.readAltitude() - groundAltitude);
  }
}

void setESC(int motorIndex, double throttle) {
  int pulseWidth = PWM_MIN + (PWM_MAX - PWM_MIN) * throttle;

  double period = 1000000.0 / PWM_FREQ;
  uint32_t maxDutyCycle = (1 << PWM_RES) - 1;

  uint32_t dutyCycle = (pulseWidth / period) * maxDutyCycle;

  ledcWrite(MOTOR_PINS[motorIndex], dutyCycle);
}

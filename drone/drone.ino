#include <Wire.h>
#include <MPU6050_tockn.h>
#include <Adafruit_BMP280.h>

// ----- PIN CONSTANTS -----

const int IMU_SDA = 23;
const int IMU_SCL = 22;

const int BAR_SDA = 21;
const int BAR_SCL = 19;

const int MOTOR_PINS[4] = {
  32, // top left
  33, // top right
  25, // bottom left
  26  // bottom right
};

const int ARM_PIN = 27;

// ----- PWM CONSTANTS -----

const int PWM_FREQ = 400; // hz
const int PWM_RES = 16;

// pulse width (μs)
const int PWM_MIN = 1000;
const int PWM_MAX = 2000;

// ----- END CONSTANTS -----

TwoWire imu_i2c = TwoWire(0);
MPU6050 imu(imu_i2c);

TwoWire bar_i2c = TwoWire(1);
Adafruit_BMP280 bar(&bar_i2c);

float groundAltitude = 0.0;

void setup() {
  Serial.begin(115200);

  // Initialize ESCs
  Serial.println("Initializing ESCs");
  for (int i = 0; i < 4; i++) {
    ledcAttachChannel(MOTOR_PINS[i], PWM_FREQ, PWM_RES, i);
    setESC(i, 0.0);
  }
  delay(3000); // allow ESCs to arm

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

  // Test ESCs by ramping throttle up and down
  Serial.println("Testing ESCs...");
  for (float t = 0.0; t <= 0.3; t += 0.01) {
    for (int i = 0; i < 4; i++) setESC(i, t);
    delay(500);
  }
  for (float t = 0.3; t >= 0.0; t -= 0.01) {
    for (int i = 0; i < 4; i++) setESC(i, t);
    delay(500);
  }
}

void loop() {
  // imu.update performs attitude (pitch, roll, yaw) sensor fusion
  imu.update();

  // Print telemetry
  Serial.print("Pitch: ");
  Serial.print(imu.getAngleX());
  Serial.print("\t");
  Serial.print("Roll: ");
  Serial.print(imu.getAngleY());
  Serial.print("\t");
  Serial.print("Yaw: ");
  Serial.print(imu.getAngleZ());
  Serial.print("\t");
  Serial.print("Altitude: ");
  Serial.print(bar.readAltitude() - groundAltitude);
  Serial.println();
}

void setESC(int motorIndex, float throttle) {
  int pulseWidth = PWM_MIN + (PWM_MAX - PWM_MIN) * throttle;

  float period = 1000000.0 / PWM_FREQ;
  uint32_t maxDutyCycle = (1 << PWM_RES) - 1;

  uint32_t dutyCycle = (pulseWidth / period) * maxDutyCycle;

  ledcWrite(MOTOR_PINS[motorIndex], dutyCycle);

}

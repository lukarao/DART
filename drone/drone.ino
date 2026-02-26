#include <Wire.h>
#include <MPU6050_tockn.h>
#include <Adafruit_BMP280.h>

// ----- PIN CONSTANTS -----

const int IMU_SDA = 23;
const int IMU_SCL = 22;

const int BAR_SDA = 21;
const int BAR_SCL = 19;

const int MOTOR_PINS[4] = {
  32, // top right
  33, // top left
  25, // bottom right
  26  // bottom left
};

const int ARM_PIN = 27;

// ----- PWM CONSTANTS -----

const int PWM_FREQ = 400; // hz
const int PWM_RES = 16;

// pulse width (μs)
const int PWM_MIN = 1000; // might be 950
const int PWM_MAX = 2000;

// ----- END CONSTANTS -----

TwoWire imu_i2c = TwoWire(0);
MPU6050 imu(imu_i2c);

TwoWire bar_i2c = TwoWire(1);
Adafruit_BMP280 bar(&bar_i2c);

float groundAltitude = 0.0;

void setup() {
  Serial.begin(115200);

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

  // Initialize and calibrate ESCs
  Serial.println("Initializing and calibrating ESCs");
  for (int i = 0; i < 4; i++) {
    ledcAttachChannel(MOTOR_PINS[i], PWM_FREQ, PWM_RES, i);
    setESC(i, 1.0);
  }
  delay(2000);
  for (int i = 0; i < 4; i++) {
    setESC(i, 0.0);
  }
  delay(2000);

  // Test ESCs by ramping throttle up and down
  Serial.println("Testing ESCs...");
  for (float throttle = 0.0; throttle <= 1.0; throttle += 0.01) {
    for (int i = 0; i < 4; i++) {
      setESC(i, throttle);
    }
    delay(100);
  }
  for (float throttle = 1.0; throttle <= 0.0; throttle -= 0.01) {
    for (int i = 0; i < 4; i++) {
      setESC(i, throttle);
    }
    delay(100);
  }
  Serial.println("Testing ESCs...");
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
  int pulse_width = PWM_MIN + (int)((PWM_MAX - PWM_MIN) * throttle);
  uint32_t duty_cycle = (uint32_t)((float)pulse_width / 2500.0 * 65535.0);
  ledcWrite(MOTOR_PINS[motorIndex], duty_cycle);
}

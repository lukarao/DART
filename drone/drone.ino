#include <Wire.h>
#include <MPU6050_tockn.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_AHRS.h>
#include <SimpleKalmanFilter.h>

// ----- CONSTANTS -----

const int IMU_SDA = 23;
const int IMU_SCL = 22;

const int BAR_SDA = 21;
const int BAR_SCL = 19;

// ----- END CONSTANTS -----

TwoWire imu_i2c = TwoWire(0);
MPU6050 imu(imu_i2c);

TwoWire bar_i2c = TwoWire(1);
Adafruit_BMP280 bar(&bar_i2c);

Adafruit_Mahony ahrs;
SimpleKalmanFilter altKF(1, 1, 0.01);

float groundAltitude = 0.0;

void setup() {
  Serial.begin(115200);

  // Initialize IMU
  imu_i2c.begin(IMU_SDA, IMU_SCL);
  imu.begin();
  Serial.println("IMU initialized");

  // Calibrate IMU
  Serial.println("Calibrating IMU");
  imu.calcGyroOffsets();

  // Initialize barometer
  bar_i2c.begin(BAR_SDA, BAR_SCL);
  if (bar.begin(0x76)) {
    Serial.println("Barometer initialized");
  } else {
    Serial.println("Failed to initialize barometer");
  }

  // Calibrate barometer
  Serial.println("Calibrating barometer");
  groundAltitude = bar.readAltitude();
}

void loop() {
  imu.update();

  // Attitude (pitch, roll, yaw) sensor fusion
  ahrs.updateIMU(
    imu.getGyroX(), imu.getGyroY(), imu.getGyroZ(),
    imu.getAccX(), imu.getAccY(), imu.getAccZ()
  );

  // Altitude sensor fusion
  // TODO: fix drifting (tends to rise over time)
  float altitude = altKF.updateEstimate(bar.readAltitude() - groundAltitude);

  Serial.print("Pitch: ");
  Serial.print(ahrs.getPitch());
  Serial.print("\t");
  
  Serial.print("Roll: ");
  Serial.print(ahrs.getRoll());
  Serial.print("\t");

  Serial.print("Yaw: ");
  Serial.print(ahrs.getYaw());
  Serial.print("\t");

  Serial.print("Altitude: ");
  Serial.print(altitude);

  Serial.println();
}
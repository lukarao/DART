#include <Wire.h>
#include <MPU6050_tockn.h>
#include <Adafruit_BMP280.h>

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

  Serial.print("Accel: ");
  Serial.print(imu.getAccX());
  Serial.print("\t");
  Serial.print(imu.getAccY());
  Serial.print("\t");
  Serial.print(imu.getAccZ());

  Serial.print("\t");

  Serial.print("Gyro: ");
  Serial.print(imu.getGyroX());
  Serial.print("\t");
  Serial.print(imu.getGyroY());
  Serial.print("\t");
  Serial.print(imu.getGyroZ());

  Serial.print("\t");

  Serial.print(" Altitude: ");
  Serial.print(bar.readAltitude() - groundAltitude);

  Serial.println();
}
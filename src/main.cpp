//MPU6050 and Raspberry Pi Pico on Arduino
// Basic demo for accelerometer readings from Adafruit MPU6050
#include <cmath>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
struct Orientation {
    float roll, pitch, yaw;  // Angles in radians
};
class ComplementaryFilter {
public:
    ComplementaryFilter(float alpha) : alpha(alpha), dt(0.01) {} // You may need to adjust dt based on your sampling rate

    void update(sensors_event_t& acData , sensors_event_t &gyrdata ) {
        float accelRoll = atan2(acData.acceleration.y, acData.acceleration.z);
        float accelPitch = atan2(-acData.acceleration.x, sqrt(acData.acceleration.y * acData.acceleration.y + acData.acceleration.z * acData.acceleration.z));

        roll = alpha * (gyroRoll + gyrdata.gyro.x * dt) + (1 - alpha) * accelRoll;
        pitch = alpha * (gyroPitch + gyrdata.gyro.y * dt) + (1 - alpha) * accelPitch;
        yaw = gyroYaw + gyrdata.gyro.z * dt;  // Yaw is directly obtained from gyroscope

        // Normalize angles to the range [0, 2π]
        roll = fmod(roll + 2 * M_PI, 2 * M_PI)* (180.0 / M_PI);
        pitch = fmod(pitch + 2 * M_PI, 2 * M_PI)* (180.0 / M_PI);
        yaw = fmod(yaw + 2 * M_PI, 2 * M_PI)* (180.0 / M_PI);
    }

    Orientation getOrientation() const {
        return {roll, pitch, yaw};
    }

private:
    float alpha;  // Complementary filter coefficient
    float dt;     // Time step

    float gyroRoll = 0.0, gyroPitch = 0.0, gyroYaw = 0.0;  // Integrated gyro values
    float roll = 0.0, pitch = 0.0, yaw = 0.0;  // Current orientation
};

Adafruit_MPU6050 mpu;
TwoWire CustomI2C0(20, 21); //important
void setup(void) {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  }

  // Try to initialize! 0x68 addrs, custom i2c, sensor_id let it be 0(zero)
  if (!mpu.begin(0x68,&CustomI2C0,0)) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("");
  delay(100);
}

void loop() {

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

//   /* Print out the values */
//   Serial.print("AccelX:");
//   Serial.print(a.acceleration.x);
//   Serial.print(",");
//   Serial.print("AccelY:");
//   Serial.print(a.acceleration.y);
//   Serial.print(",");
//   Serial.print("AccelZ:");
//   Serial.print(a.acceleration.z);
//  Serial.print(", ");
//  Serial.print("GyroX:");
//  Serial.print(g.gyro.x);
//  Serial.print(",");
//  Serial.print("GyroY:");
//  Serial.print(g.gyro.y);
//  Serial.print(",");
//  Serial.print("GyroZ:");
//  Serial.print(g.gyro.z);
//   Serial.println("");
ComplementaryFilter filter(0.98);
filter.update(a,g);
Orientation orientation = filter.getOrientation();
String str = "Roll: " + String(orientation.roll, 2) + "\tPitch: " + String(orientation.pitch, 2) + "\tYaw: " + String(orientation.yaw, 2);
Serial.println(str);
  delay(10);
}
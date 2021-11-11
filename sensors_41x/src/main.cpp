#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}

bool noiseDetect() {
  return false;
}

/**
 * Checks temperature against upper and lower limits.
 *
 * @param temp current temperature reading
 * @return integer, 0 for normal temperature, 1 for low temperature, 2 for high temperature
 */
int tempDetect(float temp) {
  float lowerLimit = 18;  // https://www.labour.gov.on.ca/english/hs/faqs/workplace.php#temperature
  float upperLimit = 35;  // https://www.ccohs.ca/oshanswers/phys_agents/heat_health.html
  Serial.print("Temperature: ");
  Serial.print(temp);
  Serial.println(" degC");

  if (temp <= lowerLimit) {
    Serial.println("Temperature too cold");
    return 1;
  }
  else if(temp >= upperLimit) {
    Serial.println("Temperature too hot");
    return 2;
  }
  return 0;
}

/**
 * Detects falls using x, y and z accelerations and gyro data.
 *
 * @param event struct holding sensor readings
 * @return boolean, true if a fall has been detected.
 */
bool fallDetect(sensors_event_t  *event) {
  float magnitude = 0;
  float lastReading = 0;
  float high_threshhold = 12;
  float changeAcc = 0;
  // change in acceleration, hold one reading and compare or look at mulitple readings for trend

  magnitude = sqrt(sq(event->acceleration.x) +sq(event->acceleration.y) + sq(event->acceleration.z));
  changeAcc = abs(magnitude - lastReading);

  if(magnitude > high_threshhold) {
    Serial.println("Fall Detected");
    Serial.println("Change in Acceleration");
    Serial.print(changeAcc);
    Serial.println(" m/s^2");
    return true;
  }
  Serial.print("Magnitude of Acceleration: ");
  Serial.print(magnitude);
  Serial.println(" m/s^2");
  lastReading = magnitude;
  return false;
}

void checkSensors() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  fallDetect(&a);
  tempDetect(temp.temperature);
}

void loop() {
  checkSensors();
  delay(500);
  
}

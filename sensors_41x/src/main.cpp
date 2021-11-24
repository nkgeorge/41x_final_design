#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h> 

Adafruit_MPU6050 mpu;
int sound_digital = 4;
int sound_analog = 0;

bool setup_mpu() {
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    delay(100);
    return false;
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
  return true;
}

void setup(void) {
  Serial.begin(115200);
  setup_mpu();

  // Initialize pins for sound sensor
  pinMode(sound_digital, INPUT);
  delay(100);
}

bool noiseDetect() {
  int val_digital = digitalRead(sound_digital);
  int val_analog = analogRead(sound_analog);
  // Serial.print(val_analog);
  // Serial.print("\t");
  // Serial.println(val_digital);

  if (val_digital == HIGH)
  {
    Serial.println("Noise level high");
    return true;
  }
  Serial.println(val_analog);
  return false;
}
bool compareFloats (float a, float b, float EPSILON) {
  return fabs(a - b) < EPSILON;
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
  float init_val = 36.53;
  Serial.print("Temperature: ");
  Serial.print(temp);
  Serial.println(" degC");

  if (temp <= lowerLimit) {
    Serial.println("Temperature too cold");
    return 1;
  }
  else if (compareFloats(temp, init_val, 0.001)) { //  if nothing is read from MPU6050
    return 3;
  }
  else if (temp >= upperLimit) {
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
bool fallDetect(sensors_event_t  *event_a, sensors_event_t  *event_g) {
  float acc_magnitude, g_magnitude = 0;
  float acc_lastReading, g_lastReading = 0;
  float high_threshhold = 20; // change in acc threshold
  float changeAcc = 0;
  // change in acceleration, hold one reading and compare or look at mulitple readings for trend
  // accelerometer gives around 11m/s^2 for sitting down
  acc_magnitude = sqrt(sq(event_a->acceleration.x) +sq(event_a->acceleration.y) + sq(event_a->acceleration.z));
  g_magnitude = sqrt(sq(event_g->gyro.x) +sq(event_g->gyro.y) + sq(event_g->gyro.z));
  // changeAcc = abs(acc_magnitude - acc_lastReading);

  if (changeAcc > high_threshhold) {
    Serial.println("Fall Detected");
    Serial.println("Change in Acceleration");
    Serial.print(changeAcc);
    return true;
  }

  Serial.print("Magnitude of Acceleration: ");
  Serial.println(acc_magnitude);
  acc_lastReading = acc_magnitude;

  Serial.print("Magnitude of Angular Velocity: ");
  Serial.println(g_magnitude);
  g_lastReading = g_magnitude;
  return false;
}

void checkSensors() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  fallDetect(&a, &g);

  if (tempDetect(temp.temperature) == 3) {
    while(!setup_mpu());
  }

  // noiseDetect();
}

void loop() {
  checkSensors();
  delay(500);
}

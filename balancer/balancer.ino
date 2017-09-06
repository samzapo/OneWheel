
#include <Wire.h>
#include <Servo.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <BasicLinearAlgebra.h>

// Set the delay between fresh samples
#define BNO055_SAMPLERATE_DELAY_MS (20)
#define DISPLAY_VALUES 1

// Servo controller
Servo myservo_forward;
Servo myservo_turn;

Adafruit_BNO055 bno = Adafruit_BNO055();

void setup(void)
{
  Serial.begin(9600);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  // Initialise the sensor
  if (!bno.begin())
  {
    // There was a problem detecting the BNO055 ... check your connections
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  myservo_forward.attach(7);
  myservo_turn.attach(6);

  delay(1000);

  // Display the current temperature
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
}

void loop(void)
{
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2

  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

  // Quaternion data
  imu::Quaternion quat = bno.getQuat();

  if (DISPLAY_VALUES) {
    // Display the floating point data
    Serial.print("GYRO ");
    Serial.print("X: ");
    Serial.print(gyro.x());
    Serial.print(" Y: ");
    Serial.print(gyro.y());
    Serial.print(" Z: ");
    Serial.print(gyro.z());
    Serial.print("\t\t");

    Serial.print("ACCEL ");
    Serial.print("X: ");
    Serial.print(accel.x());
    Serial.print(" Y: ");
    Serial.print(accel.y());
    Serial.print(" Z: ");
    Serial.print(accel.z());
    Serial.print("\t\t");

    Serial.print("GRAV ");
    Serial.print("X: ");
    Serial.print(grav.x());
    Serial.print(" Y: ");
    Serial.print(grav.y());
    Serial.print(" Z: ");
    Serial.print(grav.z());
    Serial.print("\t\t");

    Serial.print("qW: ");
    Serial.print(quat.w(), 4);
    Serial.print(" qX: ");
    Serial.print(quat.y(), 4);
    Serial.print(" qY: ");
    Serial.print(quat.x(), 4);
    Serial.print(" qZ: ");
    Serial.print(quat.z(), 4);
    Serial.print("\t\t");

    //  Display calibration status for each sensor.
    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    Serial.print("CALIBRATION: Sys=");
    Serial.print(system, DEC);
    Serial.print(" Gyro=");
    Serial.print(gyro, DEC);
    Serial.print(" Accel=");
    Serial.print(accel, DEC);
    Serial.print(" Mag=");
    Serial.print(mag, DEC);
    Serial.println();
  }

  // acceleration vector
  Matrix<3, 1> acceleration;
  acceleration(0) = accel.x();
  acceleration(1) = accel.y();
  acceleration(2) = accel.z();

  // angular rate matrix
  Matrix<3, 3> gyro_w;
  gyro_w.Fill(0);
  // W_z
  gyro_w(1, 0) = gyro.z();
  gyro_w(0, 1) = -gyro.z();
  // W_x
  gyro_w(2, 0) = -gyro.y();
  gyro_w(0, 2) = gyro.y();
  // W_y
  gyro_w(1, 2) = gyro.x();
  gyro_w(2, 1) = -gyro.x();

  double forward_command = 0, turn_command = 0;

  ////// BALANCING CODE HERE ///////

  forward_command = 90 + grav.x()*10;
  
  // calculate balance control
  myservo_forward.write(forward_command);
  myservo_turn.write(turn_command);


  delay(BNO055_SAMPLERATE_DELAY_MS);
}

#include <Arduino.h>

#include <Wire.h>
#include <SparkFun_BMP581_Arduino_Library.h>
#include <SparkFun_BNO08x_Arduino_Library.h> // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BNO08x

BMP581 pressureSensor;
BNO08x bno08x;

uint8_t i2cAddress = BMP581_I2C_ADDR;

// Constants
const float g = 9.81;  // Gravitational accelerationear (m/s^2). IMU should already account for this. !!! check
const float Cd = 0.75; // Drag coefficient (assumed constant)
const float A = 0.01;  // Cross-sectional area of the rocket (m^2)
const float m = 1.0;   // Mass of the rocket (kg)

// Air density at sea level (kg/m^3) - adjust for altitude later if necessary
float airDensity(float altitude)
{
  return 1.225 * exp(-altitude / 8500); // Simplified model
}

// Drag force calculation
float dragForce(float velocity, float altitude)
{
  return 0.5 * airDensity(altitude) * velocity * velocity * Cd * A;
}

// State variables
float z = 0.0;                    // Altitude (m)
float v = 0.0;                    // Velocity (m/s)
float a = 0.0;                    // Acceleration (m/s^2)
float P[2][2] = {{1, 0}, {0, 1}}; // Covariance matrix

// Process noise covariance
float Q[2][2] = {{0.25, 0}, {0, 0.25}};

// Measurement noise covariance
float R[1][1] = {{0.5}}; // Only for altitude

// Time step
float dt = 0.01;

// EKF Prediction Step with Drag
void predict(float *z, float *v, float a_meas, float P[2][2], float dt)
{
  // Compute drag force based on current velocity and altitude
  float F_drag = dragForce(*v, *z);
  float a_drag = F_drag / m; // Drag acceleration

  // Compute the total acceleration
  float a_total = a_meas - g - a_drag;

  // Update velocity based on the total acceleration (from the accelerometer and drag)
  *v = *v + a_total * dt;

  // Update altitude based on the predicted velocity and current total acceleration
  *z = *z + *v * dt + 0.5 * a_total * dt * dt;

  // Update process covariance matrix (P) - simplified for the state [z, v]
  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      P[i][j] += Q[i][j]; // Adding process noise
    }
  }
}

// EKF Update Step
void update(float *z, float *v, float P[2][2], float z_meas, float R[1][1])
{
  // Measurement matrix H (1x2) for altitude measurement
  float H[1][2] = {{1, 0}}; // We only use altitude measurement here

  // Predicted measurement (altitude)
  float z_pred = *z;

  // Innovation (residual) between the predicted and actual measurements
  float y = z_meas - z_pred;

  // Compute innovation covariance (S = H * P * H^T + R)
  float S = 0;
  for (int k = 0; k < 2; k++)
  {
    S += H[0][k] * P[k][k] * H[0][k];
  }
  S += R[0][0];

  // Compute Kalman gain (K = P * H^T * S^-1)
  float K[2] = {0};
  for (int i = 0; i < 2; i++)
  {
    K[i] = P[i][i] * H[0][i] / S;
  }

  // Update the state estimate (altitude and velocity)
  *z += K[0] * y;
  *v += K[1] * y; // Velocity correction

  // Update the error covariance matrix (P)
  float I[2][2] = {{1, 0}, {0, 1}}; // Identity matrix

  float KH[2][2] = {0};
  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      KH[i][j] = K[i] * H[0][j];
    }
  }

  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      P[i][j] = (I[i][j] - KH[i][j]) * P[i][j];
    }
  }
}

void setup()
{
  Serial.begin(115200);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  while (pressureSensor.beginI2C(i2cAddress) != BMP5_OK)
  {
    // Not connected, inform user
    Serial.println("Error: BMP581 not connected, check wiring and I2C address!");
    delay(1000);
  }
  Serial.println("BMP581 found!");

  if (bno08x.begin(BNO08X_I2C_ADDR, Wire, -1, BNO08X_NRST))
  {
    bno08x.enableAccelerometer();
  }
  Serial.println("BNO08x found!");
}

void loop()
{

  // Get measurements from the sensor
  bmp5_sensor_data data = {0, 0};
  int8_t err = pressureSensor.getSensorData(&data);

  // Simulated inputs
  float z_meas = 0; // Example barometer altitude measurement (meters)
  float a_meas = 0; // Example accelerometer vertical acceleration (m/s^2)

  // Check whether data was acquired successfully
  if (err == BMP5_OK)
  {
    float pressure_hPa = data.pressure / 100;
    z_meas = (1 - powf(pressure_hPa / 1013.25, 0.190284)) * 44307.69;
  }

  // Has a new event come in on the Sensor Hub Bus?
  if (bno08x.getSensorEvent() == true)
  {
    // is it the correct sensor data we want?
    if (bno08x.getSensorEventID() == SENSOR_REPORTID_ACCELEROMETER)
    {

      // float x = myIMU.getAccelX();
      // float y = myIMU.getAccelY();
      a_meas = bno08x.getAccelZ();
      Serial.print("accel: ");

      Serial.println(a_meas);
    }
  }

  // Prediction step using the measured acceleration from the IMU
  predict(&z, &v, a_meas, P, dt);

  // Update step using the altitude measurement from the barometer
  update(&z, &v, P, z_meas, R);

  // Print estimated state (altitude and velocity)
  Serial.print("Estimated altitude: ");
  Serial.println(z);
  Serial.print("Estimated velocity: ");
  Serial.println(v);

  delay(100); // Delay for demonstration
}
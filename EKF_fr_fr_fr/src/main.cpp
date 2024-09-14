#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_BMP581_Arduino_Library.h>
#include <SparkFun_BNO08x_Arduino_Library.h>

BMP581 pressureSensor;
BNO08x bno08x;

uint8_t i2cAddress = BMP581_I2C_ADDR;

// Constants
const float g = 9.80665; // Gravitational acceleration (m/s^2)
const float Cd = 0.75;   // Drag coefficient
const float A = 0.01;    // Cross-sectional area (m^2)
const float m = 1.0;     // Mass of the rocket (kg)
const float dt = 0.01;   // Time step (s)

// Air density as a function of altitude
float airDensity(float altitude)
{
  return 1.225 * exp(-altitude / 8500.0); // Exponential decrease with altitude
}

// Drag acceleration calculation
float dragAcceleration(float velocity, float altitude)
{
  float rho = airDensity(altitude);
  float v_squared = velocity * velocity;
  float drag_force = 0.5 * rho * v_squared * Cd * A;
  float drag_acc = drag_force / m;
  // Drag opposes motion
  return (velocity > 0) ? -drag_acc : drag_acc;
}

// Extended Kalman Filter Class
class ExtendedKalmanFilter
{
public:
  // State vector [altitude, velocity]
  float x[2];
  // Covariance matrix
  float P[2][2];
  // Process noise covariance
  float Q[2][2];
  // Measurement noise covariance
  float R;

  ExtendedKalmanFilter()
  {
    // Initialize state and covariance
    x[0] = 0.0; // Initial altitude
    x[1] = 0.0; // Initial velocity

    P[0][0] = 1.0;
    P[0][1] = 0.0;
    P[1][0] = 0.0;
    P[1][1] = 1.0;

    Q[0][0] = 0.1;
    Q[0][1] = 0.0;
    Q[1][0] = 0.0;
    Q[1][1] = 0.1;

    R = 0.5; // Measurement noise covariance
  }

  void predict(float a_total)
  {
    // Predict the next state
    float x_pred[2];
    x_pred[1] = x[1] + a_total * dt;
    x_pred[0] = x[0] + x[1] * dt + 0.5 * a_total * dt * dt;

    // Compute Jacobian F_k
    float rho = airDensity(x[0]);
    float v = x[1];
    float drag_coefficient = (rho * Cd * A) / m;

    // Derivative of drag acceleration with respect to velocity
    float da_drag_dv = -drag_coefficient * v;

    // State transition matrix F
    float F[2][2];
    F[0][0] = 1.0;
    F[0][1] = dt;
    F[1][0] = 0.0;
    F[1][1] = 1.0 + da_drag_dv * dt;

    // Predict covariance matrix P_pred = F * P * F^T + Q
    float P_pred[2][2];
    // First compute F * P
    float FP[2][2];
    FP[0][0] = F[0][0] * P[0][0] + F[0][1] * P[1][0];
    FP[0][1] = F[0][0] * P[0][1] + F[0][1] * P[1][1];
    FP[1][0] = F[1][0] * P[0][0] + F[1][1] * P[1][0];
    FP[1][1] = F[1][0] * P[0][1] + F[1][1] * P[1][1];
    // Then compute F * P * F^T
    P_pred[0][0] = FP[0][0] * F[0][0] + FP[0][1] * F[0][1] + Q[0][0];
    P_pred[0][1] = FP[0][0] * F[1][0] + FP[0][1] * F[1][1] + Q[0][1];
    P_pred[1][0] = FP[1][0] * F[0][0] + FP[1][1] * F[0][1] + Q[1][0];
    P_pred[1][1] = FP[1][0] * F[1][0] + FP[1][1] * F[1][1] + Q[1][1];

    // Update state and covariance
    x[0] = x_pred[0];
    x[1] = x_pred[1];
    P[0][0] = P_pred[0][0];
    P[0][1] = P_pred[0][1];
    P[1][0] = P_pred[1][0];
    P[1][1] = P_pred[1][1];
  }

  void update(float z_meas)
  {
    // Measurement residual
    float y = z_meas - x[0];

    // Measurement covariance
    float S = P[0][0] + R;

    // Kalman gain
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Update state estimate
    x[0] = x[0] + K[0] * y;
    x[1] = x[1] + K[1] * y;

    // Update covariance matrix
    float P_new[2][2];
    P_new[0][0] = (1.0 - K[0]) * P[0][0];
    P_new[0][1] = (1.0 - K[0]) * P[0][1];
    P_new[1][0] = P[1][0] - K[1] * P[0][0];
    P_new[1][1] = P[1][1] - K[1] * P[0][1];

    // Copy P_new back to P
    P[0][0] = P_new[0][0];
    P[0][1] = P_new[0][1];
    P[1][0] = P_new[1][0];
    P[1][1] = P_new[1][1];
  }

  float getAltitude()
  {
    return x[0];
  }

  float getVelocity()
  {
    return x[1];
  }
};

// Global instance of the EKF
ExtendedKalmanFilter ekf;

// Function to initialize sensors
void initializeSensors()
{
  // Initialize barometer
  if (pressureSensor.beginI2C(i2cAddress) != BMP5_OK)
  {
    Serial.println("Error: BMP581 not connected, check wiring!");
    while (1)
      ;
  }
  Serial.println("BMP581 found!");

  // Initialize IMU
  if (!bno08x.begin())
  {
    Serial.println("Error: BNO08x not connected, check wiring!");
    while (1)
      ;
  }
  bno08x.enableAccelerometer(100); // Enable accelerometer at 100Hz
  Serial.println("BNO08x found!");
}

// Function to get barometer measurement
bool getBarometerMeasurement(float &z_meas)
{
  bmp5_sensor_data data = {0, 0};
  if (pressureSensor.getSensorData(&data) == BMP5_OK)
  {
    float pressure_Pa = data.pressure;
    float pressure_hPa = pressure_Pa / 100.0;
    z_meas = (1.0 - powf(pressure_hPa / 1013.25, 0.190284)) * 44307.69;
    return true;
  }
  else
  {
    Serial.println("Error reading barometer data");
    return false;
  }
}

// Function to get accelerometer measurement
bool getAccelerometerMeasurement(float &a_meas)
{
  if (bno08x.getSensorEvent())
  {
    if (bno08x.getSensorEventID() == SENSOR_REPORTID_ACCELEROMETER)
    {
      a_meas = bno08x.getAccelZ(); // Assuming Z is the vertical axis
      return true;
    }
  }
  Serial.println("Error reading accelerometer data");
  return false;
}

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  initializeSensors();
}

void loop()
{
  static unsigned long lastTime = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= dt * 1000)
  {
    lastTime = currentTime;

    float z_meas = 0.0;
    float a_meas = 0.0;

    // Get sensor measurements
    bool baroSuccess = getBarometerMeasurement(z_meas);
    bool accelSuccess = getAccelerometerMeasurement(a_meas);

    if (!baroSuccess || !accelSuccess)
    {
      // Skip this iteration if sensor data is invalid
      return;
    }

    // Correct accelerometer measurement for gravity
    float a_actual = a_meas + g;

    // Compute drag acceleration
    float a_drag = dragAcceleration(ekf.x[1], ekf.x[0]);

    // Total acceleration
    float a_total = a_actual + a_drag;

    // EKF prediction and update steps
    ekf.predict(a_total);
    ekf.update(z_meas);

    // Print estimated state
    Serial.print("Estimated Altitude: ");
    Serial.print(ekf.getAltitude());
    Serial.print(" m, Estimated Velocity: ");
    Serial.print(ekf.getVelocity());
    Serial.println(" m/s");
  }
}

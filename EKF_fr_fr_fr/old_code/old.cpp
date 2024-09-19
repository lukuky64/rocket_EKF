#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_BMP581_Arduino_Library.h>
#include <SparkFun_BNO08x_Arduino_Library.h>

#include <EKF.hpp>

BMP581 pressureSensor;
BNO08x bno08x;

uint8_t i2cAddress = BMP581_I2C_ADDR;

EKF ekf;

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
    if (!bno08x.begin(BNO08X_I2C_ADDR, Wire, -1, BNO08X_NRST))
    {
        Serial.println("Error: BNO08x not connected, check wiring!");
        while (1)
            ;
    }
    bno08x.enableAccelerometer(10); // Enable accelerometer at 100Hz
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
    delay(500);
    Serial.println("Rocket Altitude Estimation");
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000);

    initializeSensors();
}

void loop()
{
    static unsigned long lastTime = 0;
    unsigned long currentTime = millis();
    if (currentTime - lastTime >= ekf.getDt() * 1000)
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

        // Serial.print("Barometer Measurement: ");
        // Serial.print(z_meas);

        // Serial.print(" Accelerometer Measurement: ");
        // Serial.println(a_meas);

        // EKF prediction and update steps
        ekf.iterate(a_meas, z_meas);

        // Print estimated state
        Serial.print("Estimated Altitude: ");
        Serial.print(ekf.getAltitude());
        Serial.print(" m, Estimated Velocity: ");
        Serial.print(ekf.getVelocity());
        Serial.println(" m/s");
    }
}

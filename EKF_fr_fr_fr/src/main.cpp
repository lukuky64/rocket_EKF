#include <Arduino.h>
#include <Wire.h>
#include <EKF.hpp>

float dt = 0.05; // Time step (s)

EKF ekf(dt);

bool getMeasurement(float &z_meas, float &a_meas)
{
  if (Serial.available())
  {
    String dataString = Serial.readStringUntil('\n');
    if (dataString.length() > 0)
    {
      int commaIndex = dataString.indexOf(',');
      if (commaIndex > 0)
      {
        String zString = dataString.substring(0, commaIndex);
        String aString = dataString.substring(commaIndex + 1);

        z_meas = zString.toFloat();
        a_meas = aString.toFloat();
        return true;
      }
    }
  }
  return false;
}

void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println("Rocket Altitude Estimation");

  // Flush any existing data
  while (Serial.available() > 0)
  {
    Serial.read();
  }
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

    bool newData = getMeasurement(z_meas, a_meas);

    if (!newData)
    {
      // Skip this iteration if sensor data is invalid
      return;
    }

    // EKF prediction and update steps
    ekf.iterate(a_meas, z_meas);

    // print estimates
    Serial.print(ekf.getAltitude());
    Serial.print(", ");
    Serial.println(ekf.getVelocity());
  }
}
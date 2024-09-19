#include <Arduino.h>
#include <Model.hpp>

Model::Model()
{
    // Constants
    g = 9.81; // Gravitational acceleration (m/s^2)
    Cd = 0.5; // Drag coefficient
    A = 0.02; // Cross-sectional area (m^2)
    m = 20.0; // Mass of the rocket (kg)
}

// Air density as a function of altitude
float Model::airDensity(float altitude)
{
    return 1.225 * exp(-altitude / 8500.0); // Exponential decrease with altitude
}

// Drag acceleration calculation
float Model::dragAcceleration(float velocity, float altitude)
{
    float rho = airDensity(altitude);
    float v_squared = velocity * velocity;
    float drag_force = 0.5 * rho * v_squared * Cd * A;
    float drag_acc = drag_force / m;
    // Drag opposes motion
    return (velocity > 0) ? -drag_acc : drag_acc;
}

float Model::getAccTotal(float a_meas, float state[])
{
    // Correct accelerometer measurement for gravity
    float a_actual = a_meas; // a_meas- g //                 !!! we need to only include this when the rocket is on pad

    // Compute drag acceleration
    float a_drag = 0; // dragAcceleration(state[1], state[0]);

    // Total acceleration
    float a_total = a_actual + a_drag;

    return a_total;
}

float Model::getDragCoefficient(float rho)
{
    return (rho * Cd * A) / m;
}

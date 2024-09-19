#include <Arduino.h>
#include <EKF.hpp>
// Extended Kalman Filter Class
EKF::EKF(float dt_)
{
    dt = dt_; // Time step (s)

    // Initialize state and covariance
    x[0] = 0.0;   // Initial altitude
    x[1] = 196.2; // Initial velocity

    P[0][0] = 0.5; // altitude variance. Setting this low because we know the initial altitude
    P[0][1] = 0.0;
    P[1][0] = 0.0;
    P[1][1] = 0.5; // velocity variance. Setting this low because we know the initial altitude

    Q[0][0] = 0.001; // should reflect how much we trust the model to estimate altitude. Smaller Q means we trust the model more
    Q[0][1] = 0.0;
    Q[1][0] = 0.0;
    Q[1][1] = 0.001; // should reflect how much we trust the model to estimate velocity. Smaller Q means we trust the model more

    R = 25; // Measurement noise covariance. Should be tuned based on barometer variance. Smaller R means we trust the sensor more
            // 0.5 is probably accuracy but im going to increase it because there are stages in flight where barometer will give weird readings.
}

void EKF::predict(float a_total)
{
    // Predict the next state
    float x_pred[2];
    x_pred[1] = x[1] + a_total * dt;
    x_pred[0] = x[0] + x[1] * dt + 0.5 * a_total * dt * dt;

    // Compute Jacobian F_k
    float rho = model.airDensity(x[0]);
    float drag_coefficient = model.getDragCoefficient(rho);

    // Derivative of drag acceleration with respect to velocity
    float v = x[1];
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

void EKF::update(float z_meas)
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

float EKF::getAltitude()
{
    return x[0];
}

float EKF::getVelocity()
{
    return x[1];
}

void EKF::iterate(float a_meas, float z_meas)
{
    float a_total = model.getAccTotal(a_meas, x);
    predict(a_total);
    update(z_meas);
}

float EKF::getDt()
{
    return dt;
}

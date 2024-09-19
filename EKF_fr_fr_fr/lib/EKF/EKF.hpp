#ifndef EKF_HPP_
#define EKF_HPP_

#include "Model.hpp"

class EKF
{
private:
    // create a Model object
    Model model;

    // State vector [altitude, velocity]
    float x[2];
    // Covariance matrix
    float P[2][2];
    // Process noise covariance
    float Q[2][2];
    // Measurement noise covariance
    float R;

    float dt; // Time step (s)

public:
    EKF(float dt_);
    ~EKF();
    void iterate(float a_meas, float z_meas);
    void update(float z_meas);
    void predict(float a_meas);
    float getAltitude();
    float getVelocity();
    float getDt();
};

#endif // MODEL_HPP_
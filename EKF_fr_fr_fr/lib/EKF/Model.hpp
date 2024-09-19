#ifndef MODEL_HPP_
#define MODEL_HPP_

class Model
{
private:
    float g;  // Gravitational acceleration (m/s^2)
    float Cd; // Drag coefficient
    float A;  // Cross-sectional area (m^2)
    float m;  // Mass of the rocket (kg)
public:
    Model(/* args */);
    ~Model();
    float airDensity(float altitude);
    float dragAcceleration(float velocity, float altitude);
    float getAccTotal(float a_meas, float state[]);
    float getDragCoefficient(float rho);
};

#endif // MODEL_HPP_
#include "PID.h"
#include <cmath>

PIDController::PIDController()
{
    integrator_ = 0.0;
    prevError_ = 0.0;

    differentiator_ = 0.0;
    prevMeasurement_ = 0.0;
    
    e0_ = e1_ = e2_ = 0.0;
    y0_ = y1_ = y2_ = 0.0;
    
    output_ = 0.0;
}

float PIDController::computeControlSignal(float setpoint, float measurement)
{
    // Error signal
    float currentError = setpoint - measurement;

    // Proportional
    proportional_ = Kp_ * currentError;

    /**************************************************************************/
    // Integral (trapezoidal calculation)
    integrator_ = integrator_ + 0.5 * Ki_ * T_ * (currentError + prevError_);

    // Anti wind-up via integrator clamping
    if (integrator_ >= integratorSatUpper_)
        integrator_ = integratorSatUpper_;
    else if (integrator_ < integratorSatLower_)
        integrator_ = integratorSatLower_;

    /**************************************************************************/
    // Derivative (without derivative filter - will be implemented soon)
    int dError = (currentError - prevError_) / T_;
    differentiator_ = Kd_ * dError;
    
    /**************************************************************************/
    // Compute output of the controller
    output_ = proportional_ + integrator_ + differentiator_;

    // Output saturation
    if (output_ > outputSaturationUpper_)
        output_ = outputSaturationUpper_;
    else if (output_ < outputSaturationLower_)
        output_ = outputSaturationUpper_;
    
    // Store error and measurement for the next iteration
    prevError_ = currentError;
    prevMeasurement_ = measurement;

    return output_;
}

float PIDController::derivativeFilteredControlSignal(float setpoint, float measurement)
{
    float K;
    float a0, a1, a2;
    float b0, b1, b2;
    
    setSampleTime(1.0);
    K = 2 / T_;

    b0 = pow(K, 2) * Kp_ + K * Ki_ + Ki_ * N_ + K * Kp_ * N_ + pow(K, 2) * Kd_ * N_;
    b1 = 2 * Ki_ * N_ - 2 * pow(K, 2) * Kp_ - 2 * pow(K, 2) * Kd_ * N_;
    b2 = pow(K, 2) * Kp_ - K * Ki_ + Ki_ * N_ - K * Kp_ * N_ + pow(K, 2) * Kd_ * N_;

    a0 =  pow(K, 2) + N_ * K;
    a1 = -2 * pow(K, 2);
    a2 = pow(K, 2) - K * N_;

    e2_ = e1_;
    e1_ = e0_;
    e0_ = setpoint - measurement;

    y2_ = y1_;
    y1_ = y0_;
    y0_ = -a1 / a0 * y1_ - a2 / a0 * y2_ + b0 / a0 * e0_ +
            b1 / a0 * e1_ + b2 / a0 * e2_;

    // Output saturation
    if (y0_ > outputSaturationUpper_)
        y0_ = outputSaturationUpper_;
    else if (y0_ < outputSaturationLower_)
        y0_ = outputSaturationUpper_;

    output_ = y0_;
    return output_;
}

void PIDController::setPIDCoeffs(float Kp, float Ki, float Kd)
{
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;
}

void PIDController::setIntegratorSaturations(float lowerIntSat, float upperIntSat)
{
    integratorSatLower_ = lowerIntSat;
    integratorSatUpper_ = upperIntSat;
}

void PIDController::setOutputSaturations(float lowerSat, float upperSat)
{
    outputSaturationLower_ = lowerSat;
    outputSaturationUpper_ = upperSat;
}

void PIDController::setSampleTime(float sampleTime)
{
    T_ = sampleTime;
}

void PIDController::setDerivativeFilterConstant(float n)
{
    N_ = n;
}

float PIDController::getControllerOutput() const
{
    return output_;
}
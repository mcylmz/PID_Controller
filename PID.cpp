#include "PID.h"


PIDController::PIDController()
{
    integrator_ = 0.0;
    prevError_ = 0.0;

    differentiator_ = 0.0;
    prevMeasurement_ = 0.0;
    
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
#ifndef PID_H_
#define PID_H_

class PIDController
{
public:
    PIDController();

    float computeControlSignal(float, float);

    // Setters
    void setPIDCoeffs(float, float, float);
    void setIntegratorSaturations(float, float);
    void setOutputSaturations(float, float);
    void setSampleTime(float);
    void setDerivativeFilterConstant(float);

    // Getters
    float getControllerOutput() const;

private:
    // Controller gains
    float Kp_;
    float Ki_;
    float Kd_;

    // Derivative filter constant
    float N_;

    // Integrator saturations
    float integratorSatLower_;
    float integratorSatUpper_;

    // Output saturations
    float outputSaturationLower_;
    float outputSaturationUpper_;

    // Sample time(in seconds)
    float T_;

    // PID lines output
    float proportional_;
    float integrator_;
    float differentiator_;

    float prevError_; // Required for integrator
    float prevMeasurement_; // Required for differentiator

    // Controller output
    float output_;    
};

#endif
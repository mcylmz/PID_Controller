#ifndef PID_H_
#define PID_H_

class PIDController
{
public:
    PIDController();

    float computeControlSignal(float, float);
    float derivativeFilteredControlSignal(float, float);

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

    // Classical PID variables
    float prevError_;
    float prevMeasurement_;

    // Derivative filtered discrete PID variables
    float e0_, e1_, e2_;
    float y0_, y1_, y2_;

    // Controller output
    float output_;    
};

#endif
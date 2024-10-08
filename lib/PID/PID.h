/*
* Copyright 2024 Constantin Dumitru Petre RĂDULICEA
* 
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
* 
*   http://www.apache.org/licenses/LICENSE-2.0
* 
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#ifndef __PID_H__
#define __PID_H__

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
#define ABS(a) (((a)<(0))?(-(a)):(a))
class PID
{
public:
    PID(double Kp, double Ki, double Kd, double min_output, double max_output) :
        _max_output(max_output),
        _min_output(min_output),
        _Kp(Kp),
        _Kd(Kd),
        _Ki(Ki),
        _pre_error(0.0),
        _integral(0.0)
    {
        this->_integral_max_error = -1.0;
    }

    PID(double Kp, double Ki, double Kd, double max_output, double min_output, double integral_max_error) :
        _max_output(max_output),
        _min_output(min_output),
        _Kp(Kp),
        _Kd(Kd),
        _Ki(Ki),
        _pre_error(0.0),
        _integral(0.0)
    {
        this->_integral_max_error = integral_max_error;
    }

    PID() :
        _max_output(0.0),
        _min_output(0.0),
        _Kp(0.0),
        _Kd(0.0),
        _Ki(0.0),
        _pre_error(0.0),
        _integral(0.0),
        _integral_max_error(-1.0)
    {

    }
    double calculate(double setpoint, double pv, double timePassedFromLastSample=1.0) volatile
    {
        double error, Pout, Iout, Dout, derivative, output;
        // Calculate error
        error = setpoint - pv;

        // Proportional terms
        Pout = _Kp * error;


        if (timePassedFromLastSample > 0.0) {
            // Integral term
            _integral = _integral + (error * timePassedFromLastSample);
            

            // Restrict to max/min
            if (this->_integral_max_error >= 0.0)
            {
                if (_integral > this->_integral_max_error) {
                    _integral = this->_integral_max_error;
                }
                else if (_integral < -(this->_integral_max_error)) {
                    _integral = -(this->_integral_max_error);
                }
            }

            Iout = _Ki * _integral;

            // Derivative term
            derivative = (error - _pre_error) / timePassedFromLastSample;
            Dout = _Kd * derivative;
            

        }
        else {
            Dout = 0.0;
            Iout = 0.0;
        }
        


        // Calculate total output
        output = Pout + Iout + Dout;

        // Restrict to max/min
        if (output > _max_output) {
            output = _max_output;
        }
        else if (output < _min_output) {
            output = _min_output;
        }

        // Save error to previous error
        _pre_error = error;

        return output;
    }

    ~PID() {

    }

    void setParameters(double Kp, double Ki, double Kd) volatile{
        this->_Kp = Kp;
        this->_Kd = Kd;
        this->_Ki = Ki;
    }

    void setMaxOutput(double val) volatile{
        this->_max_output = val;
    }
    void setMinOutput(double val) volatile{
        this->_min_output = val;
    }

    void setKp(double val) volatile {
        this->_Kp = val;
    }

    void setKi(double val) volatile {
        this->_Ki = val;
    }

    void setKd(double val) volatile {
        this->_Kd = val;
    }

    void setIntegralImpact(double val) volatile {
        if (val < 0.0) {
            val = -val;
        }
        this->_integral_max_error = val;
    }

    double getIntegralImpact(double val) volatile {
        return this->_integral_max_error;
    }

    double getMaxOutput(double val) volatile {
        return this->_max_output;
    }
    double getMinOutput(double val) {
        return this->_min_output;
    }

    double getKp(double val) volatile {
        return this->_Kp;
    }

    double getKi(double val) {
        return this->_Ki;
    }

    double getKd(double val) {
        return this->_Kd;
    }

    void reset() {
        this->_pre_error = 0.0;
        this->_integral = 0.0;
    }

private:
    double _max_output;
    double _min_output;
    double _Kp;
    double _Kd;
    double _Ki;
    double _pre_error = 0.0;
    double _integral = 0.0;
    double _integral_max_error = 0.0;
};

#endif // !__PID_H__

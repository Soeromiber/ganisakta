#include "ganisakta/pid/pid.hpp"

namespace ganisakta
{
    namespace pid
    {
        PID::PID() = default;

        PID::PID(double dt, double max, double min, double Kp, double Kd, double Ki){
            _dt = dt;
            _max = max;
            _min = min;
            _Kp = Kp;
            _Kd = Kd;
            _Ki = Ki;
            _pre_error = 0;
            _integral = 0;

        }

        double PID::calculate(double setpoint, double pv){
            // Calculate error
            double error = setpoint - pv;

            // Proportional term
            double Pout = _Kp * error;

            // Integral term
            this->_integral += error * _dt;
            double Iout = _Ki * _integral;

            // Derivative term
            double derivative = (error - _pre_error) / _dt;
            double Dout = _Kd * derivative;

            // Calculate total output
            double output = Pout + Iout + Dout;

            // Restrict to max/min
            if( output > _max )
                output = _max;
            else if( output < _min )
                output = _min;

            // Save error to previous error
            this->_pre_error = error;

            return output;
        }

        void PID::resetPreError(){
            this->_pre_error = 0;
        }

        void PID::resetIntegral(){
            this->_integral = 0;
        }

        void PID::reset(){
            this->resetPreError();
            this->resetIntegral();
        }
    }
}
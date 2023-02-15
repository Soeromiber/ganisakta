#pragma once

namespace ganisakta
{
    namespace pid
    {
        class PID{
            private:
                double _dt;
                double _max;
                double _min;
                double _Kp;
                double _Kd;
                double _Ki;
                double _pre_error;
                double _integral;
            public:
                PID();
                PID(double dt, double max, double min, double Kp, double Kd, double Ki);
                double calculate(double setpoint, double pv);
                void reset();
                void resetPreError();
                void resetIntegral();
        };
    }
}
#ifndef _PID_LIB

#define _PID_LIB

class PID
{
    public:

        PID()
        {
            _kp = 0;
            _kd = 0;
            _ki = 0;
        }

        PID(float kp, float kd, float ki)
        {
            _kp = kp;
            _kd = kd;
            _ki = ki;
        }

        float compute(float error_new, double dt)
        {
            float result  = _kp*error_new + _kd*error_prev/dt + _ki*error_sum;

            error_prev = error_new;

            error_sum += error_new;

            return result;
        }

    protected:
        float _kp,_kd,_ki;

        float error_prev,error_sum;
};

#endif
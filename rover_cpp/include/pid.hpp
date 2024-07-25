#ifndef _PID_LIB

#define _PID_LIB

/*

Class to contain PID constants and functions

Members:
(double) kp,kd,ki: The PID constants, can only be initialised through the constructor
(double) error_prev: The previous error used to calculate the derivative term
(double) error_sum: The sum of all error used to calculate the integral term
 */
class PID
{
    public:
        /*
        Constructor used to initialise the PID constants
        */
        PID()
        {
            _kp = 0;
            _kd = 0;
            _ki = 0;
        }

        PID(double kp, double kd, double ki)
        {
            _kp = kp;
            _kd = kd;
            _ki = ki;
        }
        /*
        Function used to calculate the PID output

        Calculates a PID output using the curent error, previous error, and sum of errors.
        Also updates the previous error and sum.

        Parameters:
        (double) error_new: The current error
        (double) dt: The difference in time required to calculate derivative and integral

        Returns:
        (double) result: The PID output
        */
        double compute(double error_new, double dt)
        {
            double result  = _kp*error_new + _kd*error_prev/dt + _ki*error_sum*dt;

            error_prev = error_new;

            error_sum += error_new;

            return result;
        }

    protected:

        double _kp,_kd,_ki;

        double error_prev,error_sum;
};

#endif
class PID {
public:
    // Constructor to initialize gains and sample time
    PID(double kp, double ki, double kd, double dt)
        : kp_(kp), ki_(ki), kd_(kd), dt_(dt), pre_error_(0), integral_(0) {}

    // Method to calculate the PID output
    double calculate(double setpoint, double pv) {
        // 1. Calculate error
        double error = setpoint - pv;

        // 2. Proportional term
        double Pout = kp_ * error;

        // 3. Integral term
        integral_ += error * dt_;
        double Iout = ki_ * integral_;

        // 4. Derivative term
        double derivative = (error - pre_error_) / dt_;
        double Dout = kd_ * derivative;

        // 5. Total output
        double output = Pout + Iout + Dout;

        // Save error for next iteration
        pre_error_ = error;

        return output;
    }

private:
    double kp_, ki_, kd_, dt_;
    double pre_error_, integral_;
};
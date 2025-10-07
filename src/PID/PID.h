class PID{
    double kp;
    double ki;
    double kd;
    double previous_error;
    double setpoint;
    unsigned long *DT;
    double integral; // Make integral a member variable to avoid static issues

    public:
        PID(double p, double i, double d, unsigned long *DT);
        ~PID();

        double compute(double measured_value);
        void setP(double p);
        void setI(double i);
        void setD(double d);
        void setSetpoint(double setpoint);
        void reset(); // Add reset function to clear integral
};
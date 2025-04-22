#ifndef PID_CONTROL
#define PID_CONTROL

class pid{
    public:
    pid(float kp = 0, float kd = 0, float ki = 0, float threshold = 0):
    k_p{kp},
    k_i{ki},
    k_d{kd},
    integral_threshold{threshold}
    {}

    float k_p;
    float k_i;
    float k_d;
    float integral_threshold;

    float sensor = 0;
    float error = 0;
    float integral = 0;
    float previous_error = 0;
    float derivative = 0;
    float target = 0;

    void update(float input_sensor);
    float get_voltage(float input_sensor);
};

#endif
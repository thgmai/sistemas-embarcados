
/* Internal PID controller state */
typedef struct controller_state {

    double error;      	/* Previous error */
    double integral;	/* Integral term */

} ControllerState;

/* Internal module state (file-scope, private) */
static ControllerState ctrl_state;


static double clamper(double value, double lower_limit, double upper_limit) {

	if(value > upper_limit) {
		return upper_limit;
	}
	else if (value < lower_limit) {
		return lower_limit;
	}
	else {
		return value;
	}
} 

void resetController(void) {

    ctrl_state.integral = 0.0;
    ctrl_state.error = 0.0;
}

double controlSignalPID(double sp, double pv, Gain gain) {

    double u;
    double e;

	double int_e = ctrl_state.integral;
	double e_prev = ctrl_state.error;
    double diff_e;

    double kp = gain.kp;
    double ki = gain.ki;
    double kd = gain.kd;

    /* Tempo de amostragem */
    dt = TS / 1000;

    /* Compute control error */
    e = sp - pv;

    /* Integral term (Euler backward) */
    int_e += e * dt;

    /* Anti-windup */
    int_e = clamper(int_e, INTEGRAL_MIN, INTEGRAL_MAX);

    /* Derivative term */
    diff_e = (e - e_prev) / dt;

    /* PID control law */
    u = kp * e + ki * int_e + kd * diff_e;

    /* Output saturation */
    u = clamper(u, CV_MIN, CV_MAX);

    /* Update stored error and integral term */
    ctrl_state.error = e;
    ctrl_state.integral = int_e;

    return u;
}
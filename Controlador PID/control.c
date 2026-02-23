#include <stdint.h>
#include <math.h>
#include "control.h"
#include "gpio.h"
#include "pwm.h"
#include "adc.h"

/* ************************************************************************** */
/* Internal Types                                                             */
/* ************************************************************************** */

/** ***************************************************************************
 * @brief Internal PID controller state
 */
typedef struct controller_state {
    double error_prev;   /* Previous error */
    double integral;     /* Integral term */
    double derivative;   /* Filtered derivative term */
} ControllerState;

/* ************************************************************************** */
/* Private Variables                                                          */
/* ************************************************************************** */

static ControllerState ctrl_state;

/* Pin definitions (module scope) */
static uint8_t sp_pin;
static uint8_t pv_pin;
static uint8_t cv_pin;
static uint8_t led_pin;

/* ************************************************************************** */
/* Global Variables                                                           */
/* ************************************************************************** */

volatile uint8_t refresh_tick = 0;

double sp = 0.0;
double pv = 0.0;
double cv = 0.0;

Gain gain = {0};

/* ************************************************************************** */
/* Private Function Prototypes                                                */
/* ************************************************************************** */

static double clamper(double value, double lower_limit, double upper_limit);

/* ************************************************************************** */
/* Public Functions                                                           */
/* ************************************************************************** */

void controllerInit(uint8_t sp_p, uint8_t pv_p, uint8_t cv_p, uint8_t led_p) {

    /* Store pin assignments */
    sp_pin  = sp_p;
    pv_pin  = pv_p;
    cv_pin  = cv_p;
    led_pin = led_p;

    uint16_t inputs  = (1U << sp_pin) | (1U << pv_pin);
    uint16_t outputs = (1U << cv_pin) | (1U << led_pin);

    /* Configure GPIO */
    GPIO_Init(GPIOE, inputs, outputs);
    GPIO_WritePins(GPIOE, outputs, 0);

    /* Initialize ADC */
    ADC_Init(ADC_FREQ);
    ADC_ConfigChannel(sp_pin, 0);
    ADC_ConfigChannel(pv_pin, 0);

    /* Initialize PWM */
    PWM_Init(TIMER3, PWM_LOC1, PWM_PARAMS_CH2_ENABLEPIN);
    PWM_Write(TIMER3, 2, 0);
    PWM_Start(TIMER3);

    /* Reset controller internal state */
    ctrl_state.integral   = 0.0;
    ctrl_state.error_prev = 0.0;
    ctrl_state.derivative = 0.0;
}

double readSetpoint(void) {
    uint32_t adc_value = ADC_Read(sp_pin);
    return (CV_MAX * (double)adc_value / ADC_RES);
}

double readProcessVariable(void) {
    uint32_t adc_value = ADC_Read(pv_pin);
    return (CV_MAX * (double)adc_value / ADC_RES);
}

void writeControlVariable(double cv_value) {

    cv_value = clamper(cv_value, CV_MIN, CV_MAX);

    uint32_t pwm_value = (uint32_t)((cv_value / CV_MAX) * MAX_TIMER);

    PWM_Write(TIMER3, 2, pwm_value);
}

void writeLED(uint8_t state) {
    GPIO_WritePins(GPIOE, (1U << led_pin), state ? (1U << led_pin) : 0);
}

bool steadyState(void) {

    static uint32_t steady_counter = 0;
    static const uint32_t STEADY_COUNT_REQUIRED = 10;

    double current_error = fabs(sp - pv);

    if (current_error < SS_ERROR_EPS) {
        steady_counter++;
        if (steady_counter >= STEADY_COUNT_REQUIRED) {
            return true;
        }
    }
    else {
        steady_counter = 0;
    }

    return false;
}

/* ************************************************************************** */
/* PID Controller                                                             */
/* ************************************************************************** */

double controlSignalPID(double sp, double pv, Gain gain) {

    double error;
    double alpha;
    double u;
    double dt = TS / 1000.0;

    /* Compute error */
    error = sp - pv;

    /* ---------- Integral (with conditional anti-windup) ---------- */

    double integral_candidate = ctrl_state.integral + error * dt;

    double derivative_raw = (error - ctrl_state.error_prev) / dt;

    /* First compute provisional output */
    double u_provisional = gain.kp * error + gain.ki * integral_candidate + gain.kd * derivative_raw;

    /* Integrate only if not saturating */
    if ((u_provisional < CV_MAX) && (u_provisional > CV_MIN)) {
        ctrl_state.integral = integral_candidate;
    }

    ctrl_state.integral = clamper(ctrl_state.integral, INTEGRAL_MIN, INTEGRAL_MAX);

    /* ---------- Derivative with low-pass filter ---------- */

    alpha = gain.td / (gain.td + dt);

    ctrl_state.derivative = alpha * ctrl_state.derivative + (1.0 - alpha) * derivative_raw;

    /* ---------- Final PID output ---------- */

    u = gain.kp * error + gain.ki * ctrl_state.integral + gain.kd * ctrl_state.derivative;

    u = clamper(u, CV_MIN, CV_MAX);

    ctrl_state.error_prev = error;

    return u;
}

/* ************************************************************************** */
/* Private Functions                                                          */
/* ************************************************************************** */

static double clamper(double value, double lower_limit, double upper_limit) {

    if (value > upper_limit) {
        return upper_limit;
    }
    else if (value < lower_limit) {
        return lower_limit;
    }
    else {
        return value;
    }
}
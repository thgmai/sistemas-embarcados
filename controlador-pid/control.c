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
static uint8_t sp_channel;
static uint8_t pv_channel;
static uint8_t cv_channel;
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

void controllerInit(uint8_t sp_ch, uint8_t pv_ch, uint8_t cv_ch, uint8_t led_p) {

    /* Store pin assignments */
    sp_channel  = sp_ch;
    pv_channel  = pv_ch;
    cv_channel  = cv_ch;
    led_pin     = led_p;

    uint16_t digital_inputs  = 0U;
    uint16_t digital_outputs = (1U << led_p);

    /* Configure GPIO */
    GPIO_Init(GPIOE, digital_inputs, digital_outputs);
    GPIO_WritePins(GPIOE, digital_outputs, 0);

    /* Initialize ADC */
    ADC_Init(ADC_FREQ);
    ADC_ConfigChannel(sp_channel, 0);
    ADC_ConfigChannel(pv_channel, 0);

    /* Initialize PWM */
    unsigned params = (PWM_PARAMS_ENABLEPIN << (4 * cv_channel));
    PWM_Init(TIMER3, PWM_LOC1, params);
    PWM_Write(TIMER3, cv_channel, 0);
    PWM_Start(TIMER3);

    /* Reset controller internal state */
    ctrl_state.integral   = 0.0;
    ctrl_state.error_prev = 0.0;
    ctrl_state.derivative = 0.0;
}

double readSetpoint(void) {
    uint32_t adc_value = ADC_Read(sp_channel);
    return (V_MAX * (double)adc_value / ADC_RES);
}

double readProcessVariable(void) {
    uint32_t adc_value = ADC_Read(pv_channel);
    return (V_MAX * (double)adc_value / ADC_RES);
}

void writeControlVariable(double cv_value) {

    cv_value = clamper(cv_value, V_MIN, V_MAX);

    uint32_t pwm_value = (uint32_t)(((cv_value - V_MIN) / (V_MAX - V_MIN)) * MAX_TIMER);

    PWM_Write(TIMER3, cv_channel, pwm_value);
}

void writeLED(bool state) {

    uint16_t led_mask = (1U << led_pin);

    if (state) {
        GPIO_WritePins(GPIOE, 0, led_mask);
    }
    else {
        GPIO_WritePins(GPIOE, led_mask, 0);
    }
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
    if ((u_provisional < V_MAX) && (u_provisional > V_MIN)) {
        ctrl_state.integral = integral_candidate;
    }

    ctrl_state.integral = clamper(ctrl_state.integral, INTEGRAL_MIN, INTEGRAL_MAX);

    /* ---------- Derivative with low-pass filter ---------- */

    alpha = gain.td / (gain.td + dt);

    ctrl_state.derivative = alpha * ctrl_state.derivative + (1.0 - alpha) * derivative_raw;

    /* ---------- Final PID output ---------- */

    u = gain.kp * error + gain.ki * ctrl_state.integral + gain.kd * ctrl_state.derivative;

    u = clamper(u, V_MIN, V_MAX);

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
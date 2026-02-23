#ifndef CONTROL_H
#define CONTROL_H

/** ***************************************************************************
 * @file    control.h
 * @brief   PID control module interface
 *
 * @details
 * This file defines constants, data types, global variables and function
 * prototypes related to the PID controller and its control state machine.
 */

#include <stdint.h>
#include <stdbool.h>

/* ************************************************************************** */
/* Constants                                                                  */
/* ************************************************************************** */

#define TS              100     /* Sampling time in milliseconds */
#define ADC_FREQ        500000  /* ADC frequency (Hz) */
#define ADC_RES         4095    /* ADC resolution (12-bit = 4095) */
#define CV_MAX          3.3     /* Maximum control signal voltage (V) */
#define CV_MIN          0.0     /* Minimum control signal voltage (V) */
#define INTEGRAL_MAX    10.0    /* Integral term upper limit (anti-windup) */
#define INTEGRAL_MIN   -10.0    /* Integral term lower limit (anti-windup) */
#define SS_ERROR_EPS    0.05    /* Steady-state error threshold */
#define MAX_TIMER       65535   /* Maximum timer value for PWM (16-bit) */

/* ************************************************************************** */
/* Types                                                                      */
/* ************************************************************************** */

/** ***************************************************************************
 * @brief PID controller gains
 */
typedef struct gain_pid {
    double kp;   /* Proportional gain */
    double ki;   /* Integral gain */
    double kd;   /* Derivative gain */
    double td;   /* Derivative filter parameter */
} Gain;

/** ***************************************************************************
 * @brief Control state machine states
 */
typedef enum state {
    idle = 0,        /* Idle state: waiting for sampling event */
    read_inputs,     /* Input acquisition (SP and PV) */
    process,         /* Control signal computation */
    write_outputs    /* Output update */
} State;

/* ************************************************************************** */
/* Global variables                                                           */
/* ************************************************************************** */

extern volatile uint8_t refresh_tick;   /* Sampling event flag */

extern double sp;   /* Setpoint */
extern double pv;   /* Process variable */
extern double cv;   /* Control variable */

extern Gain gain;   /* PID gain structure */

/* ************************************************************************** */
/* Function prototypes                                                        */
/* ************************************************************************** */

void controllerInit(uint8_t sp_pin, uint8_t pv_pin, uint8_t cv_pin, uint8_t led_pin);

double readSetpoint(void);
double readProcessVariable(void);

double controlSignalPID(double sp, double pv, Gain gain);

bool steadyState(void);

void writeControlVariable(double cv);
void writeLED(uint8_t state);

#endif /* CONTROL_H */
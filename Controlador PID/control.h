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
#define CV_MAX          3.3     /* Maximum control signal voltage (V) */
#define CV_MIN          0.0     /* Minimum control signal voltage (V) */
#define INTEGRAL_MAX    3.3     /* Integral term upper limit (anti-windup) */
#define INTEGRAL_MIN    0.0     /* Integral term lower limit (anti-windup) */
#define SS_ERROR_EPS    0.05    /* Steady-state error threshold */

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

/** ***************************************************************************
 * @brief Initializes the controller and associated peripherals
 *
 * @param sp_pin   Setpoint input pin
 * @param pv_pin   Process variable input pin
 * @param cv_pin   Control signal output pin
 * @param led_pin  Status LED pin
 */
void controllerInit(uint8_t sp_pin, uint8_t pv_pin, uint8_t cv_pin, uint8_t led_pin);

/** ***************************************************************************
 * @brief Reads the current setpoint value
 *
 * @return Setpoint in physical units
 */
double readSetpoint(void);

/** ***************************************************************************
 * @brief Reads the current process variable
 *
 * @return Process variable in physical units
 */
double readProcessVariable(void);

/** ***************************************************************************
 * @brief Computes the PID control signal
 *
 * @param sp    Setpoint
 * @param pv    Process variable
 * @param gain  PID gain structure
 *
 * @return Control signal limited between CV_MIN and CV_MAX
 */
double controlSignalPID(double sp, double pv, Gain gain);

/** ***************************************************************************
 * @brief Checks whether the system has reached steady state
 *
 * @return true if the system is in steady state, false otherwise
 */
bool steadyState(void);

/** ***************************************************************************
 * @brief Writes the control signal to the output
 *
 * @param cv Control signal
 */
void writeControlVariable(double cv);

/** ***************************************************************************
 * @brief Controls the status LED
 *
 * @param state 1 to turn on, 0 to turn off
 */
void writeLED(uint8_t state);

#endif /* CONTROL_H */
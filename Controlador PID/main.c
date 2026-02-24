/* ***************************************************************************
 * @file    main.c
 * @brief   Implementação de um controlador PID em sistema embarcado
 *
 * @details
 * O controle é executado em tempo discreto utilizando um kernel cooperativo
 * orientado a tempo (time-triggered). O SysTick gera a base de tempo de 1 ms,
 * enquanto as tarefas são despachadas no contexto não-interruptivo.
 *
 * @author  Thiago Henrique Genaio Mai
 * @date    11/12/2025
 * ************************************************************************** */

#include <stdint.h>

/* **************************************************************************
 * By including this file, it is possible to define the target processor
 * via command line, e.g. -DEFM32GG995F1024.
 * Alternatively, the processor-specific header may be included directly:
 *   #include "efm32gg995f1024.h"
 * ************************************************************************** */
#include "em_device.h"
#include "clock_efm32gg2.h"
#include "tt_tasks.h"
#include "control.h"

/* SysTick frequency: 1 kHz (1 ms) */
#define DIVIDER 1000

/* **************************************************************************
 * Hardware pin definitions
 * ************************************************************************** */

#define SP      0   /* ADC channel for Setpoint */
#define PV      1   /* ADC channel for Process Variable */
#define CV      2   /* PWM output channel */
#define LED     3   /* Status LED pin */

/* ***************************************************************************
 * @brief  SysTick interrupt handler
 *
 * @details
 * This is the only interrupt service routine in the system.
 * It provides the time base for the cooperative kernel by calling
 * Task_Update(), which increments internal task counters.
 *
 * @note   Executed every 1 ms
 * *************************************************************************** */
void SysTick_Handler(void) {
    Task_Update();
}

/* ***************************************************************************
 * @brief  Control state machine
 *
 * @details
 * Implements the control loop as a finite state machine. The controller
 * advances one state at a time and only executes a full control cycle
 * when a sampling event (refresh_tick) is generated.
 *
 * All states are executed in non-interrupt context.
 * *************************************************************************** */
void stateMachine(void) {

    /* Current state of the control state machine */
    static State state = idle;

    switch(state)
    {
        case idle:
            /* Wait for the sampling timer event */
            if (refresh_tick)
            {
                refresh_tick = 0;
                state = read_inputs;
            }
            break;

        case read_inputs:
            /* Read setpoint and process variable */
            sp = readSetpoint();
            pv = readProcessVariable();
            state = process;
            break;

        case process:
            /* Compute PID control signal */
            cv = controlSignalPID(sp, pv, gain);
            state = write_outputs;
            break;

        case write_outputs:
            /* Update control output */
            writeControlVariable(cv);

            /* Indicate steady-state condition via LED */
            writeLED(steadyState());

            state = idle;
            break;

        default:
            /* Safety fallback */
            state = idle;
            break;
    }
}

/* ***************************************************************************
 * @brief  Sampling timer callback
 *
 * @details
 * This function is executed periodically by the task kernel and signals
 * the control state machine that a new sampling period has elapsed.
 * *************************************************************************** */
void sampleTimer(void) {
    refresh_tick = 1U;
}

/** ***************************************************************************
 * @brief  Main function
 *
 * @details
 * Initializes the system clock, peripherals, task kernel and control
 * parameters. After initialization, the system runs indefinitely
 * executing scheduled tasks in a cooperative manner.
 */
int main(void) {

    /* Set system clock source to external crystal: 48 MHz */
    (void) SystemCoreClockSet(CLOCK_HFXO, 1, 1);

    /* Initialize controller I/O pins */
    controllerInit(SP, PV, CV, LED);

    /* Configure SysTick to generate a 1 ms time base */
    SysTick_Config(SystemCoreClock / DIVIDER);

    /* Initialize cooperative task kernel */
    Task_Init();

    /* Add periodic tasks */
    Task_Add(sampleTimer, TS, 0);   /* Sampling timer task (TS ms period) */
    Task_Add(stateMachine, 1, 0);   /* State machine executes every 1 ms */

    /* Define PID gains */
    gain.kp = 2.0;
    gain.ki = 10.0;
    gain.kd = 0.0;
    gain.td = 0.0;

    /* Explicit initialization of sampling event flag */
    refresh_tick = 0U;

	/* Enable IRQs */
    __enable_irq();

    /* Main loop: dispatch ready tasks */
    while (1) {
        Task_Dispatch();
    }
}

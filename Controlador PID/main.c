/**
 * @note    Implementa um controlador PID
 *
 * @author  Thiago Henrique Genaio Mai
 * @date    11/12/2025
 */
 
#include <stdint.h>
/*
 * Including this file, it is possible to define which processor using command line
 * E.g. -DEFM32GG995F1024
 * The alternative is to include the processor specific file directly
 * #include "efm32gg995f1024.h"
 */
#include "em_device.h"
#include "clock_efm32gg.h"
#include "timers.h"
#include "control.h"

#define SYSTICKDIVIDER 1000

/*****************************************************************************
 * @brief  SysTick interrupt handler
 *
 * @note   Called every 1/DIVIDER seconds (1 ms)
 */

void SysTick_Handler(void) {

    static int8_t state = idle;            // Deve ser estático

    switch(state) {

        case idle:

            // Espera o tick do timer de amostragem

            if(refresh_tick) {
                state = read_inputs;
                refresh_tick = 0;
            }

            break;

        case read_inputs:

            // Lê as entradas

            sp = readSetpoint();
            pv = readProcessVariable();

            state = process;

            break;

        case process:

            // Calcula o sinal de controle

            cv = controlSignalPID(sp, pv, gain);

            state = write_outputs;

            break;

        case write_outputs:

            // Atualiza as saídas

            writeControlVariable(cv);

            if(steadyState()) {
                writeLED(1);
            }
            else {
                writeLED(0);
            }

            state = idle;

            break;
        }

        Timers_dispatch();          // Temporizador de amostragem (100 ms)
}

void sampleTimer(void) {
    refresh_tick = 1;
}

/*****************************************************************************
 * @brief  Main function
 *
 * @note   Using default clock configuration
 *         HFCLK = HFRCO
 *         HFCORECLK = HFCLK
 *         HFPERCLK  = HFCLK
 */

int main(void) {


    // Set clock source to external crystal: 48 MHz
    (void) SystemCoreClockSet(CLOCK_HFXO, 1, 1);

    
    /* Configure Pins in GPIOE */
    controllerInit(SP, PV, CV, LED);

    /* Configure SysTick */
    SysTick_Config(SystemCoreClock/SYSTICKDIVIDER);

    Timers_add(TS, sampleTimer);

    /* Main loop */
    while (1) {}

}
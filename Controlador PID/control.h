#ifndef CONTROL_H
#define CONTROL_H

// Cabeçalhos padrão incluídos:

#include <stdint.h>
#include <stdbool.h>

// Constantes:

#define TS 100                          // Tempo de amostragem em ms
#define CV_MAX 3.3                      // Tensão máxima do sinal de controle em V
#define CV_MIN 0.0                      // Tensão mínima do sinal de controle em V
#define INTEGRAL_MAX 3.3                // Limite superior do termo integral (anti-windup)
#define INTEGRAL_MIN 0.0                // Limite inferior do termo integral (anti-windup)
#define SS_ERROR_EPS 0.05               // Critério de regime permanente


// Tipos:

typedef struct gain_pid {

    // Ganhos do controlador PID

    double kp;
    double ki;
    double kd;
    double td;

} Gain;

typedef struct error {

    // Sinal de erro do controlador

    double e;
    double e_prev;

} Error;

typedef enum state {

    // Estados da máquina de estados do controlador

    idle = 0,
    read_inputs,
    process,
    write_outputs

} State;

// Variáveis globais (definidas em outro módulo)

extern volatile uint8_t refresh_tick;

extern double sp;      // Setpoint
extern double pv;      // Process Variable
extern double cv;      // Control Variable

extern Gain gain;      // Ganho PID

// Funções:

/**
 * @brief Inicializa o controlador e os periféricos associados
 *
 * @param sp_pin   Pino de leitura do setpoint
 * @param pv_pin   Pino de leitura da variável de processo
 * @param cv_pin   Pino de saída do sinal de controle
 * @param led_pin  Pino do LED de indicação
 */
void controllerInit(uint8_t sp_pin, uint8_t pv_pin, uint8_t cv_pin, uint8_t led_pin);

/**
 * @brief Lê o valor atual do setpoint
 *
 * @return Setpoint em unidades físicas
 */
double readSetpoint(void);

/**
 * @brief Lê a variável de processo
 *
 * @return Variável de processo em unidades físicas
 */
double readProcessVariable(void);

/**
 * @brief Calcula o sinal de controle PID
 *
 * @param sp    Setpoint
 * @param pv    Variável de processo
 * @param gain  Estrutura com ganhos do PID
 *
 * @return Sinal de controle limitado entre CV_MIN e CV_MAX
 */
double controlSignalPID(double sp, double pv, Gain gain);

/**
 * @brief Verifica se o sistema atingiu regime permanente
 *
 * @return true se estiver em regime permanente, false caso contrário
 */
bool steadyState(void);

/**
 * @brief Escreve o sinal de controle na saída
 *
 * @param cv Sinal de controle
 */
void writeControlVariable(double cv);

/**
 * @brief Acende ou apaga o LED de indicação
 *
 * @param state 1 para ligar, 0 para desligar
 */
void writeLED(uint8_t state);

#endif // CONTROL_H
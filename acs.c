//  ACS — Sistema de Controle de Altitude


// ------------------------------------------------------------
//  1. INCLUDES: Nessa parte, fizemos todos os includes de bibliotecas 
// ------------------------------------------------------------
#include <stdio.h> // Inclui a biblioteca de Input-Output
#include "pico/stdlib.h" // Inclui a biblioteca padrão do Raspberry Pi Pico (SDK)
#include "pico/multicore.h" // Inclui a biblioteca que nos permite fazer o gerenciamento do multi-core do microcontrolador (O RP2040 tem dois núcleos)
#include "hardware/adc.h" // Inclui a biblioteca do conversor analógico-digital
#include "hardware/pwm.h" // Inclui a biblioteca dos PWM a serem utilizados
#include "hardware/pio.h" // Inclui a biblioteca dos blocos programáveis PIO. Tais blocos serão úteis para a perioditização precisa de algumas rotinas.
#include "hardware/clocks.h" // Inclui a biblioteca de gerenciamento de clock do microcontrolador
#include "hardware/gpio.h" // Inclui a biblioteca das portas do General Purpose da placa
#include "hardware/irq.h" // Inclui a biblioteca para o uso das interrupções.
#include "ws2812.pio.h"         // gerado automaticamente pelo build, a partir de um arquivo exemplo.

// ------------------------------------------------------------
//  2. DEFINES — PINAGEM: Com a definição dessas "variáveis", conseguimos modularizar o código com mais eficiência
// Em cas de dúvidas, sugerimos que o material seja consultado e o datasheet para entender melhor esses defines
// ------------------------------------------------------------
#define PIN_JOYSTICK    27 
// ADC1 — eixo Y do KY-023 (Importante mencionar que utilizamos o único canal ADC disponível na BitDogLab)
#define PIN_SERVO       28    
// PWM — sinal do servo motor 1
#define PIN_SERVO2      9
// ePWM do sinal do segundo servo, slice 4, canal B. Próximo fisicamente do GP28
#define PIN_BUZZER      21      
// PWM — buzzer passivo
#define PIN_LED_R       13      
// GPIO — LED RGB canal vermelho
#define PIN_LED_G       11      
// GPIO — LED RGB canal verde
#define PIN_LED_B       12      
// GPIO — LED RGB canal azul
#define PIN_BOTAO       5       
// GPIO — botão de arme (pull-up)
#define PIN_MATRIZ      7       
// PIO  — DIN da matriz WS2812B

// ------------------------------------------------------------
//  3. DEFINES — CONSTANTES DO SISTEMA
// ------------------------------------------------------------

// ADC
#define ADC_CANAL       1       
// Simplesmente definimos que o canal do GP27 é o 1 (pela ligação interna do BitDogLab
#define ADC_CENTRO      2048    
// centro mecânico do joystick de acordo com a divisão de 12 bits (setpoint)
#define ADC_MAX         2048    
// divisor da normalização do erro

// Filtro
#define FILTRO_N        4       // tamanho do buffer circular (média móvel)

// Controlador PD: A justificativa dos valores abaixo deve ser vista no ReadMe geral.
#define KP              0.8f    
// ganho proporcional
#define KD              0.15f  
 // ganho derivativo
#define T_CONTROLE      0.020f  
// período do Core 1 em segundos (20 ms)

// Servo — PWM
#define SERVO_CLK_DIV   125.0f  
// divisor: 125 MHz / 125 = 1 MHz
#define SERVO_WRAP      19999   
// 20.000 contagens = 20 ms = 50 Hz
#define SERVO_MIN_US    1000    
// 1000 µs = 0°  (freio fechado)
#define SERVO_MAX_US    2000    
// 2000 µs = 90° (freio totalmente aberto)

// Buzzer
#define BUZZER_LIMIAR   0.40f   
// erro mínimo para ativar o buzzer
#define BUZZER_F_MIN    250     
// frequência mínima (Hz)
#define BUZZER_F_MAX    1000    
// frequência máxima (Hz)

// Debounce do botão
#define DEBOUNCE_MS     250     
// intervalo mínimo entre acionamentos

// Períodos dos loops
#define PERIODO_CORE1_MS    20  
// Core 1: malha de controle
#define PERIODO_CORE0_MS    50 
// Core 0: display e sinalização

// Limiares de erro para sinalização
#define LIMIAR_VERDE    0.15f   
// |e[n]| < 0.15 → estado estável
#define LIMIAR_AMARELO  0.40f   
// |e[n]| < 0.40 → estado moderado

// Matriz WS2812B
#define MATRIZ_LEDS     25      
// 5x5 = 25 LEDs
#define WS2812_FREQ     800000  
// frequência do protocolo (800 kHz)

// ------------------------------------------------------------
//  4. VARIÁVEIS GLOBAIS VOLÁTEIS (compartilhadas entre cores)
// ------------------------------------------------------------
volatile float g_error_norm  = 0.0f;   // erro normalizado [-1.0, +1.0]
volatile float g_control_out = 0.0f;   // saída do controlador [0.0, +1.0]
volatile bool  g_armed       = false;  // estado de arme do sistema

// ------------------------------------------------------------
//  5. ISR — BOTÃO DE ARME (GP5)
// ------------------------------------------------------------
// Chamada automaticamente pelo hardware a cada borda de descida no GP5.
// Não deve fazer trabalho pesado — só toggle + debounce + reset de estado.
void botao_isr(uint gpio, uint32_t eventos) {
    // timestamp atual em ms
    static uint32_t t_ultimo = 0;
    uint32_t t_agora = to_ms_since_boot(get_absolute_time());
 
    // debounce: ignora se o intervalo desde o último acionamento for < 250 ms
    if ((t_agora - t_ultimo) < DEBOUNCE_MS) return;
    t_ultimo = t_agora;
 
    // toggle do estado de arme
    g_armed = !g_armed;
 
    // ao desarmar: fecha os servos e zera E[n-1] para evitar spike derivativo
    if (!g_armed) {
        uint slice1 = pwm_gpio_to_slice_num(PIN_SERVO);
        uint slice2 = pwm_gpio_to_slice_num(PIN_SERVO2);
        pwm_set_chan_level(slice1, pwm_gpio_to_channel(PIN_SERVO),  SERVO_MIN_US);
        pwm_set_chan_level(slice2, pwm_gpio_to_channel(PIN_SERVO2), SERVO_MIN_US);
        s_error_prev  = 0.0f;
        g_control_out = 0.0f;
        g_error_norm  = 0.0f;
    }
}

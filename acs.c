//  ACS — Sistema de Controle de Altitude

// ------------------------------------------------------------
//  BLOCO 1 DO CÓDIGO
// ------------------------------------------------------------

//  1. INCLUDES: Nessa parte, fizemos todos os includes de bibliotecas 

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


//  2. DEFINES — PINAGEM: Com a definição dessas "variáveis", conseguimos modularizar o código com mais eficiência
// Em cas de dúvidas, sugerimos que o material seja consultado e o datasheet para entender melhor esses defines

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


//  3. DEFINES — CONSTANTES DO SISTEMA


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


//  4. VARIÁVEIS GLOBAIS VOLÁTEIS (compartilhadas entre cores)

volatile float g_error_norm  = 0.0f;   // erro normalizado [-1.0, +1.0]
volatile float g_control_out = 0.0f;   // saída do controlador [0.0, +1.0]
volatile bool  g_armed       = false;  // estado de arme do sistema

// ------------------------------------------------------------
//  BLOCO 2 DO CÓDIGO: Esse bloco define uma função importante para nosso código, a função de ISR, que interage com um botão ligado ao GP5
// ------------------------------------------------------------

//  5. ISR — BOTÃO DE ARME (GP5)

// Chamada automaticamente pelo hardware a cada borda de descida no GP5.
// Realiza toggle + debounce + reset de estado.
void botao_isr(uint gpio, uint32_t eventos) {
    // timestamp atual em ms
    static uint32_t t_ultimo = 0; // O static aqui é importante para registrar essa variável sem que ela seja apagada no fim da função
    uint32_t t_agora = to_ms_since_boot(get_absolute_time()); // O get_absolute_time é uma função com precisão na escala de microsegundos
    // Apesar do get_absolute_time não ter a mesma precisão do uso dos módulos PIO, sua precisão é suficiente para essa aplicação
 
    // debounce: ignora se o intervalo desde o último acionamento for < 250 ms
    // Mecanicamente, o botão vibra algumas vezes antes de estabilizar, se após os 250 ms (definidos no bloco 1) o botão tiver mudado de estado, a contagem ocorre
    if ((t_agora - t_ultimo) < DEBOUNCE_MS) return;
    t_ultimo = t_agora;
    // toggle do estado de arme. Como se presume que o botão foi acionado, o g_armed foi trocado
    g_armed = !g_armed;
 
    // ao desarmar: fecha os servos e zera E[n-1] para evitar spike derivativo
    if (!g_armed) {
        uint slice1 = pwm_gpio_to_slice_num(PIN_SERVO); // Busca o slice exato do servo 1
        uint slice2 = pwm_gpio_to_slice_num(PIN_SERVO2); // Busca o slice exato do servo 2
        pwm_set_chan_level(slice1, pwm_gpio_to_channel(PIN_SERVO),  SERVO_MIN_US); // Achados o slice do servo 1, coloca ele em estado mínimo, no caso, 0°, fechando o mesmo.
        pwm_set_chan_level(slice2, pwm_gpio_to_channel(PIN_SERVO2), SERVO_MIN_US); // Achados o slice do servo 2, coloca ele em estado mínimo, no caso, 0°, fechando o mesmo.
        // Zera os errors e a saída do controlador. Caso isso não fosse feito, esses valores se acumulariam quando o sistema fosse rearmado, gerando problemas de spike derivativo e 
        s_error_prev  = 0.0f;
        g_control_out = 0.0f;
        g_error_norm  = 0.0f;
    }
}

// ------------------------------------------------------------
//  BLOCO 3 DO CÓDIGO
// ------------------------------------------------------------

//  6. FUNÇÕES AUXILIARES — CORE 1

 
//  6.1 Leitura ADC com filtro de média móvel 
// Mantém um buffer circular de FILTRO_N amostras.
// Retorna a média das últimas N leituras — atenua ruído do potenciômetro.
static float adc_ler_filtrado(void) {
    static uint16_t buffer[FILTRO_N] = {0};  // buffer circular
    static uint8_t  indice = 0;              // posição atual de escrita
    static bool     cheio  = false;          // controle de inicialização
 
    // lê o ADC e armazena no buffer
    adc_select_input(ADC_CANAL);
    buffer[indice] = adc_read();
    indice = (indice + 1) % FILTRO_N;
    if (indice == 0) cheio = true;
 
    // calcula a média — se buffer ainda não encheu, usa só as amostras válidas
    uint32_t soma  = 0;
    uint8_t  n_val = cheio ? FILTRO_N : indice;
    for (uint8_t i = 0; i < n_val; i++) soma += buffer[i];
    return (float)soma / (float)n_val;
}
 
// --- 6.2 Calcula erro normalizado E[n] ---
// Centraliza o valor em zero (setpoint = 2048) e normaliza para [-1, +1].
static float calcular_erro(float leitura) {
    return (leitura - (float)ADC_CENTRO) / (float)ADC_MAX;
}
 
// --- 6.3 Controlador PD com saturação unidirecional ---
// Recebe E[n], usa s_error_prev como E[n-1], retorna u[n] em [0, 1].
static float controlador_pd(float erro) {
    // termo proporcional
    float termo_p = KP * erro;
 
    // termo derivativo — aproximação por diferenças finitas
    float termo_d = KD * (erro - s_error_prev) / T_CONTROLE;
 
    // salva E[n] como E[n-1] para o próximo ciclo
    s_error_prev = erro;
 
    // ação de controle
    float u = termo_p + termo_d;
 
    // saturação unidirecional: sistema só frea, nunca acelera
    if (u < 0.0f) u = 0.0f;
    if (u > 1.0f) u = 1.0f;
 
    return u;
}
 
// --- 6.4 Aplica u[n] nos dois servos via PWM (comportamento gêmeo) ---
// Converte u[n] em largura de pulso e atualiza ambos os servos.
static void servo_set(float u) {
    uint16_t largura = (uint16_t)(SERVO_MIN_US + u * (SERVO_MAX_US - SERVO_MIN_US));
 
    uint slice1 = pwm_gpio_to_slice_num(PIN_SERVO);
    uint slice2 = pwm_gpio_to_slice_num(PIN_SERVO2);
    pwm_set_chan_level(slice1, pwm_gpio_to_channel(PIN_SERVO),  largura);
    pwm_set_chan_level(slice2, pwm_gpio_to_channel(PIN_SERVO2), largura);
}

// ------------------------------------------------------------
//  BLOCO 4 DO CÓDIGO
// ------------------------------------------------------------


//  7. FUNÇÕES AUXILIARES — CORE 0

 
// --- 7.1 LED RGB ---
// Liga o LED na cor desejada. Cátodo comum: HIGH = acende.
static void set_rgb(bool r, bool g, bool b) {
    gpio_put(PIN_LED_R, r);
    gpio_put(PIN_LED_G, g);
    gpio_put(PIN_LED_B, b);
}
 
// --- 7.2 Buzzer ---
// Silencia ou define a frequência do buzzer passivo via PWM.
// O duty cycle é fixo em 50% para máxima amplitude sonora.
static void set_buzzer(uint32_t freq_hz) {
    uint slice = pwm_gpio_to_slice_num(PIN_BUZZER);
 
    if (freq_hz == 0) {
        // silencia: desliga o PWM do buzzer
        pwm_set_chan_level(slice, pwm_gpio_to_channel(PIN_BUZZER), 0);
        return;
    }
 
    // recalcula wrap para a frequência desejada
    // f_contador = 1 MHz (clkdiv=125), wrap = f_contador / freq - 1
    uint32_t wrap = (1000000 / freq_hz) - 1;
    pwm_set_wrap(slice, wrap);
    pwm_set_chan_level(slice, pwm_gpio_to_channel(PIN_BUZZER), wrap / 2); // 50% duty
}
 
// --- 7.3 Envia cor para um LED da matriz WS2812B ---
// O protocolo WS2812B usa formato GRB de 24 bits.
// A função empacota os canais e envia pelo FIFO do PIO.
static inline void matriz_put_pixel(PIO pio, uint sm, uint8_t r, uint8_t g, uint8_t b) {
    // formato GRB: green nos bits 23-16, red nos bits 15-8, blue nos bits 7-0
    uint32_t cor = ((uint32_t)g << 16) | ((uint32_t)r << 8) | (uint32_t)b;
    pio_sm_put_blocking(pio, sm, cor << 8u); // shift para o MSB do registro de 32 bits
}
 
// --- 7.4 Atualiza a matriz WS2812B 5x5 ---
// Linhas 0-3: cursor de erro (posição e cor por faixa)
// Linha 4:    barra de controle u[n] (0 a 4 LEDs amarelos)
static void set_matriz(PIO pio, uint sm, float erro_abs, float controle) {
    // calcula coluna do cursor (0 a 4), proporcional ao erro
    // erro_abs em [0,1], mapeia para coluna 0-4
    int col_cursor = (int)(erro_abs * 4.0f + 0.5f);
    if (col_cursor > 4) col_cursor = 4;
 
    // define cor do cursor por faixa de erro
    uint8_t cur_r, cur_g, cur_b;
    if (erro_abs < LIMIAR_VERDE) {
        cur_r = 0; cur_g = 30; cur_b = 0;   // verde fraco — estável
    } else if (erro_abs < LIMIAR_AMARELO) {
        cur_r = 30; cur_g = 30; cur_b = 0;  // amarelo — moderado
    } else {
        cur_r = 30; cur_g = 0; cur_b = 0;   // vermelho — crítico
    }
 
    // número de LEDs acesos na barra de controle (linha 4)
    int barra = (int)(controle * 4.0f + 0.5f);
    if (barra > 4) barra = 4;
 
    // varre todos os 25 LEDs e define cor de cada um
    for (int linha = 0; linha < 5; linha++) {
        for (int col = 0; col < 5; col++) {
            uint8_t r = 0, g = 0, b = 0;
 
            if (linha < 4) {
                // coluna 2 sempre verde fraco = referência do setpoint
                if (col == 2) {
                    r = 0; g = 10; b = 0;
                }
                // cursor de erro na coluna calculada
                if (col == col_cursor) {
                    r = cur_r; g = cur_g; b = cur_b;
                }
            } else {
                // linha 4: barra amarela proporcional a u[n]
                if (col <= barra) {
                    r = 30; g = 30; b = 0;
                }
            }
 
            matriz_put_pixel(pio, sm, r, g, b);
        }
    }
}
 
// --- 7.5 Atualiza toda a sinalização de uma vez ---
// Decide o estado com base em |e[n]| e chama set_rgb, set_buzzer e set_matriz.
static void atualizar_sinalizacao(PIO pio, uint sm, float erro_norm, float controle) {
    float erro_abs = erro_norm < 0.0f ? -erro_norm : erro_norm; // |e[n]|
 
    if (erro_abs < LIMIAR_VERDE) {
        // estado verde: estável
        set_rgb(false, true, false);
        set_buzzer(0);
    } else if (erro_abs < LIMIAR_AMARELO) {
        // estado amarelo: moderado
        set_rgb(true, true, false);
        // apitos espaçados: frequência fixa, ligado por 50 ms a cada 500 ms
        static uint32_t t_beep = 0;
        uint32_t agora = to_ms_since_boot(get_absolute_time());
        if ((agora - t_beep) > 500) { t_beep = agora; }
        set_buzzer((agora - t_beep) < 50 ? 500 : 0);
    } else {
        // estado vermelho: crítico
        set_rgb(true, false, false);
        // frequência proporcional ao erro: 250 Hz a 1000 Hz
        uint32_t freq = (uint32_t)(BUZZER_F_MIN + erro_abs * (BUZZER_F_MAX - BUZZER_F_MIN));
        set_buzzer(freq);
    }
 
    set_matriz(pio, sm, erro_abs, controle);
}

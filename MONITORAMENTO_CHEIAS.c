#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "hardware/i2c.h"
#include "lib/ssd1306.h"
#include "lib/font.h"
#include "hardware/clocks.h"
#include "queue.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "pio_matrix.pio.h"

// Macros para uso de periféricos
#define VRX 27
#define VRY 26  
#define LED_VERMELHO 13
#define LED_VERDE 11
#define LED_AZUL 12
#define BUZZER_A 21

// Display
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C

// Matriz de leds
PIO pio;
uint sm;
uint32_t VALOR_LED;
unsigned char R, G, B;
#define NUM_PIXELS 25
#define OUT_PIN 7

// Tipo de dado criado para facilitar comunicação com o display
typedef struct {
    int vol_chuva;
    int nivel_agua;
    int modo_alerta;
} dados_display;

double v[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.1, 0.0, 0.1, 0.0, 0.0, 0.0};
double x[] = { 
        0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.1, 0.0, 0.1, 0.0,
        0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.1, 0.0,
        0.1, 0.0, 0.0, 0.0, 0.1
    };
double a[] = {
        0.0, 0.1, 0.1, 0.1, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1,
        0.1, 0.0, 0.0, 0.0, 0.1, 0.1, 0.0, 0.0, 0.0, 0.1,
        0.0, 0.1, 0.1, 0.1, 0.0
    };

QueueHandle_t xQueueJoystickData;
QueueHandle_t xQueueDisplayData;
QueueHandle_t xQueueMatrixData;
QueueHandle_t xQueueBuzzerData;


// Modo BOOTSEL com botão B
#include "pico/bootrom.h"
#define botaoB 6
void gpio_irq_handler(uint gpio, uint32_t events)
{
    reset_usb_boot(0, 0);
}

// Inicialização de dispositivos PWM
uint pwm_init_gpio(uint gpio, uint wrap) {
    
    gpio_set_function(gpio, GPIO_FUNC_PWM);

    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_wrap(slice_num, wrap);
    
    pwm_set_enabled(slice_num, true);  
    return slice_num;  

}

void vjoystick() {
    adc_init();
    adc_gpio_init(VRX);
    
        while (true) {
            adc_select_input(1); // GPIO 27 = ADC1
            uint16_t x_pos = adc_read();
            xQueueSend(xQueueJoystickData, &x_pos, portMAX_DELAY); // Envia o valor do joystick para a fila
            vTaskDelay(pdMS_TO_TICKS(200));                        // 10 Hz de leitura
        }
}


void vleds() {
    
    uint pwm_wrap = 1365;
    adc_select_input(1);
    uint16_t vrx_value;
    uint16_t conv_x;
    dados_display data;
    int vol_chuva;
    int op_matriz; // Este opicional entrará na fila da matriz e irá dizer qual desenho ela deve dizer
    int op_buzzer;
    pwm_init_gpio(LED_VERDE, pwm_wrap);
    pwm_init_gpio(LED_VERMELHO, pwm_wrap);
    pwm_init_gpio(LED_AZUL, pwm_wrap);

    data.modo_alerta = 0;
    data.nivel_agua = 0;
    data.vol_chuva = 0;

    while(true){
        if (xQueueReceive(xQueueJoystickData, &vrx_value, portMAX_DELAY)){
            data.vol_chuva = (vrx_value * 100) / 4095;
            data.nivel_agua += (int)((data.vol_chuva - data.nivel_agua) * 0.05);
            if (vrx_value >= 0 && vrx_value < 1365) {
                conv_x = (vrx_value);  // Mapeia diretamente para 0-1365
                pwm_set_gpio_level(LED_VERDE, conv_x);
                pwm_set_gpio_level(LED_VERMELHO, 0);
                op_matriz = 1;
                data.modo_alerta = 0;
                op_buzzer = 0;
                xQueueSend(xQueueBuzzerData, &op_buzzer, portMAX_DELAY);
                xQueueSend(xQueueMatrixData, &op_matriz, portMAX_DELAY);
                xQueueSend(xQueueDisplayData, &data, portMAX_DELAY);
            }             
            else if (vrx_value >= 1365 && vrx_value < 2866) {
                conv_x = vrx_value - 1365;  // Mapeia para 0-1365
                pwm_set_gpio_level(LED_VERDE, conv_x);
                pwm_set_gpio_level(LED_VERMELHO, conv_x);
                op_matriz = 2;
                data.modo_alerta = 0;
                op_buzzer = 0;
                xQueueSend(xQueueBuzzerData, &op_buzzer, portMAX_DELAY);
                xQueueSend(xQueueMatrixData, &op_matriz, portMAX_DELAY);
                xQueueSend(xQueueDisplayData, &data, portMAX_DELAY);
            } 
            else {
                // O modo alerta deve ser acionado aqui
                data.modo_alerta = 1;
                conv_x = vrx_value - 2730;  // Mapeia para 0-1365
                pwm_set_gpio_level(LED_VERMELHO, conv_x);
                pwm_set_gpio_level(LED_VERDE, 0);
                op_matriz = 3;
                op_buzzer = 1;
                xQueueSend(xQueueBuzzerData, &op_buzzer, portMAX_DELAY);
                xQueueSend(xQueueMatrixData, &op_matriz, portMAX_DELAY);
                xQueueSend(xQueueDisplayData, &data, portMAX_DELAY);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void vDisplayTask() {

    dados_display data; // 0 = vol_chuva, 1 = nivel_agua, 2 = modo alerta
    char vol_chuva_str[10], nivel_agua_str[10];
    i2c_init(I2C_PORT, 400 * 1000);

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    static ssd1306_t ssd;
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT);
    ssd1306_config(&ssd);
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    while(true) {
        if (xQueueReceive(xQueueDisplayData, &data, portMAX_DELAY) == pdTRUE) {
            int vol = data.vol_chuva;  
            int nivel = data.nivel_agua;
             sprintf(vol_chuva_str, "%d%%", vol);
             sprintf(nivel_agua_str, "%d%%", nivel);
            ssd1306_divide_em_4_linhas(&ssd);
            ssd1306_draw_string_escala(&ssd, "Vol. de Chuva - ", 4,  4, 0.8);
            ssd1306_draw_string_escala(&ssd, "Niv. da agua - ",  4,  20, 0.8);
            ssd1306_draw_string_escala(&ssd, vol_chuva_str, 100,  4, 0.8);
            ssd1306_draw_string_escala(&ssd, nivel_agua_str, 95,  20, 0.8);
                if((int)data.modo_alerta == 1) {
                    ssd1306_draw_string_escala(&ssd, "MODO ALERTA", 26, 36, 0.9);
                    ssd1306_draw_string_escala(&ssd, "PROCURE ABRIGO!", 10, 52, 0.9);
                }
            ssd1306_send_data(&ssd);
            ssd1306_draw_string_escala(&ssd, "TV - ", 4, 52, 0.9);
        }
    }
}

void buzz(uint8_t BUZZER_PIN, uint16_t freq, uint16_t duration) {
    int period = (1000000 / freq);
    int pulse = period / 2;
    int cycles = freq * duration / 1000;
    for (int j = 0; j < cycles; j++) {
        gpio_put(BUZZER_PIN, 1);
        sleep_us(pulse);
        gpio_put(BUZZER_PIN, 0);
        sleep_us(pulse);
    }
}


void vBuzzer() {
    gpio_init(BUZZER_A);
    gpio_set_dir(BUZZER_A, GPIO_OUT);
    
    int op_buzzer = 0;
    uint32_t last_toggle_time = 0;
    uint32_t buzz_end_time = 0;
    bool buzzer_state = false;
    uint16_t current_freq = 0;
    
    while(true) {
        // Recebe novos comandos se disponíveis
        if(xQueueReceive(xQueueBuzzerData, &op_buzzer, 0)) {
            if(op_buzzer == 1) {
                current_freq = 600; // 600Hz
                buzz_end_time = xTaskGetTickCount() + pdMS_TO_TICKS(500); // 500ms
            } else {
                current_freq = 0; // Desliga
                gpio_put(BUZZER_A, 0);
            }
        }
        
        // Lógica do buzzer não-bloqueante
        if(current_freq > 0 && xTaskGetTickCount() < buzz_end_time) {
            uint32_t period = 1000000 / current_freq / 2; // Período em microssegundos
            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS * 1000; // Convertendo para us
            
            if(now - last_toggle_time >= period) {
                buzzer_state = !buzzer_state;
                gpio_put(BUZZER_A, buzzer_state);
                last_toggle_time = now;
            }
        } else if(current_freq > 0) {
            current_freq = 0; // Tempo acabou
            gpio_put(BUZZER_A, 0);
        }
        
        vTaskDelay(pdMS_TO_TICKS(1)); // Libera o processador
    }
}

//rotina para definição da intensidade de cores do led
uint32_t matrix_rgb(double b, double r, double g)
{
  //unsigned char R, G, B;
  R = r * 255;
  G = g * 255;
  B = b * 255;
  return (G << 24) | (R << 16) | (B << 8);
}

// Desenha na matriz de leds em verde
void desenho_pio_green(double *desenho, uint32_t valor_led, PIO pio, uint sm, double r, double g, double b){

    for (int16_t i = 0; i < NUM_PIXELS; i++) {
            valor_led = matrix_rgb(b = 0.0, r=0.0, desenho[24-i]);
            pio_sm_put_blocking(pio, sm, valor_led);
    }
}

void desenho_pio_yellow(double *desenho, uint32_t valor_led, PIO pio, uint sm, double r, double g, double b){

    for (int16_t i = 0; i < NUM_PIXELS; i++) {
            valor_led = matrix_rgb(b = 0.0, desenho[24-i], desenho[24-i]);
            pio_sm_put_blocking(pio, sm, valor_led);
    }
}

// Desenha na matriz de leds em vermelho
void desenho_pio_red(double *desenho, uint32_t valor_led, PIO pio, uint sm, double r, double g, double b){

    for (int16_t i = 0; i < NUM_PIXELS; i++) {
            valor_led = matrix_rgb(b = 0.0, desenho[24-i], g = 0.0);
            pio_sm_put_blocking(pio, sm, valor_led);
    }
}

void vMatriz () {
    // Setando matriz de leds
    double r = 0.0, b = 0.0 , g = 0.0;
    bool ok;
    ok = set_sys_clock_khz(128000, false);
    pio = pio0;

    uint offset = pio_add_program(pio, &MONITORAMENTO_CHEIAS_program);
    uint sm = pio_claim_unused_sm(pio, true);
    MONITORAMENTO_CHEIAS_program_init(pio, sm, offset, OUT_PIN);

    int op_matriz;

        while(true){
            if(xQueueReceive(xQueueMatrixData, &op_matriz, portMAX_DELAY)){
                switch (op_matriz) {
                    case 1:
                        desenho_pio_green(v, VALOR_LED, pio, sm, R, G, B);
                        break;
                    
                    case 2:
                        desenho_pio_yellow(a, VALOR_LED, pio, sm, R, G, B);
                        break;

                    case 3:
                        desenho_pio_red(x, VALOR_LED, pio, sm, R, G, B);
                        break;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(200));
        }
}

int main()
{
    stdio_init_all();

    gpio_init(botaoB);
    gpio_set_dir(botaoB, GPIO_IN);
    gpio_pull_up(botaoB);
    gpio_set_irq_enabled_with_callback(botaoB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    xQueueJoystickData = xQueueCreate(10, sizeof(uint16_t));
    xQueueDisplayData = xQueueCreate(50, sizeof(dados_display));
    xQueueMatrixData = xQueueCreate(10, sizeof(int));
    xQueueBuzzerData = xQueueCreate(10, sizeof(int));

    xTaskCreate(vjoystick, "Joystick Task", 512, NULL, 1, NULL);
    xTaskCreate(vleds, "LED blue Task", 512, NULL, 1, NULL);
    xTaskCreate(vDisplayTask, "Display Task", 1024, NULL, 1, NULL);
    xTaskCreate(vMatriz, "Matrix Task", 512, NULL, 1, NULL);
    xTaskCreate(vBuzzer, "Buzzer Task", 512, NULL, 1, NULL);

    vTaskStartScheduler();
    panic_unsupported();

}

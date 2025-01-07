#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#define entrada0 12
#define entrada1 14
#define entrada2 27
#define entrada3 26
#define entrada4 25
#define entrada5 33
#define ignicion 32
#define UART_NUM UART_NUM_2
#define BUF_SIZE 1024
#define TASK_MEMORY 1024 * 3
#define NUM_INPUTS 5

typedef struct
{
    int pin;
    const char *valor_esperado;
} entrada_t;

entrada_t entradas[] =
    {
        {entrada0, "IN0 OK"},
        {entrada1, "IN1 OK"},

};

static const char *tag = "UART";
static uint8_t ultimo_valor[BUF_SIZE];

static void uart_task(void *pvParameters)
{
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
    while (1)
    {
        bzero(data, BUF_SIZE);
        int len = uart_read_bytes(UART_NUM, data, BUF_SIZE, pdMS_TO_TICKS(100));
        if (len == 0)
        {
            continue;
        }
        if (len > 0)
        {
            memcpy(ultimo_valor, data, 6);
        }

        // printf("Dato: %s", data);
        uart_write_bytes(UART_NUM, (const char *)data, len);
        ESP_LOGI(tag, "Data received: %s", data);
    }
}

static void init_uart(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_param_config(UART_NUM, &uart_config);

    uart_set_pin(UART_NUM, GPIO_NUM_17, GPIO_NUM_16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uart_driver_install(UART_NUM, BUF_SIZE, BUF_SIZE, 0, NULL, 0);
    xTaskCreate(uart_task, "uart_task", TASK_MEMORY, NULL, 5, NULL);

    ESP_LOGI(tag, "Init UART completed");
}

static void init_led()
{
    gpio_reset_pin(entrada0);
    gpio_set_direction(entrada0, GPIO_MODE_OUTPUT);
    gpio_set_level(entrada0, 1);
    gpio_reset_pin(entrada1);
    gpio_set_direction(entrada1, GPIO_MODE_OUTPUT);
    gpio_set_level(entrada1, 1);
    gpio_reset_pin(entrada2);
    gpio_set_direction(entrada2, GPIO_MODE_OUTPUT);
    gpio_set_level(entrada2, 1);
    gpio_reset_pin(entrada3);
    gpio_set_direction(entrada3, GPIO_MODE_OUTPUT);
    gpio_set_level(entrada3, 1);
    gpio_reset_pin(entrada4);
    gpio_set_direction(entrada4, GPIO_MODE_OUTPUT);
    gpio_set_level(entrada4, 1);
    gpio_reset_pin(entrada5);
    gpio_set_direction(entrada5, GPIO_MODE_OUTPUT);
    gpio_set_level(entrada5, 1);
    gpio_reset_pin(ignicion);
    gpio_set_direction(ignicion, GPIO_MODE_INPUT);

    ESP_LOGI(tag, "Init Led completed");
}

void enviar_mensaje(const char *mensaje)
{
    uart_write_bytes(UART_NUM, mensaje, strlen(mensaje));
}
void test_entrada(entrada_t entrada)
{
    gpio_set_level(entrada.pin, 0);
    vTaskDelay(1500 / portTICK_PERIOD_MS);
    ESP_LOGW(tag,"valor %s",ultimo_valor);
    if (strcmp((char *)ultimo_valor, entrada.valor_esperado) == 0)
    {
        ESP_LOGI(tag, "Valor correcto para entrada %d", entrada.pin);
        gpio_set_level(entrada.pin, 1);
    }
    else
    {
        ESP_LOGE(tag, "Valor incorrecto para la entrada %d,", entrada.pin);
        gpio_set_level(entrada.pin, 1);
    }
}
void test_todas_entradas()
{
    for (int i = 0; i < sizeof(entradas) / sizeof(entradas[0]); i++)
    {
        test_entrada(entradas[i]);
    }
}

void app_main(void)
{
    // vTaskDelay(1000 / portTICK_PERIOD_MS);
    init_led();
    init_uart();
    ESP_LOGW(tag, "Iniciando....");
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    ESP_LOGI(tag, "Iniciado!!!");

    for (;;)
    {
        int estado_boton = gpio_get_level(ignicion);
        ESP_LOGI(tag, "Estado de boton: %d\n", estado_boton);
        if (estado_boton == 0)
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            test_todas_entradas();
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
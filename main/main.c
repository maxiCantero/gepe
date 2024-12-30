#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"

static const char *tag = "UART";
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

static QueueHandle_t uart_queue;

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

        xQueueSend(uart_queue, data, portMAX_DELAY);

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
    // gpio_reset_pin(ignicion);
    // gpio_set_direction(ignicion, GPIO_MODE_OUTPUT);

    ESP_LOGI(tag, "Init Led completed");
}

uint8_t *read_uart_data()
{
    uint8_t *received_data = (uint8_t *)malloc(BUF_SIZE);
    if (xQueueReceive(uart_queue, received_data, portMAX_DELAY) == pdTRUE)
    {
        return received_data;
    }
    else
    {
        return NULL;
    }
}

void enviar_mensaje(const char *mensaje)
{
    uart_write_bytes(UART_NUM, mensaje, strlen(mensaje));
}

void test_entradas(void)
{
    ESP_LOGI(tag, "test IN00");
    gpio_set_level(entrada0, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void app_main(void)
{
    init_led();
    init_uart();
    test_entradas();
    uart_queue = xQueueCreate(10, BUF_SIZE*2);

    for (;;)
    {
        gpio_set_level(entrada1, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(entrada1, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        uint8_t *data = read_uart_data();

        if (data != NULL)
        {
            int data_length = strlen((char*)data);

            for (int i = 0; i < data_length; i++)
            {
                printf("%c",data[i]);
                
            }
            printf ("\n");
            free(data);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
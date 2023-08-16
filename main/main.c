#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/timer.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "driver/pcnt.h"
#include "driver/timer.h"
#include "driver/periph_ctrl.h"
#include "driver/gpio.h"

#include "flow.h"

//ToF, Coeff, Flow time, Offset, Water Temp, ID

#define TXD_PIN (GPIO_NUM_1)  //GPIO_NUM_17  GPIO_NUM_1 UART_NUM_0
#define RXD_PIN (GPIO_NUM_3)  //GPIO_NUM_16  GPIO_NUM_3
#define RX_BUF_SIZE (128)

#define PIN_FLOW 34

static const char *TAG = "RIG_BOARD";
static int16_t flow_cntr;
static int16_t flag;

esp_timer_handle_t periodic_timer;

void uart_config() {
    	//Init UART1
	uart_config_t uart_config = {
		.baud_rate = 115200,
		.data_bits = UART_DATA_8_BITS,
		.parity    = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_APB,
	};
	uart_driver_install(UART_NUM_0, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
	uart_param_config(UART_NUM_0, &uart_config);
	uart_set_pin(UART_NUM_0, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static void flow_timer_cb(void* arg)
{
    pcnt_get_counter_value(PCNT_UNIT_0, &flow_cntr);
    pcnt_counter_clear(PCNT_UNIT_0);
    flag = 1;
}

void PCNT_init(void){
    
    pcnt_config_t pcnt_config = {
            // Set PCNT input signal and control GPIOs
            .pulse_gpio_num = PIN_FLOW,
            .channel = PCNT_CHANNEL_0,
            .unit = PCNT_UNIT_0,
            .pos_mode = PCNT_COUNT_INC,   // Count up on the positive edge
            .neg_mode = PCNT_COUNT_DIS,   // Keep the counter value on the negative edge
            .lctrl_mode = PCNT_MODE_KEEP, 
            .hctrl_mode = PCNT_MODE_KEEP,    

    };

    // /* Initialize PCNT unit */
    pcnt_unit_config(&pcnt_config);
    pcnt_set_filter_value(PCNT_UNIT_0, 1000);
    pcnt_filter_enable(PCNT_UNIT_0);
    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);
    const esp_timer_create_args_t periodic_timer_args = {
            .callback = &flow_timer_cb,
    };
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 1000000));
}

static void flow_meter_task(void *arg) {
    int32_t tof_value = 0;
    uint16_t status = 0;

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    while (1) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        if(flag) {
            flag = 0;
            //if(flow_cntr > 1) {                 //Jeżeli woda choć trochę płynie to printuj, jeśli zakręcone to nie printuj
                status = flow_read_dtof(&tof_value);
                printf("%d\n", flow_cntr);
                vTaskDelay(10 / portTICK_PERIOD_MS);
                printf("Tof: %ld\n", (long)tof_value);
                //printf("%du\n", (unsigned int)status);
                vTaskDelay(100 / portTICK_PERIOD_MS);
           // }
        }
    }
}


void app_main(void)
{
    uart_config();
    PCNT_init();
    flow_init();

    ESP_LOGI(TAG, "ESP_RIG107");

    xTaskCreate(flow_meter_task, "flow_meter_task", 4096, NULL, 9, NULL);

    while(1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
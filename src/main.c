#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

#define BUF_SIZE (1024 * 2)
#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)
static const char *RX_TASK_TAG = "RX_TASK";

void uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 100000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_2,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,

    };
    
    uart_set_line_inverse(UART_NUM_2, UART_SIGNAL_TXD_INV);
    esp_err_t inverse_status = uart_set_line_inverse(UART_NUM_2, UART_SIGNAL_RXD_INV);
    printf(" UART_STATUS %d   ", inverse_status);
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}
typedef enum
{
    STATE_0_INVALID,
    STATE_1_FIND_HEADER,
    STATE_2_FIND_FOOTER,
    STARTE_3_GOT_PACKET
} sbus_state_t;

static void rx_task(void *arg)
{
    // esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    // ESP_LOGI(RX_TASK_TAG,"Reading");
    uint8_t *rx_data = (uint8_t *)malloc(BUF_SIZE + 1);
    while (1)
    {
        int length = 0;
        uart_get_buffered_data_len(UART_NUM_2, (size_t *)&length);
        const int len = uart_read_bytes(UART_NUM_2, rx_data, length, (20 / portTICK_RATE_MS));
        int counter = 0;
        while (len > 0)
        {

            if (rx_data[counter] == 0x0f)
            {
                printf("Got frame\r\n");
            }
            printf("%x  ", rx_data[counter]);
            counter++;
        }

        //  if()
        //  {
        // 			//rx_data[len] = '\0';
        // 			for(int  x= 0; x<len;x++)
        // 			{
        //                 if(rx_data[x] ==0x0f )
        //                 {
        //                     printf("Got frame\r\n" );
        //                 }
        // 				printf("%x  ", rx_data[x]);
        // 			}
        // 			//printf("%s", data);
        // 			//fflush(stdout);
        //             vTaskDelay(1000/portTICK_RATE_MS);
        // 		}
    }
    free(rx_data);
}
void app_main()
{
    uart_init();

    xTaskCreate(&rx_task, "uart_rx_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
}
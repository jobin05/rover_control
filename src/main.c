#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "../lib/main.h"

#define BUF_SIZE (1024 * 2)
#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)
static const char *RX_TASK_TAG = "RX_TASK";
void sbus_parser(sbus_channel_data_t *data);
static QueueHandle_t sbus_channel_queue;

void uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 100000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,

    };
    uart_driver_install(UART_NUM_2, BUF_SIZE, 0, 0, NULL, 0);
    uart_set_line_inverse(UART_NUM_2, UART_SIGNAL_TXD_INV);
    esp_err_t inverse_status = uart_set_line_inverse(UART_NUM_2, UART_SIGNAL_RXD_INV);
    printf(" UART_STATUS %d   ", inverse_status);
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}
typedef enum
{
    SBUS_STATE_0_SEARCH_FOR_HEADER,
    SBUS_STATE_1_FOUND_HEADER,
    SBUS_STATE_2_SEARCH_FOOTER,
} sbus_state_t;

void sbus_parser(sbus_channel_data_t *data)
{

    data->channels_out[0] = (int16_t)((payload[0] | payload[1] << 8) & 0x07FF);
    data->channels_out[1] = (int16_t)((payload[1] >> 3 | payload[2] << 5) & 0x07FF);
    data->channels_out[2] = (int16_t)((payload[2] >> 6 | payload[3] << 2 | payload[4] << 10) & 0x07FF);
    data->channels_out[3] = (int16_t)((payload[4] >> 1 | payload[5] << 7) & 0x07FF);
    data->channels_out[4] = (int16_t)((payload[5] >> 4 | payload[6] << 4) & 0x07FF);
    data->channels_out[5] = (int16_t)((payload[6] >> 7 | payload[7] << 1 | payload[8] << 9) & 0x07FF);
    data->channels_out[6] = (int16_t)((payload[8] >> 2 | payload[9] << 6) & 0x07FF);
    data->channels_out[7] = (int16_t)((payload[9] >> 5 | payload[10] << 3) & 0x07FF);
    data->channels_out[8] = (int16_t)((payload[11] | payload[12] << 8) & 0x07FF);
    data->channels_out[9] = (int16_t)((payload[12] >> 3 | payload[13] << 5) & 0x07FF);
    data->channels_out[10] = (int16_t)((payload[13] >> 6 | payload[14] << 2 | payload[15] << 10) & 0x07FF);
    data->channels_out[11] = (int16_t)((payload[15] >> 1 | payload[16] << 7) & 0x07FF);
    data->channels_out[12] = (int16_t)((payload[16] >> 4 | payload[17] << 4) & 0x07FF);
    data->channels_out[13] = (int16_t)((payload[17] >> 7 | payload[18] << 1 | payload[19] << 9) & 0x07FF);
    data->channels_out[14] = (int16_t)((payload[19] >> 2 | payload[20] << 6) & 0x07FF);
    data->channels_out[15] = (int16_t)((payload[20] >> 5 | payload[21] << 3) & 0x07FF);

    data->failsafe = payload[22] & SBUS_FAILSAFE;
    if(data->failsafe  == 1)
    {
        data->lost_frame_count = 1;
    }

    if (payload[22] & SBUS_LOST_FRAME)
    {
        if(data->lost_frame_count == sizeof(uint16_t))
        {
            data->lost_frame_count = 1;
        }
        data->lost_frame_count++;
    }

    if (xQueueSend(sbus_channel_queue, (void *)data, 10) != pdTRUE)
    {
        //printf("Error Sending the queue");
    }
    // printf("%d  %d  %d  %d  %d  %d   \n", data->failsafe,
    //        data->lost_frame_count,
    //        data->channels_out[0],
    //        data->channels_out[1],
    //        data->channels_out[2],
    //        data->channels_out[3]);
}

void rx_task(void *arg)
{

    sbus_state_t sbus_state = SBUS_STATE_0_SEARCH_FOR_HEADER;
    sbus_channel_data_t sbus_channel_data;
    static uint8_t payload_counter = 0;
    uint8_t current_byte, prev_byte;
    prev_byte = _sbusFooter;
    // esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    // ESP_LOGI(RX_TASK_TAG,"Reading");
    uart_init();

    uint8_t *rx_data = (uint8_t *)malloc(BUF_SIZE + 1);
    while (1)
    {
        int length = 0;
        uart_get_buffered_data_len(UART_NUM_2, (size_t *)&length);
        int to_read = 25;
        const int len = uart_read_bytes(UART_NUM_2, rx_data, to_read, (20 / portTICK_RATE_MS));
        int counter = 0;

        while (len >= counter)
        {
            current_byte = rx_data[counter];
            // rx_data[counter] = 0;
            // printf("%x  ", current_byte );
            switch (sbus_state)
            {
            case SBUS_STATE_0_SEARCH_FOR_HEADER:
                if ((current_byte == _sbusHeader) && (prev_byte == _sbusFooter))
                {
                    sbus_state = SBUS_STATE_1_FOUND_HEADER;
                    // printf("Got Header \n  ");
                }
                else
                {
                    sbus_state = SBUS_STATE_0_SEARCH_FOR_HEADER;
                }
                break;
            case SBUS_STATE_1_FOUND_HEADER:
                if (payload_counter < payloadSize - 1)
                {

                    payload[payload_counter++] = current_byte;
                }
                else
                {
                    // printf("Count Done \n  ");
                    payload_counter = 0;
                    sbus_state = SBUS_STATE_2_SEARCH_FOOTER;
                }
                break;
            case SBUS_STATE_2_SEARCH_FOOTER:

                if (current_byte == _sbusFooter)
                {

                    sbus_state = SBUS_STATE_0_SEARCH_FOR_HEADER;
                    // printf("Got Footer \n  ");
                    fflush(stdout);
                    sbus_parser(&sbus_channel_data);
                    memset(&payload, 0, payloadSize);
                    break;
                }
                else
                {

                    //printf(" No Footer \n  ");
                }

                break;

            default:
                break;
            }

            counter++;
            prev_byte = current_byte;
        }

        vTaskDelay(3 / portTICK_RATE_MS);

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
void motor_control_task()
{
    sbus_channel_data_t  sbus_channel_data;
    while (1)
    {
        // See if there's a message in the queue (do not block)
        bool rec_message = false;
        if (xQueueReceive(sbus_channel_queue , (void *)&sbus_channel_data, 0) != pdTRUE)
        {
           rec_message = true;

        //     printf(" Rec  %d  %d  %d  %d  %d  %d   \n", sbus_channel_data.failsafe,
        //    sbus_channel_data.lost_frame_count,
        //    sbus_channel_data.channels_out[0],
        //    sbus_channel_data.channels_out[1],
        //    sbus_channel_data.channels_out[2],
        //    sbus_channel_data.channels_out[3]);
        }
        if( rec_message)
        {
            
        }
        vTaskDelay(3 / portTICK_RATE_MS);
    }
}
void app_main()
{
    sbus_channel_queue = xQueueCreate(10, sizeof(sbus_channel_data_t));

    xTaskCreate(&rx_task, "uart_rx_task", 1024 * 2, NULL, tskIDLE_PRIORITY+1, NULL);
    xTaskCreate(&motor_control_task, "print_queue", 1024*2, NULL, tskIDLE_PRIORITY, NULL);
}
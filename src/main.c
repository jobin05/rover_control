#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "../lib/main.h"

#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "../lib/sbus/inc/sbus.h"
#define BUF_SIZE (1024)
#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)
static const char *RX_TASK_TAG = "RX_TASK";
void sbus_parser(sbus_channel_data_t *data);
static QueueHandle_t sbus_channel_queue;

#define GPIO_PWM0A_OUT 22 // Set GPIO 15 as PWM0A
#define GPIO_PWM0B_OUT 23 // Set GPIO 16 as PWM0B
#define L_DIRECTION_FORWORD GPIO_NUM_32
#define L_DIRECTION_BACKWORD GPIO_NUM_33
#define R_DIRECTION_FORWORD GPIO_NUM_34
#define R_DIRECTION_BACKWORD GPIO_NUM_35



// static long map(long x, long in_min, long in_max, long out_min, long out_max) {
//   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
// }

static int map(int x, int in_min, int in_max, int out_min, int out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * @brief Configure MCPWM module for brushed dc motor
 */
static void mcpwm_example_brushed_motor_control(void *arg)
{
    //mcpwm gpio initialization
        mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
int ON= 1;
int OFF = 0;
    gpio_set_direction(L_DIRECTION_FORWORD ,GPIO_MODE_OUTPUT);
     gpio_set_direction(L_DIRECTION_BACKWORD,GPIO_MODE_OUTPUT);
     gpio_set_direction(R_DIRECTION_FORWORD ,GPIO_MODE_OUTPUT);
     gpio_set_direction(R_DIRECTION_BACKWORD,GPIO_MODE_OUTPUT);
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000; // frequency = 500Hz,
    pwm_config.cmpr_a = 0;       // duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;       // duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config); // Configure PWM0A & PWM0B with above settings
    sbus_channel_data_t sbus_channel_data;
    int counter = 0;

    while (1)
    {
        bool rec_message = false;
        if (xQueueReceive(sbus_channel_queue, (void *)&sbus_channel_data, 0) != pdTRUE)
        {
            rec_message = true;
            // printf(" Rec  %d  %d  %d  %d  %d  %d   \n", sbus_channel_data.failsafe,
            //        sbus_channel_data.lost_frame,
            //        sbus_channel_data.channels_out[0],
            //        sbus_channel_data.channels_out[1],
            //        sbus_channel_data.channels_out[2],
            //        sbus_channel_data.channels_out[3]);

            if (sbus_channel_data.failsafe == 1)
            {
                sbus_channel_data.channels_out[1] = 0;
                sbus_channel_data.channels_out[0] = 0;
            }
        }
        if (rec_message)
        {
            int y_axis = map(sbus_channel_data.channels_out[1], 240, 1807, -100, 100);
            int x_axis = map(sbus_channel_data.channels_out[0], 240, 1807, -100, 100);
            int l_wheel = y_axis + x_axis;
            int r_wheel = y_axis - x_axis;

            if (l_wheel > 100)
            {
                l_wheel = 100;
            }
            if (r_wheel > 100)
            {
                r_wheel = 100;
            }
            if (l_wheel < -100)
            {
                l_wheel = -100;
            }
            if (r_wheel < -100)
            {
                r_wheel = -100;
            }
        
        printf("%d  %d\n", l_wheel, r_wheel);
        
        if(l_wheel< 0)
        {
            gpio_set_level(L_DIRECTION_FORWORD, ON);
            gpio_set_level(L_DIRECTION_BACKWORD, OFF);
        }
        else
        {
            gpio_set_level(L_DIRECTION_FORWORD, OFF);
            gpio_set_level(L_DIRECTION_BACKWORD, ON);
        }

          if(r_wheel< 0)
        {
            gpio_set_level(R_DIRECTION_FORWORD, ON);
            gpio_set_level(R_DIRECTION_BACKWORD, OFF);
        }
        else
        {
            gpio_set_level(R_DIRECTION_FORWORD, OFF);
            gpio_set_level(R_DIRECTION_BACKWORD,ON);
        }
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, abs(r_wheel));
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, abs(l_wheel));
        // brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, l_wheel);
        // brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, r_wheel);
    }

    vTaskDelay(5 / portTICK_RATE_MS);
}

// brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 20.0);
// vTaskDelay(5000 / portTICK_RATE_MS);
// brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, 50.0);
// vTaskDelay(5000 / portTICK_RATE_MS);
// brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
// vTaskDelay(2000 / portTICK_RATE_MS);

// gpio_set_direction(PUSH_BUTTON_PIN_SPEED, GPIO_MODE_INPUT);

vTaskDelay(5 / portTICK_RATE_MS);
}

void uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 100000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_2,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,

    };
    uart_driver_install(UART_NUM_2, BUF_SIZE, 0, 0, NULL, 0);
    uart_set_line_inverse(UART_NUM_2, UART_SIGNAL_TXD_INV);
    esp_err_t inverse_status = uart_set_line_inverse(UART_NUM_2, UART_SIGNAL_RXD_INV);
    // printf(" UART_STATUS %d   ", inverse_status);
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

    data->lost_frame = payload[22] & SBUS_LOST_FRAME;

    if (xQueueSend(sbus_channel_queue, (void *)data, 10) != pdTRUE)
    {
        // printf("Error Sending the queue");
    }
    // if(1024 != data->channels_out[3]  )
    //          {
    //          printf( "%d  %d \n",data->channels_out[3], data->channels_out[3]);
    //          }
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
            if (SBUS_FRAME_NOT_READY == SBus_ParseByte(rx_data[counter]))
            {
                SBus_DecodeFrame();

                for (int x = 0; x < 7; x++)
                {
                    sbus_channel_data.channels_out[x] = SBus_GetChannel(x);
                }
                if (xQueueSend(sbus_channel_queue, (void *)&sbus_channel_data, 10) != pdTRUE)
                {
                    printf("Error Sending the queue");
                }
            }
            // current_byte = rx_data[counter];
            // // rx_data[counter] = 0;
            // printf("%x  ", rx_data[counter]);
            // switch (sbus_state)
            // {
            // case SBUS_STATE_0_SEARCH_FOR_HEADER:
            //     if ((current_byte == _sbusHeader) && (prev_byte == _sbusFooter))
            //     {
            //         sbus_state = SBUS_STATE_1_FOUND_HEADER;
            //         // printf("Got Header \n  ");
            //     }
            //     else
            //     {
            //         sbus_state = SBUS_STATE_0_SEARCH_FOR_HEADER;
            //     }
            //     break;
            // case SBUS_STATE_1_FOUND_HEADER:
            //     if (payload_counter < payloadSize - 1)
            //     {

            //         payload[payload_counter++] = current_byte;
            //     }
            //     else
            //     {
            //         // printf("Count Done \n  ");
            //         payload_counter = 0;
            //         sbus_state = SBUS_STATE_2_SEARCH_FOOTER;
            //     }
            //     break;
            // case SBUS_STATE_2_SEARCH_FOOTER:

            //     if (current_byte == _sbusFooter)
            //     {

            //         sbus_state = SBUS_STATE_0_SEARCH_FOR_HEADER;
            //         // printf("Got Footer \n  ");
            //         fflush(stdout);
            //         sbus_parser(&sbus_channel_data);
            //         memset(&payload, 0, payloadSize);
            //         break;
            //     }
            //     else
            //     {

            //         sbus_state = SBUS_STATE_0_SEARCH_FOR_HEADER;
            //         // printf("Got Footer \n  ");
            //         //fflush(stdout);

            //         break;
            //     }

            //     break;

            // default:
            //     break;
            // }

            counter++;
            // prev_byte = current_byte;
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
    sbus_channel_data_t sbus_channel_data;
    while (1)
    {
        // See if there's a message in the queue (do not block)
        bool rec_message = false;
        if (xQueueReceive(sbus_channel_queue, (void *)&sbus_channel_data, 0) != pdTRUE)
        {
            rec_message = true;

            printf(" Rec  %d  %d  %d  %d  %d  %d   \n", sbus_channel_data.failsafe,
                   sbus_channel_data.lost_frame,
                   sbus_channel_data.channels_out[0],
                   sbus_channel_data.channels_out[1],
                   sbus_channel_data.channels_out[2],
                   sbus_channel_data.channels_out[3]);
        }
        if (rec_message)
        {
        }
        vTaskDelay(3 / portTICK_RATE_MS);
    }
}
void app_main()
{
    sbus_channel_queue = xQueueCreate(10, sizeof(sbus_channel_data_t));

    xTaskCreate(&rx_task, "uart_rx_task", 1024 * 2, NULL, tskIDLE_PRIORITY + 2, NULL);
    // xTaskCreate(&motor_control_task, "print_queue", 1024*2, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(mcpwm_example_brushed_motor_control, "mcpwm_example_brushed_motor_control", 4096, NULL, tskIDLE_PRIORITY + 1, NULL);
}
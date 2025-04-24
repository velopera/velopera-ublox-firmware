#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "gatts.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"

#include "can_data_handler.h"
#include "driver/twai.h"
#include "esp_err.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "mc6470.h"

/* --------------------- Definitions and static variables ------------------ */
#define NO_OF_ITERS 3
#define RX_TASK_PRIO 9
#define TX_GPIO_NUM (gpio_num_t)32
#define RX_GPIO_NUM (gpio_num_t)25
#define EXAMPLE_TAG "TWAI Listen Only"

static const int RX_BUF_SIZE = 1024;
#define TXD_PIN (gpio_num_t)23
#define RXD_PIN (gpio_num_t)18

static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();

static const twai_general_config_t g_config = { .mode = TWAI_MODE_LISTEN_ONLY,
                                                .tx_io = TX_GPIO_NUM,
                                                .rx_io = RX_GPIO_NUM,
                                                .clkout_io = TWAI_IO_UNUSED,
                                                .bus_off_io = TWAI_IO_UNUSED,
                                                .tx_queue_len = 0,
                                                .rx_queue_len = 5,
                                                .alerts_enabled =
                                                  TWAI_ALERT_NONE,
                                                .clkout_divider = 0 };

const uart_config_t uart_config = {
  .baud_rate = 115200,
  .data_bits = UART_DATA_8_BITS,
  .parity = UART_PARITY_DISABLE,
  .stop_bits = UART_STOP_BITS_1,
  .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
  .source_clk = UART_SCLK_APB,
};

struct MC6470_Dev_t* compass;
MC6470_MagReading magData;
MC6470_AccelReading accData;

static SemaphoreHandle_t rx_sem;
static CANVelopera velopera_handler;

/* --------------------------- Tasks and Functions -------------------------- */
int count = 0;

int
send_data_to_nrf(char* data)
{
  int len = strlen(data);
  char rxdata[64] = ""; // Ensure space for response
  int txBytes = uart_write_bytes(UART_NUM_1, data, len);
  uart_read_bytes(UART_NUM_1, rxdata, len, 20 / portTICK_PERIOD_MS);
  // ESP_LOGI("send_data_to_nrf", "Wrote %d bytes of msg=> %s", txBytes, data);
  return txBytes;
}

int
updateSpeed(void* can_state, const float newSpeed, char valid)
{
  char msg[128];
  count++;

  if (count > 300) {
    MC6470_getData(compass, &magData, &accData);
    sprintf(msg,
            "{\"speed\":%d,\"mag\":[%d,%d,%d]}\n",
            (int)newSpeed,
            (int)magData.x,
            (int)magData.y,
            (int)magData.z);
    send_data_to_nrf(msg);
    count = 0;
  }
  return 0;
}

static void
twai_receive_task(void* arg)
{
  xSemaphoreTake(rx_sem, portMAX_DELAY);

  CANVelopera_init(&velopera_handler); // Initialize CAN handler
  velopera_handler.base.funcs.callbacks.updateSpeed = updateSpeed;

  while (1) {
    twai_message_t rx_msg;
    twai_receive(&rx_msg, 5000 / portTICK_PERIOD_MS);

    velopera_handler.base.funcs.handle_message(&velopera_handler.base.funcs,
                                               velopera_handler.base.state,
                                               rx_msg.identifier,
                                               (*(uint64_t*)rx_msg.data),
                                               8,
                                               0);
  }

  xSemaphoreGive(rx_sem);
  vTaskDelete(NULL);
}

void
app_main(void)
{
  rx_sem = xSemaphoreCreateBinary();
  gatts_init();
  xTaskCreatePinnedToCore(twai_receive_task,
                          "TWAI_rx",
                          4096,
                          NULL,
                          RX_TASK_PRIO,
                          NULL,
                          tskNO_AFFINITY);

  compass = (struct MC6470_Dev_t*)malloc(sizeof(struct MC6470_Dev_t));

  uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
  uart_param_config(UART_NUM_1, &uart_config);
  uart_set_pin(
    UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  MC6470_Init(compass, MC6470_ACCEL_ADDRESS_VDD);
  esp_err_t ret = MC6470_begin(compass);

  ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
  // ESP_LOGI(EXAMPLE_TAG, "Driver installed");
  ESP_ERROR_CHECK(twai_start());
  // ESP_LOGI(EXAMPLE_TAG, "Driver started");

  xSemaphoreGive(rx_sem); // Start RX task
  vTaskDelay(pdMS_TO_TICKS(100));
  xSemaphoreTake(rx_sem, portMAX_DELAY); // Wait for RX task to complete

  ESP_ERROR_CHECK(twai_stop());
  // ESP_LOGI(EXAMPLE_TAG, "Driver stopped");
  ESP_ERROR_CHECK(twai_driver_uninstall());
  // ESP_LOGI(EXAMPLE_TAG, "Driver uninstalled");

  vSemaphoreDelete(rx_sem);
}

// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>
// #include <inttypes.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/event_groups.h"
// #include "gatts.h"
// #include "esp_system.h"
// #include "//ESP_LOG.h"
// #include "gatts.h"

// #include "sdkconfig.h"
// #include "esp_system.h"

// #include "driver/gpio.h"

// #include "driver/uart.h"

// #include "freertos/queue.h"
// #include "freertos/semphr.h"
// #include "esp_err.h"
// #include "driver/twai.h"
// #include "can_data_handler.h"
// #include "mc6470.h"
// /* --------------------- Definitions and static variables ------------------
// */
// // Example Configuration
// #define NO_OF_ITERS 3
// #define RX_TASK_PRIO 9
// #define TX_GPIO_NUM (gpio_num_t)32 // nina w106 => pin 32, others => pin 33
// #define RX_GPIO_NUM (gpio_num_t)25
// #define EXAMPLE_TAG "TWAI Listen Only"

// static const int RX_BUF_SIZE = 1024;

// #define TXD_PIN (gpio_num_t)23
// #define RXD_PIN (gpio_num_t)18

// static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
// static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();

// //  Set TX queue length to 0 due to listen only mode
// static const twai_general_config_t g_config = {.mode = TWAI_MODE_LISTEN_ONLY,
//                                                .tx_io = TX_GPIO_NUM,
//                                                .rx_io = RX_GPIO_NUM,
//                                                .clkout_io = TWAI_IO_UNUSED,
//                                                .bus_off_io = TWAI_IO_UNUSED,
//                                                .tx_queue_len = 0,
//                                                .rx_queue_len = 5,
//                                                .alerts_enabled =
//                                                TWAI_ALERT_NONE,
//                                                .clkout_divider = 0};

// const uart_config_t uart_config = {
//     .baud_rate = 115200,
//     .data_bits = UART_DATA_8_BITS,
//     .parity = UART_PARITY_DISABLE,
//     .stop_bits = UART_STOP_BITS_1,
//     .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
//     .source_clk = UART_SCLK_APB,
// };
// struct MC6470_Dev_t *compass;
// MC6470_MagReading magData;
// MC6470_AccelReading accData;
// // struct MC6470_Dev_t compass = {
// //    .ctx = NULL,
// //    .accel_address = MC6470_ACCEL_ADDRESS_7_BITS_1,
// //    .mag_address = MC6470_MAG_ADDRESS,
// //    .accel_resolution = MC6470_ACCEL_OUTCFG_RES_8_Bits,
// //    .accel_range = MC6470_ACCEL_OUTCFG_RANGE_16G,
// //};

// static SemaphoreHandle_t rx_sem;

// CANDataHandler *velopera_handler = new CANVelopera();

// /* --------------------------- Tasks and Functions --------------------------
// */ int count = 0;

// int send_data_to_nrf(char *data)
// {
//     const int len = strlen(data);
//     char rxdata[] = "";
//     const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
//     uart_read_bytes(UART_NUM_1, rxdata, len, 20 / portTICK_PERIOD_MS);
//     ESP_LOGI("send_data_to_nrf", "Wrote %d bytes of msg=> %s", txBytes,
//     data); return txBytes;
// }

// int updateSpeed(void *can_state, const float newSpeed, char valid)
// {
//     char msg[128];
//     count++;
//     if (count > 300)
//     {
//         MC6470_getData(compass, &magData, &accData);
//         // ESP_LOGI("COMPASS", "(x, y, z) (%g, %g, %g)", accData.x,
//         accData.y, accData.z); sprintf(msg,
//         "{\"speed\":%d,\"mag\":[%d,%d,%d]}\n", (int)newSpeed, (int)magData.x,
//         (int)magData.y, (int)magData.z); send_data_to_nrf(msg); count = 0;
//     }

//     return 0;
// }

// static void twai_receive_task(void *arg)
// {
//     xSemaphoreTake(rx_sem, portMAX_DELAY);

//     // dbcc_time_stamp_t time_stamp = 0;
//     velopera_handler->init();
//     velopera_handler->funcs.callbacks.updateSpeed = updateSpeed;
//     while (1)
//     {
//         twai_message_t rx_msg;

//         twai_receive(&rx_msg, 5000 / portTICK_PERIOD_MS);

//         int ext = rx_msg.extd;
//         int rtr = rx_msg.rtr;

//         // printf("can frame: can0 %" PRIx32 "#", rx_msg.identifier);
//         for (size_t i = 0; i < rx_msg.data_length_code; i++)
//         {
//             // printf("%02x", rx_msg.data[i]);
//         }
//         // printf("\r\n");

//         // printf("uint64_t raw_data %llu \n\r", (*(uint64_t *)rx_msg.data));

//         velopera_handler->handleCANData(rx_msg.identifier, (*(uint64_t
//         *)rx_msg.data));
//     }

//     xSemaphoreGive(rx_sem);
//     vTaskDelete(NULL);
// }

// extern "C" void app_main(void)
// {
//     rx_sem = xSemaphoreCreateBinary();
//     gatts_init();
//     xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL,
//     RX_TASK_PRIO, NULL, tskNO_AFFINITY); compass = (struct MC6470_Dev_t
//     *)malloc(sizeof(struct MC6470_Dev_t));

//     uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
//     uart_param_config(UART_NUM_1, &uart_config);
//     uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE,
//     UART_PIN_NO_CHANGE);

//     // send_data_to_nrf("FF\n");
//     MC6470_Init(compass, MC6470_ACCEL_ADDRESS_VDD);
//     esp_err_t ret = MC6470_begin(compass);
//     // while (1)
//     //{
//     //     MC6470_getData(compass, &magData, &accData);
//     //     ESP_LOGI("COMPASS", "acc(x, y, z) (%g, %g, %g), mag(x, y, z) (%g,
//     %g, %g)", accData.x, accData.y, accData.z, magData.x, magData.y,
//     magData.z);
//     //     if (ret != MC6470_Status_OK)
//     //     {
//     //         ESP_LOGE("main", "MC6470_begin ERR %d", ret);
//     //     }
//     //     vTaskDelay(500 / portTICK_PERIOD_MS);
//     // }
//     //   Install and start TWAI driver
//     ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
//     ESP_LOGI(EXAMPLE_TAG, "Driver installed");
//     ESP_ERROR_CHECK(twai_start());
//     ESP_LOGI(EXAMPLE_TAG, "Driver started");

//     xSemaphoreGive(rx_sem); // Start RX task
//     vTaskDelay(pdMS_TO_TICKS(100));
//     xSemaphoreTake(rx_sem, portMAX_DELAY); // Wait for RX task to complete

//     // Stop and uninstall TWAI driver
//     ESP_ERROR_CHECK(twai_stop());
//     ESP_LOGI(EXAMPLE_TAG, "Driver stopped");
//     ESP_ERROR_CHECK(twai_driver_uninstall());
//     ESP_LOGI(EXAMPLE_TAG, "Driver uninstalled");

//     // Cleanup
//     vSemaphoreDelete(rx_sem);
// }

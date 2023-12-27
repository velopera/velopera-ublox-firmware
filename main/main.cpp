#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "gatts.h"
#include "esp_system.h"
#include "esp_log.h"
#include "gatts.h"
#include "can_data_handler.h"

#include "ktm-a612.h"
#include "sdkconfig.h"

#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "driver/twai.h"
#include "can_data_handler.h"
#include "ktm-a612.h"
/* --------------------- Definitions and static variables ------------------ */
// Example Configuration
#define NO_OF_ITERS 3
#define RX_TASK_PRIO 9
#define TX_GPIO_NUM GPIO_NUM_33
#define RX_GPIO_NUM GPIO_NUM_25
#define EXAMPLE_TAG "TWAI Listen Only"

static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
// can_obj_ktm_a612_h_t ktm_a612;
//  Set TX queue length to 0 due to listen only mode
static const twai_general_config_t g_config = {.mode = TWAI_MODE_LISTEN_ONLY,
                                               .tx_io = TX_GPIO_NUM,
                                               .rx_io = RX_GPIO_NUM,
                                               .clkout_io = TWAI_IO_UNUSED,
                                               .bus_off_io = TWAI_IO_UNUSED,
                                               .tx_queue_len = 0,
                                               .rx_queue_len = 5,
                                               .alerts_enabled = TWAI_ALERT_NONE,
                                               .clkout_divider = 0};

static SemaphoreHandle_t rx_sem;

CANDataHandler *pja612_handler = new CANPJA612();

/* --------------------------- Tasks and Functions -------------------------- */

static void twai_receive_task(void *arg)
{
    xSemaphoreTake(rx_sem, portMAX_DELAY);

    // dbcc_time_stamp_t time_stamp = 0;
    pja612_handler->init();
    while (1)
    {
        twai_message_t rx_msg;

        twai_receive(&rx_msg, portMAX_DELAY);

        int ext = rx_msg.extd;
        int rtr = rx_msg.rtr;

        printf("can frame: can0 %" PRIx32 "#", rx_msg.identifier);
        for (size_t i = 0; i < rx_msg.data_length_code; i++)
        {
            printf("%02x", rx_msg.data[i]);
        }
        printf("\r\n");

        printf("uint64_t raw_data %llu \n\r", (*(uint64_t *)rx_msg.data));

        pja612_handler->handleCANData(rx_msg.identifier, (*(uint64_t *)rx_msg.data));

        vTaskDelay(100);
    }

    xSemaphoreGive(rx_sem);
    vTaskDelete(NULL);
}

extern "C" void app_main(void)
{
    rx_sem = xSemaphoreCreateBinary();
    gatts_init();
    xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, RX_TASK_PRIO, NULL, tskNO_AFFINITY);

    // Install and start TWAI driver
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(EXAMPLE_TAG, "Driver installed");
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(EXAMPLE_TAG, "Driver started");

    xSemaphoreGive(rx_sem); // Start RX task
    vTaskDelay(pdMS_TO_TICKS(100));
    xSemaphoreTake(rx_sem, portMAX_DELAY); // Wait for RX task to complete

    // Stop and uninstall TWAI driver
    ESP_ERROR_CHECK(twai_stop());
    ESP_LOGI(EXAMPLE_TAG, "Driver stopped");
    ESP_ERROR_CHECK(twai_driver_uninstall());
    ESP_LOGI(EXAMPLE_TAG, "Driver uninstalled");

    // Cleanup
    vSemaphoreDelete(rx_sem);
}

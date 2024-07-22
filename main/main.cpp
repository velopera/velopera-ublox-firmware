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

#include "sdkconfig.h"
#include "esp_system.h"

#include "driver/gpio.h"

#include "driver/uart.h"

#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "driver/twai.h"
#include "can_data_handler.h"
#include "mc6470.h"
#include "bme280.h"
#include "driver/i2c.h"

/* --------------------- Definitions and static variables ------------------ */
// Example Configuration
#define NO_OF_ITERS 3
#define RX_TASK_PRIO 9
#define TX_GPIO_NUM (gpio_num_t)32 // nina w106 => pin 32, others => pin 33
#define RX_GPIO_NUM (gpio_num_t)25
#define EXAMPLE_TAG "TWAI Listen Only"

#define SDA_PIN GPIO_NUM_19
#define SCL_PIN GPIO_NUM_22

#define I2C_MASTER_NUM 0
static const int RX_BUF_SIZE = 1024;
#define TAG_BME280 "BME280"

#define TXD_PIN (gpio_num_t)23
#define RXD_PIN (gpio_num_t)18

static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();

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

const uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_APB,
};
struct MC6470_Dev_t *compass;
MC6470_MagReading magData;
MC6470_AccelReading accData;

static SemaphoreHandle_t rx_sem;

CANDataHandler *velopera_handler = new CANVelopera();

/* --------------------------- Tasks and Functions -------------------------- */
int count = 0;

// BME280 I2C write function
s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    s32 err = BME280_INIT_VALUE;

    esp_err_t esp_ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);

    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, reg_data, cnt, true);
    i2c_master_stop(cmd);

    esp_ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    if (esp_ret == ESP_OK)
    {
        err = SUCCESS;
    }
    else
    {
        err = ERROR;
    }
    i2c_cmd_link_delete(cmd);

    return (s8)err;
}

// BME280 I2C read function
s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    s32 err = BME280_INIT_VALUE;
    esp_err_t esp_ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);

    if (cnt > 1)
    {
        i2c_master_read(cmd, reg_data, cnt - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, reg_data + cnt - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    if (esp_ret == ESP_OK)
    {
        err = SUCCESS;
    }
    else
    {
        err = ERROR;
    }

    i2c_cmd_link_delete(cmd);

    return (s8)err;
}

// BME280 I2C delay function
void BME280_delay_msek(u32 msek)
{
    vTaskDelay(msek / portTICK_PERIOD_MS);
}

int send_data_to_nrf(char *data)
{
    const int len = strlen(data);
    char rxdata[] = "";
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    uart_read_bytes(UART_NUM_1, rxdata, len, 20 / portTICK_PERIOD_MS);
    ESP_LOGI("send_data_to_nrf", "Wrote %d bytes of msg=> %s", txBytes, data);
    return txBytes;
}

int updateSpeed(void *can_state, const float newSpeed, char valid)
{
    char msg[128];
    count++;
    if (count > 300)
    {
        MC6470_getData(compass, &magData, &accData);
        // ESP_LOGI("COMPASS", "(x, y, z) (%g, %g, %g)", accData.x, accData.y, accData.z);
        // "{\"speed\":%s,\"gear\":%d,\"aku_voltage\":%s,\"temperature\":%s,\"humidity\":%d,\"compass\":[%d,%d,%d],\"msg_counter\":%d}"
        sprintf(msg, "{\"speed\":%d,\"mag\":[%d,%d,%d]}\n", (int)newSpeed, (int)magData.x, (int)magData.y, (int)magData.z);
        send_data_to_nrf(msg);
        count = 0;
    }

    return 0;
}

static void twai_receive_task(void *arg)
{
    xSemaphoreTake(rx_sem, portMAX_DELAY);

    // dbcc_time_stamp_t time_stamp = 0;
    velopera_handler->init();
    velopera_handler->funcs.callbacks.updateSpeed = updateSpeed;
    while (1)
    {
        twai_message_t rx_msg;

        twai_receive(&rx_msg, 5000 / portTICK_PERIOD_MS);

        int ext = rx_msg.extd;
        int rtr = rx_msg.rtr;

        // printf("can frame: can0 %" PRIx32 "#", rx_msg.identifier);
        for (size_t i = 0; i < rx_msg.data_length_code; i++)
        {
            // printf("%02x", rx_msg.data[i]);
        }
        // printf("\r\n");

        // printf("uint64_t raw_data %llu \n\r", (*(uint64_t *)rx_msg.data));

        velopera_handler->handleCANData(rx_msg.identifier, (*(uint64_t *)rx_msg.data));
    }

    xSemaphoreGive(rx_sem);
    vTaskDelete(NULL);
}

extern "C" void app_main(void)
{
    rx_sem = xSemaphoreCreateBinary();
    gatts_init();
    xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, RX_TASK_PRIO, NULL, tskNO_AFFINITY);
    compass = (struct MC6470_Dev_t *)malloc(sizeof(struct MC6470_Dev_t));

    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // send_data_to_nrf("FF\n");
    MC6470_Init(compass, MC6470_ACCEL_ADDRESS_VDD);
    esp_err_t ret = MC6470_begin(compass);

    struct bme280_t bme280 = {

        .dev_addr = BME280_I2C_ADDRESS2,
        .bus_write = BME280_I2C_bus_write,
        .bus_read = BME280_I2C_bus_read,
        .delay_msec = BME280_delay_msek};

    s32 com_rslt;
    s32 v_uncomp_pressure_s32;
    s32 v_uncomp_temperature_s32;
    s32 v_uncomp_humidity_s32;

    // Initialize BME280 sensor and set internal parameters
    com_rslt = bme280_init(&bme280);
    printf("com_rslt %d\n", com_rslt);

    com_rslt += bme280_set_oversamp_pressure(BME280_OVERSAMP_16X);
    com_rslt += bme280_set_oversamp_temperature(BME280_OVERSAMP_2X);
    com_rslt += bme280_set_oversamp_humidity(BME280_OVERSAMP_1X);

    com_rslt += bme280_set_standby_durn(BME280_STANDBY_TIME_1_MS);
    com_rslt += bme280_set_filter(BME280_FILTER_COEFF_16);
    com_rslt += bme280_set_power_mode(BME280_NORMAL_MODE);
    int newSpeed = 0;
    while (1)
    {
        char msg[128];
        MC6470_getData(compass, &magData, &accData);
        ESP_LOGI("COMPASS", "acc(x, y, z) (%g, %g, %g), mag(x, y, z) (%g, %g, %g)", accData.x, accData.y, accData.z, magData.x, magData.y, magData.z);
        if (ret != MC6470_Status_OK)
        {
            ESP_LOGE("main", "MC6470_begin ERR %d", ret);
        }

        com_rslt = bme280_read_uncomp_pressure_temperature_humidity(
            &v_uncomp_pressure_s32, &v_uncomp_temperature_s32, &v_uncomp_humidity_s32);

        double temp = bme280_compensate_temperature_double(v_uncomp_temperature_s32);
        char temperature[12];
        sprintf(temperature, "%.2f degC", temp);

        double press = bme280_compensate_pressure_double(v_uncomp_pressure_s32) / 100; // Pa -> hPa
        char pressure[10];
        sprintf(pressure, "%.2f hPa", press);

        double hum = bme280_compensate_humidity_double(v_uncomp_humidity_s32);
        char humidity[10];
        sprintf(humidity, "%.2f %%", hum);

        // Print BME data
        if (com_rslt == SUCCESS)
        {
            printf("Temperature %s\n", temperature);
            printf("Humidity %s\n", humidity);
            printf("Pressure %s\n", pressure);

            sprintf(msg, "{\"speed\":%d,\"gear\":%d,\"aku_voltage\":%d,\"temperature\":%.2f,\"humidity\":%.2f,\"compass\":[%d,%d,%d]}\n",
                    (int)newSpeed, 0, 0, temp, hum, (int)magData.x, (int)magData.y, (int)magData.z);
            printf("printed %s \r\n", msg);
            send_data_to_nrf(msg);
        }
        else
        {
            ESP_LOGE(TAG_BME280, "measure error. code: %d", com_rslt);
        }
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
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

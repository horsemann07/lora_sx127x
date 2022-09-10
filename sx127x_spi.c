

#include <driver/spi_master.h>
#include <driver/spi_slave.h>
#include <driver/gpio.h>

#include "sx127x.h"

#define TAG __FILENAME__

#define HIGH 1
#define LOW 0
#define SX127x_GPIO_ISR_QUEUE 5
#define ESP_INTR_FLAG_DEFAULT 0

#define LORA_SPI_HOST VSPI_HOST
#define LORA_SPI_CLK_FREQ FREQ_MZ(9)

/**
 * @brief
 *
 */
QueueHandle_t g_gpio_evnt_q = NULL;

static spi_device_handle_t gs_spi_device;

static gpio_num_t gs_spi_cs_pin;

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(g_gpio_evnt_q, &(gpio_num), NULL);
}

esp_err_t spi_write_reg(int reg, int val)
{
    uint8_t out[2] = {0x80 | reg, val};
    uint8_t in[2];

    spi_transaction_t t = {
        .flags = 0,
        .length = 8 * sizeof(out),
        .tx_buffer = out,
        .rx_buffer = in};

    ESP_ERROR_CHECK(gpio_set_level(gs_spi_cs_pin, LOW));
    ESP_ERROR_CHECK(spi_device_transmit(gs_spi_device, &(t)));
    ESP_ERROR_CHECK(gpio_set_level(gs_spi_cs_pin, HIGH));

    return ESP_OK;
}

/**
 * Read the current value of a register.
 * @param reg Register index.
 * @return Value of the register.
 */
uint8_t spi_read_reg(int reg)
{
    uint8_t out[2] = {reg, 0xFF};
    uint8_t in[2];
    spi_transaction_t t = {
        .flags = 0,
        .length = 8 * sizeof(out),
        .tx_buffer = out,
        .rx_buffer = in};

    ESP_ERROR_CHECK(gpio_set_level(gs_spi_cs_pin, LOW));
    ESP_ERROR_CHECK(spi_device_transmit(gs_spi_device, &(t)));
    ESP_ERROR_CHECK(gpio_set_level(gs_spi_cs_pin, HIGH));
    return in[1];
}

esp_err_t gpio_toggle_pin(uint32_t pin)
{
    gpio_set_level(pin, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(pin, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    return ESP_OK;
}

esp_err_t lora_spi_init(lora_config_t *lora_setting)
{
    esp_err_t ret = ESP_OK;

    /* lora reset pin */
    gpio_pad_select_gpio(lora_setting->reset_pin);
    gpio_set_direction(lora_setting->reset_pin, GPIO_MODE_OUTPUT);

    gs_spi_cs_pin = lora_setting->spi_config.cs_pin;

    /* spi chip select pin */
    gpio_pad_select_gpio(lora_setting->spi_config.cs_pin);
    gpio_set_direction(lora_setting->spi_config.cs_pin, GPIO_MODE_OUTPUT);

    /* spi bus configuration */
    spi_bus_config_t spi_bus = {
        .miso_io_num = lora_setting->spi_config.miso_pin,
        .mosi_io_num = lora_setting->spi_config.mosi_pin,
        .sclk_io_num = lora_setting->spi_config.clk_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0};

    ret = spi_bus_initialize(LORA_SPI_HOST, &spi_bus, 0);
    ESP_ERROR_CHECK(ret);

    spi_device_interface_config_t dev = {
        .clock_speed_hz = LORA_SPI_CLK_FREQ,
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 1,
        .flags = 0,
        .pre_cb = NULL};
    ret = spi_bus_add_device(LORA_SPI_HOST, &dev, &(gs_spi_device));
    ESP_ERROR_CHECK(ret);

    gpio_config_t lora_irq_config = {
        .intr_type = GPIO_INTR_HIGH_LEVEL,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << lora_setting->di0_pin),
        .pull_down_en = GPIO_PULLUP_ENABLE,
        .pull_up_en = GPIO_PULLDOWN_ENABLE,
    };
    ret = gpio_config(&(lora_irq_config));
    ESP_ERROR_CHECK(ret);

    g_gpio_evnt_q = xQueueCreate(SX127x_GPIO_ISR_QUEUE, sizeof(uint32_t));
    ESP_NON_NULL_CHECK(g_gpio_evnt_q);

    ret = gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "gpio isr service installed");
    ESP_ERROR_CHECK(gpio_isr_handler_add(lora_setting->di0_pin, gpio_isr_handler, (void *)lora_setting->di0_pin));
    return ret;
}

esp_err_t spi_dev_deinit(void)
{
    ESP_ERROR_CHECK(spi_bus_free(LORA_SPI_HOST));
    ESP_ERROR_CHECK(spi_bus_remove_device(gs_spi_device));
    return ESP_OK;
}
#ifndef _SX1276_H_
#define _SX1276_H_

/* Kernal headers */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

/* esp_idf common headers */
#include <esp_err.h>
#include <esp_utils.h>

/* sx127x headers */
#include "sx127x_reg.h"
#include "sx127x_fsk.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define BITREAD(value, bit) (((value) >> (bit)) & 0x01)
#define BITSET(value, bit) ((value) |= (1UL << (bit)))
#define BITCLEAR(value, bit) ((value) &= ~(1UL << (bit)))
#define BITWRITE(value, bit, bitvalue) ((bitvalue) ? BITSET(value, bit) : BITCLEAR(value, bit))
#define FREQ_MZ(x) (uint64_t)(x * 1000000U)

    /**! dio intruppt task stack size */
    extern uint32_t dio_intruppt_task_stack;

    /**! dio intruppt task priority */
    extern uint8_t dio_intruppt_task_prior;

    /**
     * @brief
     *
     */
    typedef esp_err_t (*receive_cb_t)(size_t);

    /**
     * @brief
     *
     */
    typedef esp_err_t (*tx_done_t)(void);

    typedef enum
    {
        EU433 = FREQ_MZ(433),
        CN470 = FREQ_MZ(470),
        IN865 = FREQ_MZ(865),
        EU868 = FREQ_MZ(868),
        US915 = FREQ_MZ(915),
        AU915 = FREQ_MZ(915),
        KR920 = FREQ_MZ(920),
        AS923 = FREQ_MZ(923)
    } lora_freq_t;

    /**
     * @brief
     *
     */
    typedef enum
    {
        LORA_SX1276 = 1,
        LORA_SX1277 = 2,
        LORA_SX1278 = 3,
        LORA_SX1279 = 4
    } lora_sx127x_module_t;

    typedef enum
    {
        LORA_TX_MODE = 1,
        LORA_RX_MODE = 2
    } lora_mode_t;

    /**
     * @brief
     *
     */
    typedef struct
    {
        uint64_t clk_speed_hz; /*! spi clock speed freq. */
        uint32_t miso_pin;     /*! spi miso pin. */
        uint32_t mosi_pin;     /*! spi mosi pin. */
        uint32_t clk_pin;      /*! spi clk pin. */
        uint32_t cs_pin;       /*! spi cs pin */
    } lora_spi_config_t;

    /**
     * @brief
     *
     */
    typedef struct
    {
        lora_sx127x_module_t module;  /*! lora module specification */
        lora_freq_t freq_hz;          /*! lora operating frequency. */
        uint32_t di0_pin;             /*! lora intruppt (di0) pin. */
        uint32_t reset_pin;           /*! lora reset_pin pin. */
        receive_cb_t onRecvCb;        /*! callback function on receive. */
        tx_done_t onTxDoneCb;         /*! callback function after tx done. */
        bool IsImplicitModeEnable;    /*! implicit header mode enable. */
        lora_spi_config_t spi_config; /*! lora spi setting. */
    } lora_config_t;

    // define the pins used by the LoRa transceiver module
    // #define SCK 5
    // #define MISO 19
    // #define MOSI 27
    // #define SS 18
    // #define RST 14
    // #define DIO0 26

#define LORA_127x_CONFIG_DEFAULT()     \
    {                                  \
        .module = LORA_SX1276,         \
        .freq_hz = IN865,              \
        .reset_pin = 14,               \
        .di0_pin = 26,                 \
        .IsImplicitModeEnable = false, \
        .onRecvCb = NULL,              \
        .onTxDoneCb = NULL,            \
        .spi_config = {                \
            .mosi_pin = 27,            \
            .miso_pin = 19,            \
            .clk_pin = 5,              \
            .cs_pin = 18,              \
            .clk_speed_hz = 9000000,   \
        },                             \
    }

    /**
     * @brief
     *
     * @return esp_err_t
     */
    esp_err_t lora_sx127x_sleep(void);

    /**
     * @brief
     *
     * @return int32_t
     */
    int32_t lora_sx127x_get_spreading_factor(void);

    /**
     * @brief
     *
     * @return size_t
     */
    size_t lora_sx127x_get_signal_bandwidth(void);

    /**
     * @brief
     *
     * @return int32_t
     */
    int32_t lora_sx127x_get_packet_rssi(void);

    /**
     * @brief
     *
     * @return size_t
     */
    size_t lora_sx127x_get_packet_frequency_error(void);

    /**
     * @brief
     *
     * @return int32_t
     */
    int32_t lora_sx127x_get_rssi(void);

    /**
     * @brief
     *
     * @return float
     */
    float lora_sx127x_get_packet_snr(void);

    /**
     * @brief
     *
     * @return
     *      - ESP_OK:
     *      - ESP_FAIL:
     */
    esp_err_t lora_sx127x_reset(void);

    /**
     * @brief
     *
     * @param[out] packet_size : size of data available
     * @return
     *      - true: data availabe and 'packet_size' == packet size.
     *      - false: data not availabe and 'packet_size' == -1.
     */
    bool lora_sx127x_packet_available(void);

    /**
     * @brief send lora payload
     *
     * @param[in] data : pointer to data buffer.
     * @param[in] length : size of data.
     * @return
     *      - ESP_OK:
     *      - ESP_FAIL:
     *      - ESP_INVALID_PARAMETER:
     */
    esp_err_t lora_sx127x_send(uint8_t *data, uint32_t length);

    /**
     * @brief
     *
     * @param[out] data :
     * @param[out] length :
     * @return
     *      - ESP_OK:
     *      - ESP_FAIL:
     *      - ESP_INVALID_PARAMETER:
     */
    esp_err_t lora_sx127x_receive(uint8_t *data, size_t *length);

    /**
     * @brief
     *
     * @return
     *      - true: lora device is in tx mode.
     *      - false: lora device is in rx mode..
     */
    bool lora_sx127x_is_tx_mode(void);

    /**
     * @brief set the lora mode.
     *
     * @param mode : mode of lora LORA_TX_MODE or LORA_RX_MODE
     * @return
     *      - ESP_OK:
     *      - ESP_FAIL:
     */
    esp_err_t lora_sx127x_set_mode(lora_mode_t mode);

    /**
     * @brief
     *
     * @return
     *      - ESP_OK:
     *      - ESP_FAIL:
     */
    esp_err_t lora_sx127x_enable_invertIQ(void);

    /**
     * @brief
     *
     * @return
     *      - ESP_OK:
     *      - ESP_FAIL:
     */
    esp_err_t lora_sx127x_disable_invertIQ(void);

    /**
     * @brief
     *
     * @param level
     * @param boostPin
     * @return
     *      - ESP_OK:
     *      - ESP_FAIL:
     */
    esp_err_t lora_sx127x_set_tx_power(int level, bool boostEnable);

    /**
     * @brief
     *
     * @param freq
     * @return
     *      - ESP_OK:
     *      - ESP_FAIL:
     */
    esp_err_t lora_sx127x_set_freq(size_t freq);

    /**
     * @brief
     *
     * @param sf
     * @return
     *      - ESP_OK:
     *      - ESP_FAIL:
     */
    esp_err_t lora_sx127x_set_spreading_factor(int sf);

    /**
     * @brief
     *
     * @param bandwidth
     * @return
     *      - ESP_OK:
     *      - ESP_FAIL:
     */
    esp_err_t lora_sx127x_set_signal_bandwidth(size_t bandwidth);

    /**
     * @brief
     *
     * @param denominator
     * @return
     *      - ESP_OK:
     *      - ESP_FAIL:
     */
    esp_err_t lora_sx127x_coding_rate4(size_t denominator);

    /**
     * @brief
     *
     * @param length
     * @return
     *      - ESP_OK:
     *      - ESP_FAIL:
     */
    esp_err_t lora_sx127x_preamble_length(size_t length);

    /**
     * @brief
     *
     * @param sw
     * @return
     *      - ESP_OK:
     *      - ESP_FAIL:
     */
    esp_err_t lora_sx127x_set_sync_word(int sw);

    /**
     * @brief
     *
     * @return
     *      - ESP_OK:
     *      - ESP_FAIL:
     */
    esp_err_t lora_sx127x_enable_crc(void);

    /**
     * @brief
     *
     * @return
     *      - ESP_OK:
     *      - ESP_FAIL:
     */
    esp_err_t lora_sx127x_disable_crc(void);

    /**
     * @brief
     *
     * @return
     *      - ESP_OK:
     *      - ESP_FAIL:
     */
    esp_err_t lora_sx127x_dump_registor(void);

    // /**
    //  * @brief
    //  *
    //  * @param mA
    //  * @return
    //  *      - ESP_OK:
    //  *      - ESP_FAIL:
    //  */
    // esp_err_t lora_sx127x_set_ocp(uint8_t mA); // Over Current Protection control

    /**
     * @brief
     *
     * @param gain
     * @return
     *      - ESP_OK:
     *      - ESP_FAIL:
     */
    esp_err_t lora_sx127x_set_gain(uint8_t gain); // Set LNA gain

    /**
     * @brief Initilize the sx1276 lora module setting.
     *
     * @param[in] lora_setting : pointer to lora configuration.
     * @return
     *      - ESP_OK:
     *      - ESP_FAIL:
     */
    esp_err_t lora_sx127x_init(lora_config_t *lora_setting);

    /**
     * @brief Deinitilize the sx1276 lora modules setting.
     *
     * @param[in] lora_setting : pointer to lora configuration struct.
     * @return
     *      - ESP_OK:
     *      - ESP_FAIL:
     */
    esp_err_t lora_sx127x_deint(void);

    /**
     * @brief
     *
     * @param reg
     * @param val
     * @return esp_err_t
     */
    esp_err_t spi_write_reg(int reg, int val);

    /**
     * @brief
     *
     * @param reg
     * @return uint8_t
     */
    uint8_t spi_read_reg(int reg);

    /**
     * @brief
     *
     * @param pin
     * @return esp_err_t
     */
    esp_err_t gpio_toggle_pin(uint32_t pin);

    /**
     * @brief
     *
     * @param lora_setting
     * @return esp_err_t
     */
    esp_err_t lora_spi_init(lora_config_t *lora_setting);

    /**
     * @brief
     *
     * @return esp_err_t
     */
    esp_err_t lora_spi_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // _SX1276_H_
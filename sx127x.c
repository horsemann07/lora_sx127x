
#include <stdbool.h>
#include "sx127x.h"

#define TAG __FILENAME__

#define RF_MID_BAND_THRESHOLD 525E6
#define RSSI_OFFSET_HF_PORT 157
#define RSSI_OFFSET_LF_PORT 164

#define MAX_PAYLOAD_LENGTH 255
#define TIMEOUT_RESET 100
/**
 * @brief
 *
 */
TaskHandle_t gx_di0_intr_task = NULL;

/**! dio intruppt task stack size */
uint32_t di0_intruppt_task_stack = 1024 * 3;

/**! dio intruppt task priority */
uint8_t di0_intruppt_task_prior = 10;

/* */
extern QueueHandle_t g_gpio_evnt_q;

/**
 * @brief
 *
 */
static lora_config_t gs_lora_setting = {0};

/* ---------------------------------------------------------------------------- */
static esp_err_t lora_sx127x_idle(void)
{
    spi_write_reg(REG_LR_OPMODE, RFLR_OPMODE_LONGRANGEMODE_ON | RFLR_OPMODE_STANDBY);
    return ESP_OK;
}

/* ---------------------------------------------------------------------------- */
/**
 * @brief Configure explicit header mode. Packet size will be included in the frame.
 *
 * @return
 *      - ESP_OK
 */
static esp_err_t lora_sx127x_explicit_header_mode(void)
{
    gs_lora_setting.IsImplicitModeEnable = false;
    spi_write_reg(REG_LR_MODEMCONFIG1, spi_read_reg(REG_LR_MODEMCONFIG1) & RFLR_INVERTIQ_TX_MASK);
    return ESP_OK;
}

/* ---------------------------------------------------------------------------- */
/**
 * @brief Configure implicit header mode. All packets will have a predefined size.
 *
 * @return
 *      - ESP_OK
 */
static esp_err_t lora_sx127x_implicit_header_mode(void)
{
    gs_lora_setting.IsImplicitModeEnable = true;
    spi_write_reg(REG_LR_MODEMCONFIG1, spi_read_reg(REG_LR_MODEMCONFIG1) | RFLR_INVERTIQ_TX_OFF);
    return ESP_OK;
}

/* ---------------------------------------------------------------------------- */
/**
 * @brief
 *
 * @param[in] mA
 *
 * @return
 *      - ESP_OK
 */
static esp_err_t lora_sx127x_set_ocp(uint8_t mA)
{
    uint8_t ocpTrim = 27;

    if (mA <= 120)
    {
        ocpTrim = (mA - 45) / 5;
    }
    else if (mA <= 240)
    {
        ocpTrim = (mA + 30) / 10;
    }

    spi_write_reg(REG_OCP, RFLR_OCP_ON | (RFLR_LNA_GAIN_MASK & ocpTrim));
    return ESP_OK;
}

/*-----------------------------------------------------------*/
/**
 * @brief
 *
 * @param[in] paremeters
 */
static void handle_di0_intruppt_task(void *paremeters)
{
    /* Remove compiler warning */
    (void)paremeters;

    for (;;)
    {
        uint32_t gpio_num;
        if (xQueueReceive(g_gpio_evnt_q, &(gpio_num), portMAX_DELAY))
        {
            uint8_t irqFlags = spi_read_reg(REG_LR_IRQFLAGS);
            if ((irqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK) == 0)
            {
                if ((irqFlags & RFLR_IRQFLAGS_RXDONE_MASK) != 0)
                {
                    /* read packet length */
                    int packetLength = -1;
                    if (gs_lora_setting.IsImplicitModeEnable)
                    {
                        packetLength = spi_read_reg(REG_LR_PAYLOADLENGTH);
                    }
                    else
                    {
                        packetLength = spi_read_reg(REG_LR_RXNBBYTES);
                    }

                    /* set FIFO address to current RX address */
                    spi_write_reg(REG_LR_FIFOADDRPTR, spi_read_reg(REG_LR_FIFORXCURRENTADDR));

                    /* call on recevie callback function */
                    if (gs_lora_setting.onRecvCb != NULL)
                    {
                        gs_lora_setting.onRecvCb(packetLength);
                    }
                }
                else if ((irqFlags & RFLR_IRQFLAGS_TXDONE_MASK) != 0)
                {
                    /* call tx done callback function */
                    if (gs_lora_setting.onTxDoneCb != NULL)
                    {
                        gs_lora_setting.onTxDoneCb();
                    }
                }
            }
        }
    }
    /* program will never come here */
    gx_di0_intr_task = NULL;
    vTaskDelete(NULL);
}

/* ---------------------------------------------------------------------------- */
esp_err_t lora_sx127x_reset(void)
{
    gpio_toggle_pin(gs_lora_setting.spi_config.cs_pin);
    return ESP_OK;
}

/* ---------------------------------------------------------------------------- */
esp_err_t lora_sx127x_sleep(void)
{
    spi_write_reg(REG_LR_OPMODE, RFLR_OPMODE_LONGRANGEMODE_ON | RFLR_OPMODE_SLEEP);
    return ESP_OK;
}

/* ---------------------------------------------------------------------------- */
int32_t lora_sx127x_get_spreading_factor(void)
{
    return (int32_t)spi_read_reg(REG_LR_MODEMCONFIG2) >> 4;
}

/* ---------------------------------------------------------------------------- */
size_t lora_sx127x_get_signal_bandwidth(void)
{
    uint8_t bw = (spi_read_reg(REG_LR_MODEMCONFIG1) >> 4);

    switch (bw)
    {
    case 0:
        return 7.8E3;
    case 1:
        return 10.4E3;
    case 2:
        return 15.6E3;
    case 3:
        return 20.8E3;
    case 4:
        return 31.25E3;
    case 5:
        return 41.7E3;
    case 6:
        return 62.5E3;
    case 7:
        return 125E3;
    case 8:
        return 250E3;
    case 9:
        return 500E3;
    }

    return -1;
}

/* ---------------------------------------------------------------------------- */
int32_t lora_sx127x_get_packet_rssi(void)
{
    uint64_t freq = gs_lora_setting.freq_hz;
    return (spi_read_reg(REG_LR_PKTRSSIVALUE) - (freq < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

/* ---------------------------------------------------------------------------- */
float lora_sx127x_get_packet_snr(void)
{
    return ((int8_t)spi_read_reg(REG_LR_PKTSNRVALUE)) * 0.25;
}

/* ---------------------------------------------------------------------------- */
size_t lora_sx127x_get_packet_frequency_error(void)
{
    int32_t freqError = 0;
    freqError = (int32_t)(spi_read_reg(REG_LR_FEIMSB) & 0b111);
    freqError <<= 8L;
    freqError += (int32_t)(spi_read_reg(REG_LR_FEIMID));
    freqError <<= 8L;
    freqError += (int32_t)spi_read_reg(REG_LR_FEILSB);

    if (spi_read_reg(REG_LR_FEIMSB) & 0b1000)
    {                        // Sign bit is on
        freqError -= 524288; // 0b1000'0000'0000'0000'0000
    }

    // FXOSC: crystal oscillator (XTAL) frequency (2.5. Chip Specification, p. 14)
    const float fXtal = 32E6;
    const float fError = ((freqError * (1L << 24)) / fXtal) * (lora_sx127x_get_signal_bandwidth() / 500000.0f); // p. 37

    return (size_t)fError;
}

/* ---------------------------------------------------------------------------- */
int32_t lora_sx127x_get_rssi(void)
{
    size_t freq = gs_lora_setting.freq_hz;
    return (spi_read_reg(REG_LR_RSSIVALUE) - (freq < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

/* ---------------------------------------------------------------------------- */
bool lora_sx127x_is_transmitting(void)
{
    if ((spi_read_reg(REG_LR_OPMODE) & RFLR_OPMODE_TRANSMITTER) == RFLR_OPMODE_TRANSMITTER)
    {
        return true;
    }

    if (spi_read_reg(REG_LR_IRQFLAGS) & RFLR_IRQFLAGS_TXDONE_MASK)
    {
        // clear IRQ's
        spi_write_reg(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE_MASK);
    }

    return false;
}

/* ---------------------------------------------------------------------------- */
bool lora_sx127x_packet_available(void)
{
    if (spi_read_reg(REG_LR_IRQFLAGS) & RFLR_IRQFLAGS_RXDONE_MASK)
    {
        return true;
    }

    return false;
}

/* ---------------------------------------------------------------------------- */
uint8_t lora_sx127x_random(void)
{
    return spi_read_reg(REG_LR_RSSIWIDEBAND);
}

/* ---------------------------------------------------------------------------- */
esp_err_t lora_sx127x_set_freq(size_t freq)
{
    if ((gs_lora_setting.module == LORA_SX1276) || (gs_lora_setting.module == LORA_SX1277))
    {
        if ((freq < FREQ_MZ(137)) || (freq > FREQ_MZ(1020)))
        {
            return ESP_FAIL;
        }
    }
    else if (gs_lora_setting.module == LORA_SX1278)
    {
        if ((freq < FREQ_MZ(137)) || (freq > FREQ_MZ(525)))
        {
            return ESP_FAIL;
        }
    }
    else
    {
        /* module LORA_SX1279 */
        if ((freq < FREQ_MZ(137)) || (freq > FREQ_MZ(960)))
        {
            return ESP_FAIL;
        }
    }

    gs_lora_setting.freq_hz = freq;

    uint64_t frf = ((uint64_t)freq << 19) / 32000000;

    spi_write_reg(REG_LR_FRFMSB, (uint8_t)(frf >> 16));
    spi_write_reg(REG_LR_FRFMID, (uint8_t)(frf >> 8));
    spi_write_reg(REG_LR_FRFLSB, (uint8_t)(frf >> 0));
    return ESP_OK;
}

/* ---------------------------------------------------------------------------- */
esp_err_t lora_sx127x_set_gain(uint8_t gain)
{
    // check allowed range
    if (gain > 6)
    {
        gain = 6;
    }

    // set to standby
    lora_sx127x_idle();

    // set gain
    if (gain == 0)
    {
        // if gain = 0, enable AGC
        spi_write_reg(REG_LR_MODEMCONFIG3, RFLR_MODEMCONFIG3_AGCAUTO_ON);
    }
    else
    {
        // disable AGC
        spi_write_reg(REG_LR_MODEMCONFIG3, RFLR_MODEMCONFIG3_AGCAUTO_OFF);

        // clear Gain and set LNA boost
        spi_write_reg(REG_LR_LNA, RFLR_LNA_BOOST_HF_ON);

        // set gain
        spi_write_reg(REG_LR_LNA, spi_read_reg(REG_LR_LNA) | (gain << 5));
    }
    return ESP_OK;
}

/* ---------------------------------------------------------------------------- */
esp_err_t lora_sx127x_set_tx_power(int level, bool boostEnable)
{
    if (boostEnable == false)
    {
        /* RFO */
        level = ((level < 0) ? 0 : ((level > 14) ? 14 : level));

        // if (level < 0)
        // {
        //     level = 0;
        // }
        // else if (level > 14)
        // {
        //     level = 14;
        // }

        spi_write_reg(REG_LR_PACONFIG, REG_LR_PLL | level);
    }
    else
    {
        /* PA BOOST */
        if (level > 17)
        {
            if (level > 20)
            {
                level = 20;
            }

            /* subtract 3 from level, so 18 - 20 maps to 15 - 17 */
            level -= 3;

            /* High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.) */
            spi_write_reg(REG_LR_PADAC, 0x87);
            lora_sx127x_set_ocp(140);
        }
        else
        {
            if (level < 2)
            {
                level = 2;
            }

            /* Default value PA_HF/LF or +17dBm */
            spi_write_reg(REG_LR_PADAC, 0x84);
            lora_sx127x_set_ocp(100);
        }

        spi_write_reg(REG_LR_PACONFIG, RFLR_PACONFIG_PASELECT_PABOOST | (level - 2));
    }
    return ESP_OK;
}

/* ---------------------------------------------------------------------------- */
esp_err_t lora_sx127x_set_ldo_flag(void)
{
    // Section 4.1.1.5
    long symbolDuration = 1000 / (lora_sx127x_get_signal_bandwidth() / (1L << lora_sx127x_get_spreading_factor()));

    // Section 4.1.1.6
    bool ldoOn = symbolDuration > 16;

    uint8_t config3 = spi_read_reg(REG_LR_MODEMCONFIG3);
    BITWRITE(config3, 3, ldoOn);
    spi_write_reg(REG_LR_MODEMCONFIG3, config3);
    return ESP_OK;
}

/* ---------------------------------------------------------------------------- */
esp_err_t lora_sx127x_coding_rate4(size_t denominator)
{
    if (denominator < 5)
    {
        denominator = 5;
    }
    else if (denominator > 8)
    {
        denominator = 8;
    }

    int cr = denominator - 4;

    spi_write_reg(REG_LR_MODEMCONFIG1, (spi_read_reg(REG_LR_MODEMCONFIG1) & RFLR_MODEMCONFIG1_CODINGRATE_MASK) | (cr << 1));
    return ESP_OK;
}

/* ---------------------------------------------------------------------------- */
esp_err_t lora_sx127x_preamble_length(size_t length)
{
    spi_write_reg(REG_LR_PREAMBLEMSB, (uint8_t)(length >> 8));
    spi_write_reg(REG_LR_PREAMBLELSB, (uint8_t)(length >> 0));
    return ESP_OK;
}

/* ---------------------------------------------------------------------------- */
esp_err_t lora_sx127x_set_sync_word(int sw)
{
    spi_write_reg(REG_LR_SYNCWORD, sw);
    return ESP_OK;
}

/* ---------------------------------------------------------------------------- */
esp_err_t lora_sx127x_enable_crc()
{
    spi_write_reg(REG_LR_MODEMCONFIG2, spi_read_reg(REG_LR_MODEMCONFIG2) | RFLR_MODEMCONFIG2_RXPAYLOADCRC_ON);
    return ESP_OK;
}

/* ---------------------------------------------------------------------------- */
esp_err_t lora_sx127x_disable_crc(void)
{
    spi_write_reg(REG_LR_MODEMCONFIG2, spi_read_reg(REG_LR_MODEMCONFIG2) & RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK);
    return ESP_OK;
}

/* ---------------------------------------------------------------------------- */
esp_err_t lora_sx127x_enable_invertIQ(void)
{
    spi_write_reg(REG_LR_INVERTIQ, RF_FDEVLSB_100000_HZ);
    spi_write_reg(REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON);
    return ESP_OK;
}

/* ---------------------------------------------------------------------------- */
esp_err_t lora_sx127x_disable_invertIQ(void)
{
    spi_write_reg(REG_LR_INVERTIQ, REG_SYNCCONFIG);
    spi_write_reg(REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF);
    return ESP_OK;
}

/* ---------------------------------------------------------------------------- */
esp_err_t lora_sx127x_dump_registers(void)
{
    for (int i = 0; i < 128; i++)
    {
        printf("%02X ", spi_read_reg(i));
    }
    printf("\n");
    return ESP_OK;
}

/* ---------------------------------------------------------------------------- */
esp_err_t lora_sx127x_receive(uint8_t *data_buf, size_t *length)
{
    ESP_PARAM_CHECK(data_buf);

    int packetLength = 0;
    int irqFlags = spi_read_reg(REG_LR_IRQFLAGS);

    if ((*length) > 0)
    {
        lora_sx127x_implicit_header_mode();

        spi_write_reg(REG_LR_PAYLOADLENGTH, (*length) & 0xFF);
    }
    else
    {
        lora_sx127x_explicit_header_mode();
    }

    /* clear IRQ's */
    spi_write_reg(REG_LR_IRQFLAGS, irqFlags);

    if ((irqFlags & RFLR_IRQFLAGS_RXDONE_MASK) && (irqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK) == 0)
    {
        /* read packet length */
        if (gs_lora_setting.IsImplicitModeEnable)
        {
            packetLength = spi_read_reg(REG_LR_PAYLOADLENGTH);
        }
        else
        {
            packetLength = spi_read_reg(REG_LR_RXNBBYTES);
        }

        /* set FIFO address to current RX address */
        spi_write_reg(REG_LR_FIFOADDRPTR, spi_read_reg(REG_LR_FIFORXCURRENTADDR));

        /* put in standby mode */
        lora_sx127x_idle();
    }
    else if (spi_read_reg(REG_LR_OPMODE) != (RFLR_OPMODE_LONGRANGEMODE_ON | RFLR_OPMODE_RECEIVER_SINGLE))
    {
        /* not currently in RX mode
         * reset FIFO address */
        spi_write_reg(REG_LR_FIFOADDRPTR, 0);

        /* put in single RX mode */
        spi_write_reg(REG_LR_OPMODE, RFLR_OPMODE_LONGRANGEMODE_ON | RFLR_OPMODE_RECEIVER_SINGLE);
    }

    if (packetLength > 0)
    {
        /* Transfer data from radio. */
        lora_sx127x_idle();
        spi_write_reg(REG_LR_FIFOADDRPTR, spi_read_reg(REG_LR_FIFORXCURRENTADDR));

        if (packetLength > (*length))
        {
            (*length) = packetLength;
            data_buf = ESP_REALLOC(data_buf, packetLength);
        }
        for (int i = 0; i < (*length); i++)
        {
            data_buf[i] = spi_read_reg(REG_LR_FIFO);
        }
        return ESP_OK;
    }

    return ESP_FAIL;
}

/* ---------------------------------------------------------------------------- */
esp_err_t lora_sx127x_send(uint8_t *data, uint32_t length)
{
    ESP_PARAM_CHECK(data);

    if (lora_sx127x_is_transmitting())
    {
        ESP_LOGE(TAG, "Lora is transmitting... Send after TX done.");
        return ESP_FAIL;
    }

    /* put in standby mode */
    lora_sx127x_idle();

    if (gs_lora_setting.IsImplicitModeEnable)
    {
        lora_sx127x_implicit_header_mode();
    }
    else
    {
        lora_sx127x_explicit_header_mode();
    }

    /* reset FIFO address and payload length */
    spi_write_reg(REG_LR_FIFOADDRPTR, 0);
    spi_write_reg(REG_LR_PAYLOADLENGTH, 0);

    /* get max packet */
    int currentLength = spi_read_reg(REG_LR_PAYLOADLENGTH);

    /* check size */
    if ((currentLength + length) > MAX_PAYLOAD_LENGTH)
    {
        length = MAX_PAYLOAD_LENGTH - currentLength;
        ESP_LOGW(TAG, "setting data packet length %u", length);
    }

    /* write data */
    for (int i = 0; i < length; i++)
    {
        spi_write_reg(REG_LR_FIFO, data[i]);
    }

    /* update length */
    spi_write_reg(REG_LR_PAYLOADLENGTH, currentLength + length);

    /* Start transmission and wait for conclusion. */
    spi_write_reg(REG_LR_OPMODE, RFLR_OPMODE_LONGRANGEMODE_ON | RFLR_OPMODE_TRANSMITTER);

    /* If callback function set then call intruppt on tx done
     * else make sure tx is done. */
    if (gs_lora_setting.onTxDoneCb)
    {
        spi_write_reg(REG_LR_DIOMAPPING1, REG_LR_DIOMAPPING1); // DIO0 => TXDONE
    }
    else
    {
        while ((spi_read_reg(REG_LR_IRQFLAGS) & RFLR_IRQFLAGS_TXDONE_MASK) == 0)
        {
            vTaskDelay(pdMS_TO_TICKS(2));
        }

        spi_write_reg(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE_MASK);
    }

    return ESP_OK;
}

/* ---------------------------------------------------------------------------- */
esp_err_t lora_sx127x_init(lora_config_t *lora_setting)
{
    ESP_PARAM_CHECK(lora_setting);
    esp_err_t ret = ESP_OK;

    /* stored spi configurations */
    gs_lora_setting.spi_config.mosi_pin = lora_setting->spi_config.mosi_pin;
    gs_lora_setting.spi_config.miso_pin = lora_setting->spi_config.miso_pin;
    gs_lora_setting.spi_config.clk_pin = lora_setting->spi_config.clk_pin;
    gs_lora_setting.spi_config.cs_pin = lora_setting->spi_config.cs_pin;

    /* stored lora configurations */
    if ((lora_setting->module != LORA_SX1276) &&
        (lora_setting->module != LORA_SX1277) &&
        (lora_setting->module != LORA_SX1278) &&
        (lora_setting->module != LORA_SX1279))
    {
        ESP_LOGE(TAG, "Unsupported LoRa module.");
        return ESP_FAIL;
    }

    gs_lora_setting.module = lora_setting->module;
    gs_lora_setting.freq_hz = lora_setting->freq_hz;
    gs_lora_setting.reset_pin = lora_setting->reset_pin;
    gs_lora_setting.di0_pin = lora_setting->di0_pin;
    gs_lora_setting.IsImplicitModeEnable = lora_setting->IsImplicitModeEnable;
    gs_lora_setting.onRecvCb = lora_setting->onRecvCb;
    gs_lora_setting.onTxDoneCb = lora_setting->onTxDoneCb;

    ret = lora_spi_init(&(gs_lora_setting));
    ESP_ERROR_RETURN(ret != ESP_OK, ret, "failed to init lora spi");

    assert(xTaskCreate(handle_di0_intruppt_task,
                       "dio_intruppt",
                       di0_intruppt_task_stack,
                       NULL,
                       di0_intruppt_task_prior,
                       &(gx_di0_intr_task)));

    /* reset device */
    lora_sx127x_reset();

    /* check version */
    uint8_t version;
    uint8_t i = 0;
    while (i++ < TIMEOUT_RESET)
    {
        version = spi_read_reg(REG_VERSION);
        if (version == 0x12)
        {
            break;
        }
        vTaskDelay(2);
    }
    assert(i <= TIMEOUT_RESET + 1); // at the end of the loop above, the max value i can reach is TIMEOUT_RESET + 1
    if (version)
    {
        ESP_LOGE(TAG, "version 0x%x", version);
    }

    /* put in sleep mode */
    lora_sx127x_sleep();

    /* set frequency */
    lora_sx127x_set_freq(gs_lora_setting.freq_hz);

    /* set base addresses */
    spi_write_reg(REG_LR_FIFOTXBASEADDR, 0);
    spi_write_reg(REG_LR_FIFORXBASEADDR, 0);

    /* set LNA boost */
    spi_write_reg(REG_LNA, spi_read_reg(REG_LNA) | 0x03);

    /* set auto AGC */
    spi_write_reg(REG_LR_MODEMCONFIG3, 0x04);

    /* set output power to 17 dBm */
    lora_sx127x_set_tx_power(17, false);

    /* put in standby mode */
    lora_sx127x_idle();

    return ESP_OK;
}

/* ---------------------------------------------------------------------------- */
esp_err_t lora_sx127x_deint(void)
{
    lora_spi_deinit();
    return ESP_OK;
}

/* ---------------------------------------------------------------------------- */

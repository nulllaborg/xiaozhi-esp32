#pragma once

#include <driver/i2s_pdm.h>
#include <driver/i2s_std.h>
#include <esp_log.h>

#include <cmath>

#include "audio/audio_codec.h"

class AudioInputDevice : virtual public AudioCodec {
public:
    AudioInputDevice(int sample_rate, gpio_num_t clk, gpio_num_t din) {
        input_sample_rate_ = sample_rate;

        // Create a new channel for speaker
        i2s_chan_config_t chan_cfg = {
            .id = (i2s_port_t)0,
            .role = I2S_ROLE_MASTER,
            .dma_desc_num = 6,
            .dma_frame_num = 240,
            .auto_clear_after_cb = true,
            .auto_clear_before_cb = false,
            .intr_priority = 0,
        };
        ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, nullptr, &rx_handle_));

        i2s_pdm_rx_config_t std_cfg = {
            .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG((uint32_t)sample_rate),
            .slot_cfg =
                I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
            .gpio_cfg = {
                .clk = clk,
                .din = din,
                .invert_flags =
                    {
                        .clk_inv = false,
                    },
            }};
        ESP_ERROR_CHECK(i2s_channel_init_pdm_rx_mode(rx_handle_, &std_cfg));
    }

    virtual ~AudioInputDevice() {
        if (rx_handle_ != nullptr) {
            ESP_ERROR_CHECK(i2s_channel_disable(rx_handle_));
        }
    }

    int Read(int16_t* dest, int samples) override {
        size_t bytes_read;

        std::vector<int32_t> bit32_buffer(samples);
        if (i2s_channel_read(rx_handle_, bit32_buffer.data(), samples * sizeof(int32_t),
                             &bytes_read, portMAX_DELAY) != ESP_OK) {
            return 0;
        }

        samples = bytes_read / sizeof(int32_t);
        for (int i = 0; i < samples; i++) {
            // #if defined(CONFIG_MIC_TYPE_INMP441)
            int32_t value = bit32_buffer[i] >> 12;
            // #elif defined(CONFIG_MIC_TYPE_SPH0645)
            //             int32_t value = bit32_buffer[i] >> 14;
            // #endif
            dest[i] = (value > INT16_MAX)    ? INT16_MAX
                      : (value < -INT16_MAX) ? -INT16_MAX
                                             : (int16_t)value;
        }
        return samples;
    }
};
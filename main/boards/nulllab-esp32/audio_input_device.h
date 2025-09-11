#pragma once

#include <driver/i2s_std.h>
#include <esp_log.h>

#include <cmath>

#include "audio/audio_codec.h"

class AudioInputDevice : virtual public AudioCodec {
 public:
  AudioInputDevice(int sample_rate, gpio_num_t bclk, gpio_num_t ws,
                   gpio_num_t din) {
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

    i2s_std_config_t std_cfg = {
        .clk_cfg =
            {
                .sample_rate_hz = (uint32_t)sample_rate,
                .clk_src = I2S_CLK_SRC_DEFAULT,
                .mclk_multiple = I2S_MCLK_MULTIPLE_256,
#ifdef I2S_HW_VERSION_2
                .ext_clk_freq_hz = 0,
#endif
            },
        .slot_cfg = {.data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
                     .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
                     .slot_mode = I2S_SLOT_MODE_MONO,
                     .slot_mask = I2S_STD_SLOT_LEFT,
                     .ws_width = I2S_DATA_BIT_WIDTH_32BIT,
                     .ws_pol = false,
                     .bit_shift = true,
#ifdef I2S_HW_VERSION_2
                     .left_align = true,
                     .big_endian = false,
                     .bit_order_lsb = false
#endif
        },
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = bclk,
            .ws = ws,
            .dout = I2S_GPIO_UNUSED,
            .din = din,
            .invert_flags = {
                .mclk_inv = false, .bclk_inv = false, .ws_inv = false}}};
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle_, &std_cfg));
  }

  virtual ~AudioInputDevice() {
    if (rx_handle_ != nullptr) {
      ESP_ERROR_CHECK(i2s_channel_disable(rx_handle_));
    }
  }

  int Read(int16_t* dest, int samples) override {
    size_t bytes_read;

    std::vector<int32_t> bit32_buffer(samples);
    if (i2s_channel_read(rx_handle_, bit32_buffer.data(),
                         samples * sizeof(int32_t), &bytes_read,
                         portMAX_DELAY) != ESP_OK) {
      return 0;
    }

    samples = bytes_read / sizeof(int32_t);
    for (int i = 0; i < samples; i++) {
#if defined(CONFIG_MIC_TYPE_INMP441)
      int32_t value = bit32_buffer[i] >> 12;
#elif defined(CONFIG_MIC_TYPE_SPH0645)
      int32_t value = bit32_buffer[i] >> 14;
#endif
      dest[i] = (value > INT16_MAX)    ? INT16_MAX
                : (value < -INT16_MAX) ? -INT16_MAX
                                       : (int16_t)value;
    }
    return samples;
  }
};
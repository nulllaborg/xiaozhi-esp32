#pragma once

#include <driver/i2s_std.h>
#include <esp_log.h>

#include <cmath>

#include "audio/audio_codec.h"

class AudioOutputDevice : virtual public AudioCodec {
 public:
  AudioOutputDevice(int sample_rate, gpio_num_t bclk, gpio_num_t ws,
                    gpio_num_t dout) {
    output_sample_rate_ = sample_rate;
    // Create a new channel for speaker
    i2s_chan_config_t chan_cfg = {
        .id = (i2s_port_t)1,
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = 6,
        .dma_frame_num = 240,
        .auto_clear_after_cb = true,
        .auto_clear_before_cb = false,
        .intr_priority = 0,
    };
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle_, nullptr));

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
                     .slot_mask = I2S_STD_SLOT_BOTH,
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
            .dout = dout,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false, .bclk_inv = false, .ws_inv = false}}};

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle_, &std_cfg));
  }

  ~AudioOutputDevice() {
    if (tx_handle_ != nullptr) {
      ESP_ERROR_CHECK(i2s_channel_disable(tx_handle_));
    }
  }

  int Write(const int16_t* data, int samples) override {
    std::vector<int32_t> buffer(samples);

    // output_volume_: 0-100
    // volume_factor_: 0-65536
    int32_t volume_factor = pow(double(output_volume_) / 100.0, 2) * 65536;
    for (int i = 0; i < samples; i++) {
      int64_t temp =
          int64_t(data[i]) * volume_factor;  // 使用 int64_t 进行乘法运算
      if (temp > INT32_MAX) {
        buffer[i] = INT32_MAX;
      } else if (temp < INT32_MIN) {
        buffer[i] = INT32_MIN;
      } else {
        buffer[i] = static_cast<int32_t>(temp);
      }
    }

    size_t bytes_written;
    ESP_ERROR_CHECK(i2s_channel_write(tx_handle_, buffer.data(),
                                      samples * sizeof(int32_t), &bytes_written,
                                      portMAX_DELAY));
    return bytes_written / sizeof(int32_t);
  }
};
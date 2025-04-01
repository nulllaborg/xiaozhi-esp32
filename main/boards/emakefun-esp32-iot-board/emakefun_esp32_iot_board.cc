#include <driver/i2c_master.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_log.h>
#include <wifi_station.h>

#include "application.h"
#include "audio_codecs/no_audio_codec.h"
#include "button.h"
#include "config.h"
#include "display/oled_display.h"
#include "iot/thing_manager.h"
#include "led/single_led.h"
#include "system_reset.h"
#include "wifi_board.h"

#define TAG "EmakefunEsp32IotBoard"

LV_FONT_DECLARE(font_puhui_14_1);
LV_FONT_DECLARE(font_awesome_14_1);

class EmakefunEsp32IotBoard : public WifiBoard {
 private:
  Button boot_button_;

  i2c_master_bus_handle_t display_i2c_bus_;
  esp_lcd_panel_io_handle_t panel_io_ = nullptr;
  esp_lcd_panel_handle_t panel_ = nullptr;
  Display* display_ = nullptr;

  void InitializeDisplayI2c() {
    i2c_master_bus_config_t bus_config = {
        .i2c_port = (i2c_port_t)0,
        .sda_io_num = DISPLAY_SDA_PIN,
        .scl_io_num = DISPLAY_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags =
            {
                .enable_internal_pullup = 1,
            },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &display_i2c_bus_));
  }

  void InitializeSsd1306Display() {
    // SSD1306 config
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = 0x3C,
        .on_color_trans_done = nullptr,
        .user_ctx = nullptr,
        .control_phase_bytes = 1,
        .dc_bit_offset = 6,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .flags =
            {
                .dc_low_on_data = 0,
                .disable_control_phase = 0,
            },
        .scl_speed_hz = 400 * 1000,
    };

    ESP_ERROR_CHECK(
        esp_lcd_new_panel_io_i2c_v2(display_i2c_bus_, &io_config, &panel_io_));

    ESP_LOGI(TAG, "Install SSD1306 driver");
    esp_lcd_panel_dev_config_t panel_config = {};
    panel_config.reset_gpio_num = -1;
    panel_config.bits_per_pixel = 1;

    esp_lcd_panel_ssd1306_config_t ssd1306_config = {
        .height = static_cast<uint8_t>(DISPLAY_HEIGHT),
    };
    panel_config.vendor_config = &ssd1306_config;

    ESP_ERROR_CHECK(
        esp_lcd_new_panel_ssd1306(panel_io_, &panel_config, &panel_));
    ESP_LOGI(TAG, "SSD1306 driver installed");

    // Reset the display
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_));
    if (esp_lcd_panel_init(panel_) != ESP_OK) {
      ESP_LOGE(TAG, "Failed to initialize display");
      display_ = new NoDisplay();
      return;
    }

    // Set the display to on
    ESP_LOGI(TAG, "Turning display on");
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_, true));

    display_ = new OledDisplay(panel_io_, panel_, DISPLAY_WIDTH, DISPLAY_HEIGHT,
                               DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y,
                               {&font_puhui_14_1, &font_awesome_14_1});
  }

  void InitializeButtons() {
    // 配置 GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << BUILTIN_LED_GPIO,  // 设置需要配置的 GPIO 引脚
        .mode = GPIO_MODE_OUTPUT,                  // 设置为输出模式
        .pull_up_en = GPIO_PULLUP_DISABLE,         // 禁用上拉
        .pull_down_en = GPIO_PULLDOWN_DISABLE,     // 禁用下拉
        .intr_type = GPIO_INTR_DISABLE             // 禁用中断
    };
    gpio_config(&io_conf);  // 应用配置

    boot_button_.OnClick([this]() {
      auto& app = Application::GetInstance();
      if (app.GetDeviceState() == kDeviceStateStarting &&
          !WifiStation::GetInstance().IsConnected()) {
        ResetWifiConfiguration();
      }
      gpio_set_level(BUILTIN_LED_GPIO, 1);
      app.ToggleChatState();
    });
  }

  // 物联网初始化，添加对 AI 可见设备
  void InitializeIot() {
    auto& thing_manager = iot::ThingManager::GetInstance();
    thing_manager.AddThing(iot::CreateThing("Speaker"));
    thing_manager.AddThing(iot::CreateThing("Lamp"));
  }

 public:
  EmakefunEsp32IotBoard() : boot_button_(BOOT_BUTTON_GPIO) {
    InitializeDisplayI2c();
    InitializeSsd1306Display();
    InitializeButtons();
    InitializeIot();
  }

  virtual AudioCodec* GetAudioCodec() override {
    static NoAudioCodecSimplex audio_codec(
        AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
        AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK,
        AUDIO_I2S_SPK_GPIO_DOUT, I2S_STD_SLOT_BOTH, AUDIO_I2S_MIC_GPIO_SCK,
        AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN, I2S_STD_SLOT_LEFT);
    return &audio_codec;
  }

  virtual Display* GetDisplay() override { return display_; }
};

DECLARE_BOARD(EmakefunEsp32IotBoard);

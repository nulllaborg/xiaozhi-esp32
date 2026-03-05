#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_log.h>
#include <wifi_station.h>

#include "application.h"
#include "audio/codecs/no_audio_codec.h"
#include "audio_input_device.h"
#include "audio_output_device.h"
#include "button.h"
#include "config.h"
#include "display/lcd_display.h"
#include "led/single_led.h"
#include "system_reset.h"
#include "wifi_board.h"

#define TAG "NulllabEsp32Lcd"

LV_FONT_DECLARE(font_puhui_basic_20_4);
LV_FONT_DECLARE(font_awesome_20_4);
const lv_font_t* font_noto_emoji_128_init(void);
class NulllabEsp32Lcd : public WifiBoard {
private:
    Button boot_button_;
    LcdDisplay* display_;

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_CLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeLcdDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片
        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));

        esp_lcd_panel_reset(panel);

        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        display_ = new SpiLcdDisplay(panel_io, panel, DISPLAY_WIDTH, DISPLAY_HEIGHT,
                                     DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X,
                                     DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

    void InitializeButtons() {
        // // 配置 GPIO
        // gpio_config_t io_conf = {
        //     .pin_bit_mask = 1ULL << BUILTIN_LED_GPIO,  // 设置需要配置的 GPIO 引脚
        //     .mode = GPIO_MODE_OUTPUT,                  // 设置为输出模式
        //     .pull_up_en = GPIO_PULLUP_DISABLE,         // 禁用上拉
        //     .pull_down_en = GPIO_PULLDOWN_DISABLE,     // 禁用下拉
        //     .intr_type = GPIO_INTR_DISABLE             // 禁用中断
        // };
        // gpio_config(&io_conf);  // 应用配置

        boot_button_.OnClick([this]() {
            lv_mem_monitor_t mon;
            lv_mem_monitor(&mon);
            ESP_LOGI("MEM2", "used: %d, free: %d, frag: %d%%, max_free: %d",
                     mon.total_size - mon.free_size, mon.free_size, mon.frag_pct,
                     mon.free_biggest_size);

            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
    }

public:
    NulllabEsp32Lcd() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializeSpi();
        InitializeLcdDisplay();
        InitializeButtons();

        lv_mem_monitor_t mon;
        lv_mem_monitor(&mon);
        ESP_LOGI("MEM", "used: %d, free: %d, frag: %d%%, max_free: %d",
                 mon.total_size - mon.free_size, mon.free_size, mon.frag_pct,
                 mon.free_biggest_size);
    }

    virtual AudioCodec* GetAudioCodec() override {
        class AudioDevice : public AudioInputDevice, public AudioOutputDevice {
        public:
            AudioDevice(int input_sample_rate, int output_sample_rate, gpio_num_t spk_bclk,
                        gpio_num_t spk_ws, gpio_num_t spk_dout, gpio_num_t mic_sck,
                        gpio_num_t mic_din)
                : AudioInputDevice(input_sample_rate, mic_sck, mic_din),
                  AudioOutputDevice(output_sample_rate, spk_bclk, spk_ws, spk_dout) {}
        };

        static AudioDevice audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
                                       AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK,
                                       AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_CLK,
                                       AUDIO_I2S_MIC_GPIO_DIN);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override { return display_; }
};

DECLARE_BOARD(NulllabEsp32Lcd);

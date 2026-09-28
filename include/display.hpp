#pragma once

#include <LovyanGFX.hpp>

class LGFX : public lgfx::LGFX_Device
{
    lgfx::Panel_ILI9341 _panel;
    lgfx::Bus_SPI _bus;
    lgfx::Light_PWM _light;
    lgfx::Touch_XPT2046 _touch;

public:
    LGFX()
    {
        // SPI bus configuration
        {
            auto cfg = _bus.config();

            cfg.spi_host = SPI2_HOST;
            cfg.spi_mode = 0;
            cfg.freq_write = 40000000;
            cfg.freq_read = 16000000;

            cfg.spi_3wire = false;
            cfg.use_lock = true;
            cfg.dma_channel = SPI_DMA_CH_AUTO;

            cfg.pin_sclk = 14;
            cfg.pin_mosi = 13;
            cfg.pin_miso = 12;
            cfg.pin_dc   = 2;

            _bus.config(cfg);
            _panel.setBus(&_bus);
        }

        // ILI9341 panel configuration
        {
            auto cfg = _panel.config();

            cfg.pin_cs = 15;
            cfg.pin_rst = -1;
            cfg.pin_busy = -1;

            cfg.panel_width = 240;
            cfg.panel_height = 320;

            cfg.offset_x = 0;
            cfg.offset_y = 0;

            cfg.offset_rotation = 0;

            _panel.config(cfg);
        }

        // Backlight configuration
        {
            auto cfg = _light.config();

            cfg.pin_bl = 21;
            cfg.invert = false;

            _light.config(cfg);
            _panel.setLight(&_light);
        }

        // Touch configuration
        {
            auto cfg = _touch.config();

            cfg.spi_host = -1;
            cfg.freq = 2500000;

            cfg.pin_sclk = 25;
            cfg.pin_mosi = 32;
            cfg.pin_miso = 39;
            cfg.pin_cs   = 33;
            cfg.pin_int  = 36;

            cfg.bus_shared = false;

            _touch.config(cfg);

            _panel.setTouch(&_touch);
        }

        setPanel(&_panel);
    }
};
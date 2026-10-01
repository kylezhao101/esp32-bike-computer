#include "bluetooth.hpp"
#include "gps.hpp"
#include "ui.hpp"
#include "wifi_files.hpp"

#include <cstdio>
#include <dirent.h>
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

static void check_sd_card()
{
    spi_bus_config_t bus{};
    bus.mosi_io_num = 23;
    bus.miso_io_num = 19;
    bus.sclk_io_num = 18;
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;
    bus.max_transfer_sz = 4096;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI3_HOST;

    esp_err_t err = spi_bus_initialize(
        static_cast<spi_host_device_t>(host.slot), &bus, SPI_DMA_CH_AUTO
    );    
    if (err != ESP_OK) {
        printf("SD SPI setup failed: %s\n", esp_err_to_name(err));
        return;
    }

    sdspi_device_config_t device = SDSPI_DEVICE_CONFIG_DEFAULT();
    device.gpio_cs = GPIO_NUM_5;
    device.host_id = static_cast<spi_host_device_t>(host.slot);

    esp_vfs_fat_sdmmc_mount_config_t mount{};
    mount.format_if_mount_failed = false;
    mount.max_files = 5;

    sdmmc_card_t *card = nullptr;
    err = esp_vfs_fat_sdspi_mount(
        "/sdcard", &host, &device, &mount, &card
    );
    if (err != ESP_OK) {
        printf("SD mount failed: %s\n", esp_err_to_name(err));
        spi_bus_free(static_cast<spi_host_device_t>(host.slot));
        return;
    }

    printf("SD card mounted successfully\n");
    sdmmc_card_print_info(stdout, card);

    DIR *dir = opendir("/sdcard");
    if (!dir) {
        printf("Could not list SD card files\n");
        return;
    }

    while (dirent *entry = readdir(dir)) {
        printf("SD file: %s\n", entry->d_name);
    }
    closedir(dir);
}

extern "C" void app_main() {
    check_sd_card();
    bluetooth::init();
    if (!wifi_files::init())
        printf("Wi-Fi file browser failed to start\n");
    gps::init();
    ui::init();
}

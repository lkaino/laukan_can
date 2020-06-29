/* SD card and FAT filesystem example.
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "sdkconfig.h"

#ifdef CONFIG_IDF_TARGET_ESP32
#include "driver/sdmmc_host.h"
#endif

static const char *TAG = "example";

#define MOUNT_POINT "/sdcard"

// DMA channel to be used by the SPI peripheral
#ifndef SPI_DMA_CHAN
#define SPI_DMA_CHAN 1
#endif //SPI_DMA_CHAN

// When testing SD and SPI modes, keep in mind that once the card has been
// initialized in SPI mode, it can not be reinitialized in SD mode without
// toggling power to the card.

// Pin mapping when using SPI mode.
// With this mapping, SD card can be used both in SPI and 1-line SD mode.
// Note that a pull-up on CS line is required in SD mode.
#define GPIO_PIN_NUM_MISO GPIO_NUM_19 // pin 31
#define GPIO_PIN_NUM_MOSI GPIO_NUM_23 // pin 37
#define GPIO_PIN_NUM_CLK GPIO_NUM_18  // pin 30
#define GPIO_PIN_NUM_CS GPIO_NUM_5    // pin 29

void sd_init(void)
{
    esp_log_level_set(TAG, ESP_LOG_VERBOSE);

    Serial.printf("Using SPI peripheral");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    sdspi_slot_config_t slot_config = SDSPI_SLOT_CONFIG_DEFAULT();
    slot_config.gpio_miso = GPIO_PIN_NUM_MISO;
    slot_config.gpio_mosi = GPIO_PIN_NUM_MOSI;
    slot_config.gpio_sck = GPIO_PIN_NUM_CLK;
    slot_config.gpio_cs = GPIO_PIN_NUM_CS;
    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.

    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024};

    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc_mount is an all-in-one convenience function.
    // Please check its source code and implement error recovery when developing
    // production applications.
    sdmmc_card_t *card;
    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            Serial.printf("Failed to mount filesystem. "
                          "If you want the card to be formatted, set format_if_mount_failed = true.");
        }
        else
        {
            Serial.printf("Failed to initialize the card (%s). "
                          "Make sure SD card lines have pull-up resistors in place.",
                          esp_err_to_name(ret));
        }
        return;
    }

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);

    // Use POSIX and C standard library functions to work with files.
    // First create a file.
    Serial.printf("Opening file\n");
    FILE *f = fopen("/sdcard/hello.txt", "w");
    if (f == NULL)
    {
        Serial.printf("Failed to open file for writing\n");
        return;
    }
    fprintf(f, "Hello %s!\n", card->cid.name);
    fclose(f);
    Serial.printf("File written\n");

    // Check if destination file exists before renaming
    struct stat st;
    if (stat("/sdcard/foo.txt", &st) == 0)
    {
        // Delete it if it exists
        unlink("/sdcard/foo.txt\n");
    }

    // Rename original file
    Serial.printf("Renaming file");
    if (rename("/sdcard/hello.txt", "/sdcard/foo.txt") != 0)
    {
        Serial.printf("Rename failed\n");
        return;
    }

    // Open renamed file for reading
    Serial.printf("Reading file\n");
    f = fopen("/sdcard/foo.txt", "r");
    if (f == NULL)
    {
        Serial.printf("Failed to open file for reading\n");
        return;
    }
    char line[64];
    fgets(line, sizeof(line), f);
    fclose(f);
    // strip newline
    char *pos = strchr(line, '\n');
    if (pos)
    {
        *pos = '\0';
    }
    Serial.printf("Read from file: '%s'\n", line);

    // All done, unmount partition and disable SDMMC or SPI peripheral
    esp_vfs_fat_sdmmc_unmount();
    Serial.printf("Card unmounted\n");
}
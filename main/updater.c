/* esp32_mouse_keyboard - bootloader for firmware upgrades
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * 
 * Copyright 2020-2021
 * Benjamin Aigner <beni@asterics-foundation.org>,<aignerb@technikum-wien.at>
 * Junaid Khan <junaid.khan.wien@gmail.com>
 *
 * This file is mostly based on the Espressif ESP32 native ota example.
 * Adaption were made:
 * 
 * * Update done via UART
 * * Cleanup & downsizing
 * * Status feedback via UART
 *
 */

 /* Original license text:
    This example code is in the Public Domain (or CC0 licensed, at your option.)
    Unless required by applicable law or agreed to in writing, this software is
    distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
    CONDITIONS OF ANY KIND, either express or implied.
 */


#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_flash_partitions.h"
#include "esp_partition.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "errno.h"
#include "driver/uart.h"

#define BUFFSIZE 512
#define HASH_LEN 32 /* SHA-256 digest length */

static const char *TAG = "esp32_addon_ota";
/*an ota data write buffer ready to write to the flash*/
static char ota_write_data[BUFFSIZE + 1] = { 0 };

const char* nl = "\r\n";


static void __attribute__((noreturn)) task_fatal_error(const char * message)
{
    uart_write_bytes(UART_NUM_0, "OTA:error-", strlen("OTA:error-"));
    uart_write_bytes(UART_NUM_0,message,strnlen(message,128));
    uart_write_bytes(UART_NUM_0, nl, sizeof(nl));
    vTaskDelay(2);
    esp_restart();
}

static void ota_example_task(void *pvParameter)
{
    esp_err_t err;
    /* update handle : set by esp_ota_begin(), must be freed via esp_ota_end() */
    esp_ota_handle_t update_handle = 0 ;
    const esp_partition_t *update_partition = NULL;

    ESP_LOGI(TAG, "Starting OTA example");

	int changePinning = 0;
    
    /*  determine if we should switch RX/TX pins.  */
    /*   we enable the RX pin as GPIO with pull-down.
     * 	 if "1" is read, this is the "real" RX pin. If not, it
     *   should be the TX pin. */
    
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask, we want to set the RX pin as input
    io_conf.pin_bit_mask = (1ULL<<GPIO_NUM_17);
    //disable pull-down mode
    io_conf.pull_down_en = 1;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    
    vTaskDelay(1);

	//Install UART driver, and get the queue.
	esp_err_t ret = ESP_OK;
	const uart_config_t uart_config = {
		.baud_rate = 500000,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
	};
	
	//update UART config
	ret = uart_param_config(UART_NUM_0, &uart_config);
	if(ret != ESP_OK) 
	{
		ESP_LOGE(TAG,"external UART param config failed"); 
	}
	
	//set IO pins: not necessary, on RP2040 we use default UART0 config
	
	//install driver
    uart_driver_install(UART_NUM_0, UART_FIFO_LEN * 1024, UART_FIFO_LEN * 2, 0, NULL, 0);
    
    const esp_partition_t *configured = esp_ota_get_boot_partition();
    const esp_partition_t *running = esp_ota_get_running_partition();

    if (configured != running) {
        ESP_LOGW(TAG, "Configured OTA boot partition at offset 0x%08x, but running from offset 0x%08x",
                 configured->address, running->address);
        ESP_LOGW(TAG, "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
    }
    ESP_LOGI(TAG, "Running partition type %d subtype %d (offset 0x%08x)",
             running->type, running->subtype, running->address);


    update_partition = esp_ota_get_next_update_partition(NULL);
    if(update_partition == NULL) task_fatal_error("partition not found");
    ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%x",
             update_partition->subtype, update_partition->address);

    err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
		task_fatal_error("ota begin failed");
	}
	ESP_LOGI(TAG, "esp_ota_begin succeeded");

	//send message via UART for signalling ready.
	uart_write_bytes(UART_NUM_0, "OTA:ready", strlen("OTA:ready"));
    uart_write_bytes(UART_NUM_0, nl, sizeof(nl));
    uart_flush(UART_NUM_0);
    uart_flush_input(UART_NUM_0);

	//receive binary image via UART.
    int binary_file_length = 0;
    while (1) {
        int data_read = uart_read_bytes(UART_NUM_0, (uint8_t*) ota_write_data, BUFFSIZE, 2000/portTICK_PERIOD_MS);
        if (data_read > 0) {
            err = esp_ota_write( update_handle, (const void *)ota_write_data, data_read);
            if (err != ESP_OK) {
                task_fatal_error("ota write failed");
            }
            binary_file_length += data_read;
        } else if (data_read == 0 && binary_file_length > 100000) {
           ESP_LOGE(TAG, "empty buffer, assume finished update...");
           break;
        }
    }
    ESP_LOGI(TAG, "Total Write binary data length: %d", binary_file_length);

	
    err = esp_ota_end(update_handle);
    if (err != ESP_OK) {
        if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
            ESP_LOGE(TAG, "Image validation failed, image is corrupted");
        }
        ESP_LOGE(TAG, "esp_ota_end failed (%s)!", esp_err_to_name(err));
        task_fatal_error(esp_err_to_name(err));
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
        task_fatal_error(esp_err_to_name(err));
    }

    uart_flush(UART_NUM_0);
    uart_flush_input(UART_NUM_0);

    ESP_LOGE(TAG, "OTA is finished");
    uart_write_bytes(UART_NUM_0, "OTA:$FINISHED", strlen("OTA:$FINISHED"));
    uart_write_bytes(UART_NUM_0, nl, sizeof(nl));

    ESP_LOGI(TAG, "Prepare to restart system!");
    esp_restart();
    return ;
}

void app_main(void)
{
    // Initialize NVS.
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // OTA app partition table has a smaller NVS partition size than the non-OTA
        // partition table. This size mismatch may cause NVS initialization to fail.
        // If this happens, we erase NVS partition and initialize NVS again.
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    xTaskCreate(&ota_example_task, "esp32-addon-OTA", 8192, NULL, 5, NULL);
}

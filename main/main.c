#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include "esp_netif.h"
#include "sdkconfig.h"

#include "my_wifi.h"
#include "my_server.h"
#include "led_driver.h"

static const char *TAG = "led_controller";

void app_main(void)
{
    //Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    init_virtual_dns(CONFIG_SERVER_MDNS_HOST_NAME);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    if (wifi_init()) {
    	ESP_LOGI(TAG, "Connected to WiFi");
    	start_rest_server();
    } else {
    	ESP_LOGE(TAG, "Continuing without web interface.");
    }

    led_driver_init(1200);
}

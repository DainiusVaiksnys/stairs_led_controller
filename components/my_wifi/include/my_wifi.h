#include "freertos/event_groups.h"
#include "esp_system.h"

static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

/* Requires:
    esp_netif_init()
    esp_event_loop_create_default()
*/
bool wifi_init(void);

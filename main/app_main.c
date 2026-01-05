/* tls-mutual-auth example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "protocol_examples_common.h"

#include "esp_log.h"
#include "esp_netif_ip_addr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

int aws_iot_demo_main( int argc, char ** argv );

static const char *TAG = "MQTT_EXAMPLE";

static void print_and_fix_dns( void )
{
    esp_netif_t * netif = get_example_netif();
    if( netif == NULL )
    {
        ESP_LOGW( TAG, "No example netif found; cannot read/set DNS" );
        return;
    }

    /* Reduce flakiness on some APs/guest networks: disable STA power save. */
    ( void ) esp_wifi_set_ps( WIFI_PS_NONE );

    esp_netif_dns_info_t dns_main = { 0 };
    if( esp_netif_get_dns_info( netif, ESP_NETIF_DNS_MAIN, &dns_main ) == ESP_OK )
    {
        if( dns_main.ip.type == ESP_IPADDR_TYPE_V4 )
        {
            ESP_LOGI( TAG, "DNS(main) = " IPSTR, IP2STR( &dns_main.ip.u_addr.ip4 ) );
        }
        else
        {
            ESP_LOGI( TAG, "DNS(main) type = %d", (int) dns_main.ip.type );
        }
    }
    else
    {
        ESP_LOGW( TAG, "Failed to read DNS(main)" );
    }

    /* Force known public DNS to avoid intermittent getaddrinfo(EAI_FAIL=202). */
    ESP_LOGW( TAG, "Setting DNS to 8.8.8.8 (main) and 1.1.1.1 (backup)" );
    esp_netif_dns_info_t dns1 = { 0 };
    dns1.ip.type = ESP_IPADDR_TYPE_V4;
    ESP_ERROR_CHECK( esp_netif_str_to_ip4( "8.8.8.8", &dns1.ip.u_addr.ip4 ) );
    ESP_ERROR_CHECK( esp_netif_set_dns_info( netif, ESP_NETIF_DNS_MAIN, &dns1 ) );

    esp_netif_dns_info_t dns2 = { 0 };
    dns2.ip.type = ESP_IPADDR_TYPE_V4;
    ESP_ERROR_CHECK( esp_netif_str_to_ip4( "1.1.1.1", &dns2.ip.u_addr.ip4 ) );
    ESP_ERROR_CHECK( esp_netif_set_dns_info( netif, ESP_NETIF_DNS_BACKUP, &dns2 ) );

    ESP_LOGI( TAG, "DNS updated." );
}

/*
 * Prototypes for the demos that can be started from this project.  Note the
 * MQTT demo is not actually started until the network is already.
 */

void app_main()
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %"PRIu32" bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    
    /* Initialize NVS partition */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /* NVS partition was truncated
         * and needs to be erased */
        ESP_ERROR_CHECK(nvs_flash_erase());

        /* Retry nvs_flash_init */
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    while( true )
    {
        esp_err_t err = example_connect();
        if( err == ESP_OK )
        {
            break;
        }
        ESP_LOGW( TAG, "example_connect() failed: %s. Retrying in 5 seconds...", esp_err_to_name( err ) );
        vTaskDelay( pdMS_TO_TICKS( 5000 ) );
    }

    print_and_fix_dns();

    aws_iot_demo_main(0,NULL);
}

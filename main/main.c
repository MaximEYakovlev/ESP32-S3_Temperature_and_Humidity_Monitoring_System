#include <stdio.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_http_server.h"
#include "esp_log.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "freertos/event_groups.h"

#define I2C_MASTER_NUM 0
#define I2C_MASTER_SCL_IO 2
#define I2C_MASTER_SDA_IO 3
#define I2C_MASTER_FREQ_HZ 100000

#define WIFI_SSID "Nist"
#define WIFI_PASS "vistnist21"
#define MAX_RETRY 5

static EventGroupHandle_t s_wifi_event_group;
static const int WIFI_CONNECTED_BIT = BIT0;
static const char *TAG_WIFI = "wifi_station";
static int s_retry_num = 0;

static const char *TAG_SHT40 = "SHT40_SENSOR";
static const char *TAG_WEB = "WEB_SERVER";
QueueHandle_t temp_queue;

// I2C initialization function
void i2c_master_init()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ};
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// Temperature reading (replace with real reading function with SHT 40)
float read_temperature()
{
    // The function is a stub, there should be a call to the real reading function from SHT 40
    return 25.0; // Returns the test temperature
}

// Task for reading temperature from SHT 40 (running on the first core)
void read_temperature_task(void *arg)
{
    i2c_master_init();

    while (1)
    {
        float temperature = read_temperature();
        ESP_LOGI(TAG_SHT40, "Температура: %.2f°C", temperature);

        // Sending the data to the queue
        if (xQueueSend(temp_queue, &temperature, portMAX_DELAY) != pdPASS)
        {
            ESP_LOGE(TAG_SHT40, "Ошибка отправки данных в очередь");
        }

        vTaskDelay(pdMS_TO_TICKS(2000)); // Delay of 2 seconds between readings
    }
}

// HTTP request handler for temperature return
esp_err_t temp_get_handler(httpd_req_t *req)
{
    float latest_temperature = 0.0;
    char temp_str[64];

    // Getting data from the queue
    if (xQueueReceive(temp_queue, &latest_temperature, pdMS_TO_TICKS(100)) == pdPASS)
    {
        snprintf(temp_str, sizeof(temp_str), "{\"temperature\": \"%.2f\"}", latest_temperature);
    }
    else
    {
        snprintf(temp_str, sizeof(temp_str), "{\"error\": \"Failed to get temperature\"}");
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, temp_str, strlen(temp_str));

    return ESP_OK;
}

// Configuring the web server
void start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    httpd_uri_t temp_uri = {
        .uri = "/temperature",
        .method = HTTP_GET,
        .handler = temp_get_handler,
        .user_ctx = NULL};

    if (httpd_start(&server, &config) == ESP_OK)
    {
        httpd_register_uri_handler(server, &temp_uri);
    }
}

// The task for the web server (running on the second core)
void web_server_task(void *arg)
{
    start_webserver();
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(10000)); // The web server is running, we are not doing anything
    }
}

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < MAX_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG_WIFI, "retry to connect to the AP");
        }
        else
        {
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        }
        ESP_LOGI(TAG_WIFI, "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG_WIFI, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG_WIFI, "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG_WIFI, "connected to ap SSID:%s password:%s",
                 WIFI_SSID, WIFI_PASS);
    }
    else
    {
        ESP_LOGI(TAG_WIFI, "Failed to connect to SSID:%s, password:%s",
                 WIFI_SSID, WIFI_PASS);
    }
}

// The main function is to create tasks on different cores
void app_main(void)
{
    // Initializing NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initializing Wi-Fi
    wifi_init_sta();

    // Creating a queue to transfer the temperature between tasks
    temp_queue = xQueueCreate(5, sizeof(float));

    if (temp_queue == NULL)
    {
        ESP_LOGE("MAIN", "Failed to create a queue");
        return;
    }

    // Starting the temperature reading task on the first core
    xTaskCreatePinnedToCore(read_temperature_task, "temperature_task", 4096, NULL, 5, NULL, 0);

    // Running a web server task on the second core
    xTaskCreatePinnedToCore(web_server_task, "web_server_task", 4096, NULL, 5, NULL, 1);
}

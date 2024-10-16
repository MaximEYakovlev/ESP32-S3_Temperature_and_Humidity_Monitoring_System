// https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/linux-macos-setup.html 
// Step 4. Set up the Environment Variables
// . $HOME/esp/esp-idf/export.sh
// idf.py set-target esp32-S3
// idf.py build
// idf.py flash
// idf.py monitor

#include <stdio.h>
#include "driver/i2c.h"
#include "driver/adc.h"
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
#define SHT40_SENSOR_ADDR 0x44
#define SHT40_MEASURE_CMD 0xFD

#define WIFI_SSID "Nist"
#define WIFI_PASS "vistnist21"
#define MAX_RETRY 5

#define ADC1_CHANNEL_0 ADC1_CHANNEL_0
#define ADC2_CHANNEL_0 ADC2_CHANNEL_0

static EventGroupHandle_t s_wifi_event_group;
static const int WIFI_CONNECTED_BIT = BIT0;
static const char *TAG_WIFI = "wifi_station";
static const char *TAG_SHT40 = "SHT40_SENSOR";
static const char *TAG_WEB = "WEB_SERVER";
static int s_retry_num = 0;

typedef struct
{
    float temperature;
    float humidity;
} sensor_data;

typedef struct
{
    float voltage_0;
    float voltage_1;
} adc_data;

typedef struct
{
    float temperature;
    float humidity;
    float voltage_0;
    float voltage_1;
    unsigned long timestamp;
} sensor_record;

QueueHandle_t temp_queue;
QueueHandle_t temp_queue_adc;

#define MAX_RECORDS 1000
sensor_record records_buffer[MAX_RECORDS];
size_t records_count = 0;
bool is_recording = false;
SemaphoreHandle_t buffer_mutex;

static const char index_html[] = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>ESP32 Sensor Dashboard</title>
    <meta charset="utf-8" />
    <script>
        async function fetchData() {
            try {
                const response = await fetch('/data');
                const data = await response.json();
                if (!data.error) {
                    document.getElementById('temperature').innerText = data.temperature + ' °C';
                    document.getElementById('humidity').innerText = data.humidity + ' %';
                    document.getElementById('voltage1').innerText = data["voltage ADC1"] + ' V';
                    document.getElementById('voltage2').innerText = data["voltage ADC2"] + ' V';
                } else {
                    console.error(data.error);
                }
            } catch (error) {
                console.error('Error fetching data:', error);
            }
        }

        async function startRecording() {
            try {
                const response = await fetch('/start', {method: 'POST'});
                const text = await response.text();
                alert(text);
            } catch (error) {
                console.error('Error starting recording:', error);
            }
        }

        async function stopRecording() {
            try {
                const response = await fetch('/stop', {method: 'POST'});
                if (response.ok) {
                    const blob = await response.blob();
                    const url = window.URL.createObjectURL(blob);
                    const a = document.createElement('a');
                    a.href = url;
                    a.download = 'data.csv';
                    a.click();
                    window.URL.revokeObjectURL(url);
                    alert('The data has been downloaded successfully');
                } else {
                    const text = await response.text();
                    alert(text);
                }
            } catch (error) {
                console.error('Error stopping recording:', error);
            }
        }

        setInterval(fetchData, 500);
        window.onload = fetchData;
    </script>
</head>
<body>
    <h1>ESP32 Sensor Dashboard</h1>
    <p>Temperature: <span id="temperature">--</span></p>
    <p>Humidity: <span id="humidity">--</span></p>
    <p>ADC Voltage 1: <span id="voltage1">--</span></p>
    <p>ADC Voltage 2: <span id="voltage2">--</span></p>
    <button onclick="startRecording()">START</button>
    <button onclick="stopRecording()">STOP</button>
</body>
</html>
)rawliteral";

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

void adc_init()
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_12);
    adc2_config_channel_atten(ADC2_CHANNEL_0, ADC_ATTEN_DB_12);
}

static esp_err_t sht40_read_data(float *temperature, float *humidity)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHT40_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, SHT40_MEASURE_CMD, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK)
    {
        return ret;
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);

    uint8_t data[6];
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHT40_SENSOR_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 6, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK)
    {
        *temperature = -45.0 + 175.0 * ((data[0] << 8 | data[1]) / 65535.0);
        *humidity = 100.0 * ((data[3] << 8 | data[4]) / 65535.0);
    }

    return ret;
}

void add_record(float temperature, float humidity, float voltage_0, float voltage_1)
{
    if (is_recording && records_count < MAX_RECORDS)
    {

        if (xSemaphoreTake(buffer_mutex, portMAX_DELAY) == pdTRUE)
        {
            records_buffer[records_count].temperature = temperature;
            records_buffer[records_count].humidity = humidity;
            records_buffer[records_count].voltage_0 = voltage_0;
            records_buffer[records_count].voltage_1 = voltage_1;
            records_buffer[records_count].timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
            records_count++;
            ESP_LOGI(TAG_WEB, "Record added: %zu", records_count);
            xSemaphoreGive(buffer_mutex);
        }
        else
        {
            ESP_LOGE(TAG_WEB, "Failed to take buffer mutex");
        }
    }
}

void read_sensor_task(void *arg)
{
    i2c_master_init();

    while (1)
    {
        float temperature = 0.0;
        float humidity = 0.0;

        if (sht40_read_data(&temperature, &humidity) == ESP_OK)
        {
            ESP_LOGI(TAG_SHT40, "Temperature: %.2f°C, Humidity: %.2f%%", temperature, humidity);

            sensor_data temp_data = {temperature, humidity};
            if (xQueueSend(temp_queue, &temp_data, portMAX_DELAY) != pdPASS)
            {
                ESP_LOGE(TAG_SHT40, "Failed to send data to queue");
            }
        }
        else
        {
            ESP_LOGE(TAG_SHT40, "Failed to read from SHT40 sensor");
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void read_adc_task(void *arg)
{
    adc_init();

    while (1)
    {
        int adc_value_0 = adc1_get_raw(ADC1_CHANNEL_0);

        int adc_value_1;
        esp_err_t ret = adc2_get_raw(ADC2_CHANNEL_0, ADC_WIDTH_BIT_12, &adc_value_1);
        if (ret != ESP_OK)
        {
            ESP_LOGE("ADC2", "Failed to read ADC2 channel 0");
            adc_value_1 = 0;
        }

        float voltage_0 = (adc_value_0 * 3.3) / 4095.0;
        float voltage_1 = (adc_value_1 * 3.3) / 4095.0;

        adc_data data_r = {voltage_0, voltage_1};

        if (xQueueSend(temp_queue_adc, &data_r, portMAX_DELAY) != pdPASS)
        {
            ESP_LOGE("ADC", "Failed to send data to queue");
        }

        ESP_LOGI("ADC", "ADC1 Channel 0: %.2f V, ADC2 Channel 0: %.2f V", voltage_0, voltage_1);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}ESP32-S3_Monitoring_System

void record_data_task(void *arg)
{
    sensor_data temp_data;
    adc_data adc_data_r;

    while (1)
    {
        if (xQueueReceive(temp_queue, &temp_data, portMAX_DELAY) == pdPASS &&
            xQueueReceive(temp_queue_adc, &adc_data_r, portMAX_DELAY) == pdPASS)
        {
            add_record(temp_data.temperature, temp_data.humidity, adc_data_r.voltage_0, adc_data_r.voltage_1);
        }ESP32-S3_Monitoring_System

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t data_get_handler(httpd_req_t *req)
{
    sensor_data temp_data;
    adc_data adc_data_r;

    char response[256];

    if (xQueueReceive(temp_queue, &temp_data, pdMS_TO_TICKS(100)) == pdPASS &&
        xQueueReceive(temp_queue_adc, &adc_data_r, pdMS_TO_TICKS(100)) == pdPASS)
    {
        snprintf(response, sizeof(response),
                 "{\"temperature\": \"%.2f\", \"humidity\": \"%.2f\", \"voltage ADC1\": \"%.2f\", \"voltage ADC2\": \"%.2f\"}",
                 temp_data.temperature, temp_data.humidity, adc_data_r.voltage_0, adc_data_r.voltage_1);
    }
    else
    {
        snprintf(response, sizeof(response), "{\"error\": \"Failed to get sensor data\"}");
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
    httpd_resp_send(req, response, strlen(response));

    return ESP_OK;
}

esp_err_t index_html_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-CESP32-S3_Monitoring_Systemontrol", "no-cache");
    httpd_resp_send(req, index_html, strlen(index_html));
    return ESP_OK;
}

esp_err_t start_recording_handler(httpd_req_t *req)
{
    if (!is_recording)
    {
        if (records_count >= MAX_RECORDS)
        {
            httpd_resp_send(req, "Buffer full. Cannot start recording.", HTTPD_RESP_USE_STRLEN);
            return ESP_FAIL;
        }

        is_recording = true;
        records_count = 0;
        ESP_LOGI(TAG_WEB, "Recording started");
        httpd_resp_send(req, "Recording started", HTTPD_RESP_USE_STRLEN);
    }
    else
    {
        httpd_resp_send(req, "Already recording", HTTPD_RESP_USE_STRLEN);
    }
    return ESP_OK;
}

esp_err_t stop_recording_handler(httpd_req_t *req)
{
    if (is_recording)
    {
        is_recording = false;
        ESP_LOGI(TAG_WEB, "Recording stopped");

        size_t csv_size = 256;
        char *csv_data = (char *)malloc(csv_size);
        if (!csv_data)
        {
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        strcpy(csv_data, "Timestamp,Temperature,Humidity,Voltage_ADC1,Voltage_ADC2\n");

        for (size_t i = 0; i < records_count; i++)
        {
            if (strlen(csv_data) + 100 > csv_size)
            {
                csv_size *= 2;
                char *new_csv_data = (char *)realloc(csv_data, csv_size);
                if (!new_csv_data)
                {
                    free(csv_data);
                    httpd_resp_send_500(req);
                    return ESP_FAIL;
                }
                csv_data = new_csv_data;
            }

            char line[100];
            snprintf(line, sizeof(line), "%lu,%.2f,%.2f,%.2f,%.2f\n",
                     records_buffer[i].timestamp,
                     records_buffer[i].temperature,
                     records_buffer[i].humidity,
                     records_buffer[i].voltage_0,
                     records_buffer[i].voltage_1);
            strcat(csv_data, line);
        }

        httpd_resp_set_type(req, "text/csv");
        httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=data.csv");
        httpd_resp_send(req, csv_data, strlen(csv_data));

        free(csv_data);
    }
    else
    {
        httpd_resp_send(req, "Not recording", HTTPD_RESP_USE_STRLEN);
    }
    return ESP_OK;
}

void start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_html_handler,
        .user_ctx = NULL};

    httpd_uri_t sensor_data_uri = {
        .uri = "/data",
        .method = HTTP_GET,
        .handler = data_get_handler,
        .user_ctx = NULL};

    httpd_uri_t start_uri = {
        .uri = "/start",
        .method = HTTP_POST,
        .handler = start_recording_handler,
        .user_ctx = NULL};

    httpd_uri_t stop_uri = {
        .uri = "/stop",
        .method = HTTP_POST,
        .handler = stop_recording_handler,
        .user_ctx = NULL};

    if (httpd_start(&server, &config) == ESP_OK)
    {
        httpd_register_uri_handler(server, &index_uri);
        httpd_register_uri_handler(server, &sensor_data_uri);
        httpd_register_uri_handler(server, &start_uri);
        httpd_register_uri_handler(server, &stop_uri);
        ESP_LOGI(TAG_WEB, "Web server started");
    }
    else
    {
        ESP_LOGE(TAG_WEB, "Failed to start web server");
    }
}

void web_server_task(void *arg)
{
    start_webserver();

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(10000));
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
            ESP_LOGI(TAG_WIFI, "Retry to connect to the AP");
        }
        else
        {
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        }
        ESP_LOGI(TAG_WIFI, "Connect to the AP failed");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG_WIFI, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS}};
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id);  // Обработчик Wi-Fi событий
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip); // Обработчик получения IP

    esp_wifi_start();

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG_WIFI, "Connected to AP SSID:%s password:%s",
                 WIFI_SSID, WIFI_PASS);
    }
    else
    {
        ESP_LOGI(TAG_WIFI, "Failed to connect to SSID:%s, password:%s",
                 WIFI_SSID, WIFI_PASS);
    }

    ESP_LOGI(TAG_WIFI, "wifi_init_sta finished.");
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);

    wifi_init_sta();

    temp_queue = xQueueCreate(10, sizeof(sensor_data));
    temp_queue_adc = xQueueCreate(10, sizeof(adc_data));

    if (temp_queue == NULL || temp_queue_adc == NULL)
    {
        ESP_LOGE(TAG_WEB, "Failed to create queues");
        return;
    }

    buffer_mutex = xSemaphoreCreateMutex();

    if (buffer_mutex == NULL)
    {
        ESP_LOGE(TAG_WEB, "Failed to create buffer mutex");
        return;
    }

    xTaskCreate(read_sensor_task, "read_sensor_task", 4096, NULL, 5, NULL);
    xTaskCreate(read_adc_task, "read_adc_task", 4096, NULL, 5, NULL);
    xTaskCreate(record_data_task, "record_data_task", 4096, NULL, 5, NULL);
    xTaskCreate(web_server_task, "web_server_task", 8192, NULL, 5, NULL);

    ESP_LOGI(TAG_WIFI, "System ready");
}

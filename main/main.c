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

// Definitions for I2C and SHT40 sensor
#define I2C_MASTER_NUM 0          // I2C port number
#define I2C_MASTER_SCL_IO 2       // GPIO number for I2C SCL (clock) line
#define I2C_MASTER_SDA_IO 3       // GPIO number for I2C SDA (data) line
#define I2C_MASTER_FREQ_HZ 100000 // I2C clock frequency in Hz
#define SHT40_SENSOR_ADDR 0x44    // Address of the SHT40 sensor
#define SHT40_MEASURE_CMD 0xFD    // Command to start a measurement on the SHT40 sensor

// Wi-Fi credentials
#define WIFI_SSID "Nist"
#define WIFI_PASS "vistnist21"
#define MAX_RETRY 5 // Maximum number of retry attempts for Wi-Fi connection

// Event group and tags for logging
static EventGroupHandle_t s_wifi_event_group;
static const int WIFI_CONNECTED_BIT = BIT0; // Bit used to indicate connection to Wi-Fi
static const char *TAG_WIFI = "wifi_station";
static const char *TAG_SHT40 = "SHT40_SENSOR";
static const char *TAG_WEB = "WEB_SERVER";
static int s_retry_num = 0; // Counter for Wi-Fi connection retries
QueueHandle_t temp_queue;   // Queue used for sharing temperature and humidity data

// I2C initialization function
void i2c_master_init()
{
    // Configure the I2C parameters
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,               // Set as I2C master mode
        .sda_io_num = I2C_MASTER_SDA_IO,       // Set the SDA pin
        .sda_pullup_en = GPIO_PULLUP_ENABLE,   // Enable pull-up for SDA
        .scl_io_num = I2C_MASTER_SCL_IO,       // Set the SCL pin
        .scl_pullup_en = GPIO_PULLUP_ENABLE,   // Enable pull-up for SCL
        .master.clk_speed = I2C_MASTER_FREQ_HZ // Set I2C frequency
    };
    // Initialize the I2C driver with the configured parameters
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0); // Install the I2C driver
}

// Function to read temperature and humidity from SHT40 sensor
static esp_err_t sht40_read_data(float *temperature, float *humidity)
{
    // Create a command link for I2C communication
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                                // Start the I2C transaction
    i2c_master_write_byte(cmd, (SHT40_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);        // Write the sensor's address with a write bit
    i2c_master_write_byte(cmd, SHT40_MEASURE_CMD, true);                                  // Send the measurement command to the sensor
    i2c_master_stop(cmd);                                                                 // Stop the I2C transaction
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS); // Execute the I2C command
    i2c_cmd_link_delete(cmd);                                                             // Delete the I2C command link

    if (ret != ESP_OK)
    {
        return ret; // Return error if command execution failed
    }

    vTaskDelay(50 / portTICK_PERIOD_MS); // Delay to allow the sensor to complete measurement

    // Read data from the sensor
    uint8_t data[6];
    cmd = i2c_cmd_link_create();                                                  // Create a new I2C command link
    i2c_master_start(cmd);                                                        // Start a new I2C transaction
    i2c_master_write_byte(cmd, (SHT40_SENSOR_ADDR << 1) | I2C_MASTER_READ, true); // Address the sensor for reading
    i2c_master_read(cmd, data, 6, I2C_MASTER_LAST_NACK);                          // Read 6 bytes of data (temperature + humidity + CRC)
    i2c_master_stop(cmd);                                                         // Stop the I2C transaction
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);   // Execute the read command
    i2c_cmd_link_delete(cmd);                                                     // Delete the I2C command link

    if (ret == ESP_OK)
    {
        // Convert the raw sensor data to temperature and humidity values
        *temperature = -45.0 + 175.0 * ((data[0] << 8 | data[1]) / 65535.0); // Convert temperature
        *humidity = 100.0 * ((data[3] << 8 | data[4]) / 65535.0);            // Convert humidity
    }

    return ret; // Return the result of the read operation
}

// Task for reading temperature and humidity data
void read_sensor_task(void *arg)
{
    i2c_master_init(); // Initialize the I2C interface

    while (1)
    {
        float temperature = 0.0;
        float humidity = 0.0;
        if (sht40_read_data(&temperature, &humidity) == ESP_OK) // Read sensor data
        {
            ESP_LOGI(TAG_SHT40, "Temperature: %.2f°C, Humidity: %.2f%%", temperature, humidity); // Log the sensor data

            // Structure to hold sensor data
            typedef struct
            {
                float temperature;
                float humidity;
            } sensor_data_t;

            sensor_data_t data = {temperature, humidity};               // Create a sensor data structure
            if (xQueueSend(temp_queue, &data, portMAX_DELAY) != pdPASS) // Send the data to the queue
            {
                ESP_LOGE(TAG_SHT40, "Failed to send data to queue"); // Log an error if the data could not be sent
            }
        }
        else
        {
            ESP_LOGE(TAG_SHT40, "Failed to read from SHT40 sensor"); // Log an error if reading from the sensor failed
        }

        vTaskDelay(pdMS_TO_TICKS(2000)); // Wait for 2 seconds before reading again
    }
}

// HTTP request handler for serving temperature and humidity data
esp_err_t sensor_data_get_handler(httpd_req_t *req)
{
    typedef struct
    {
        float temperature;
        float humidity;
    } sensor_data_t;

    sensor_data_t data;
    char response[128];

    // Retrieve sensor data from the queue
    if (xQueueReceive(temp_queue, &data, pdMS_TO_TICKS(100)) == pdPASS)
    {
        snprintf(response, sizeof(response), "{\"temperature\": \"%.2f\", \"humidity\": \"%.2f\"}", data.temperature, data.humidity); // Format the sensor data into a JSON response
    }
    else
    {
        snprintf(response, sizeof(response), "{\"error\": \"Failed to get sensor data\"}"); // Send an error message if data retrieval fails
    }

    httpd_resp_set_type(req, "application/json");                            // Set the response type to JSON
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");             // Allow all origins
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET");          // Allow only GET requests
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type"); // Allow headers
    httpd_resp_send(req, response, strlen(response));                        // Send the response

    return ESP_OK; // Return success
}

// Function to configure and start the web server
void start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG(); // Default server configuration

    // URI handler for serving sensor data
    httpd_uri_t sensor_data_uri = {
        .uri = "/sensor_data",
        .method = HTTP_GET,
        .handler = sensor_data_get_handler,
        .user_ctx = NULL};

    if (httpd_start(&server, &config) == ESP_OK) // Start the web server
    {
        httpd_register_uri_handler(server, &sensor_data_uri); // Register the URI handler for sensor data
    }
}

// Task to run the web server
void web_server_task(void *arg)
{
    start_webserver(); // Start the web server
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(10000)); // Keep the task alive by delaying periodically
    }
}

// Event handler for Wi-Fi events
static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect(); // Attempt to connect to the Wi-Fi
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < MAX_RETRY)
        {
            esp_wifi_connect(); // Retry connecting to Wi-Fi
            s_retry_num++;
            ESP_LOGI(TAG_WIFI, "Retry to connect to the AP");
        }
        else
        {
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT); // Clear the connected bit
        }
        ESP_LOGI(TAG_WIFI, "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;      // Get the IP address event
        ESP_LOGI(TAG_WIFI, "got ip:" IPSTR, IP2STR(&event->ip_info.ip)); // Log the IP address
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT); // Set the connected bit
    }
}

// Initialize Wi-Fi as a station (client)
void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate(); // Create the event group for Wi-Fi events

    // Initialize TCP/IP stack
    esp_netif_init();
    esp_event_loop_create_default();     // Create default event loop
    esp_netif_create_default_wifi_sta(); // Create default Wi-Fi station (client)

    // Configure Wi-Fi with credentials
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    // Set up Wi-Fi station mode and event handlers
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS}};
    esp_wifi_set_mode(WIFI_MODE_STA);               // Set Wi-Fi mode to station (client)
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config); // Set Wi-Fi configuration

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id);  // Register handler for Wi-Fi events
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip); // Register handler for IP events

    esp_wifi_start(); // Start the Wi-Fi driver

    // Wait for connection
    ESP_LOGI(TAG_WIFI, "wifi_init_sta finished.");
}

// Main function
void app_main(void)
{
    // Initialize non-volatile storage (NVS)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize Wi-Fi connection
    wifi_init_sta();

    // Create a queue for temperature and humidity data
    temp_queue = xQueueCreate(10, sizeof(float) * 2); // Queue can hold 10 sensor readings

    // Create a task to read sensor data from the SHT40 sensor
    xTaskCreate(read_sensor_task, "read_sensor_task", 2048, NULL, 5, NULL);

    // Create a task to run the web server
    xTaskCreate(web_server_task, "web_server_task", 4096, NULL, 5, NULL);
}

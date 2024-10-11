#include <stdio.h>                 // Standard I/O library for input and output functions.
#include <string.h>                // String manipulation functions.
#include "driver/i2c.h"            // I2C driver for I2C communication.
#include "driver/adc.h"            // ADC driver for analog-to-digital conversion.
#include "freertos/FreeRTOS.h"     // FreeRTOS definitions and macros for operating system functionality.
#include "freertos/task.h"         // FreeRTOS task management functions for creating and managing tasks.
#include "esp_log.h"               // Logging library for logging messages and errors.
#include "esp_wifi.h"              // Wi-Fi library for Wi-Fi functionality.
#include "esp_event.h"             // Event library for event handling in ESP-IDF.
#include "nvs_flash.h"             // Non-Volatile Storage (NVS) library for flash memory operations.
#include "esp_system.h"            // System-level functions and definitions for the ESP32 system.
#include "freertos/event_groups.h" // FreeRTOS event group functions for managing multiple event flags.
#include "esp_spiffs.h"            // SPIFFS library for filesystem operations.

#define I2C_MASTER_NUM 0          // I2C port number.
#define I2C_MASTER_SCL_IO 2       // GPIO number for I2C SCL (clock) line.
#define I2C_MASTER_SDA_IO 3       // GPIO number for I2C SDA (data) line.
#define I2C_MASTER_FREQ_HZ 100000 // I2C clock frequency in Hz.
#define SHT40_SENSOR_ADDR 0x44    // Address of the SHT40 sensor.
#define SHT40_MEASURE_CMD 0xFD    // Command to start a measurement on the SHT40 sensor.

#define WIFI_SSID "Nist"       // Wi-Fi SSID.
#define WIFI_PASS "vistnist21" // Wi-Fi password.
#define MAX_RETRY 5            // Maximum number of retry attempts for Wi-Fi connection.

#define ADC1_CHANNEL ADC1_CHANNEL_0 // GPIO1 corresponds to ADC1 channel 0.
#define ADC2_CHANNEL ADC2_CHANNEL_0 // GPIO11 corresponds to ADC2 channel 0.

static const char *TAG = "Main"; // General tag for logging.

static EventGroupHandle_t s_wifi_event_group;
static const int WIFI_CONNECTED_BIT = BIT0; // Bit used to indicate connection to Wi-Fi.
static int s_retry_num = 0;                 // Counter for Wi-Fi connection retries.

// Structure to hold sensor data.
typedef struct
{
    float temperature;
    float humidity;
} sensor_data;

// Structure to hold ADC data.
typedef struct
{
    float voltage_0;
    float voltage_1;
} adc_data;

// Function Prototypes.
void i2c_master_init();
void adc_init();
void write_data_to_file(const char *filename, const char *data);
void init_spiffs();
static esp_err_t sht40_read_data(float *temperature, float *humidity);
void read_sensor_task(void *arg);
void read_adc_task(void *arg);
static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void wifi_init_sta();
void app_main(void);

// Function to initialize SPIFFS.
void init_spiffs()
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",        // Mount point.
        .partition_label = NULL,       // Use default partition.
        .max_files = 5,                // Maximum number of open files.
        .format_if_mount_failed = true // Format if mount fails.
    };

    // Register and mount SPIFFS.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(TAG, "Failed to mount or format SPIFFS");
        }
        else if (ret == ESP_ERR_NOT_FOUND)
        {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI(TAG, "SPIFFS partition size: total: %d, used: %d", total, used);
    }
}

// Helper function to write data to a file.
void write_data_to_file(const char *filename, const char *data)
{
    FILE *f = fopen(filename, "a"); // Open file in append mode.
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file %s for writing", filename);
        return;
    }
    fprintf(f, "%s\n", data); // Write data followed by a newline.
    fclose(f);                // Close the file.
}

// I2C initialization function.
void i2c_master_init()
{
    // Configure the I2C parameters.
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,               // Set as I2C master mode.
        .sda_io_num = I2C_MASTER_SDA_IO,       // Set the SDA pin.
        .sda_pullup_en = GPIO_PULLUP_ENABLE,   // Enable pull-up for SDA.
        .scl_io_num = I2C_MASTER_SCL_IO,       // Set the SCL pin.
        .scl_pullup_en = GPIO_PULLUP_ENABLE,   // Enable pull-up for SCL.
        .master.clk_speed = I2C_MASTER_FREQ_HZ // Set I2C frequency.
    };
    // Initialize the I2C driver with the configured parameters.
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0); // Install the I2C driver.
}

// ADC initialization function.
void adc_init()
{
    // Configure the ADC1 width (resolution: 12-bit).
    adc1_config_width(ADC_WIDTH_BIT_12);
    // Configure attenuation for ADC1 channel.
    adc1_config_channel_atten(ADC1_CHANNEL, ADC_ATTEN_DB_12);

    // Configure attenuation for ADC2 channel.
    // Note: ADC2 is used by Wi-Fi, usage is limited.
    adc2_config_channel_atten(ADC2_CHANNEL, ADC_ATTEN_DB_12);
}

// Function to read temperature and humidity from SHT40 sensor.
static esp_err_t sht40_read_data(float *temperature, float *humidity)
{
    // Create a command link for I2C communication.
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                                // Start the I2C transaction.
    i2c_master_write_byte(cmd, (SHT40_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);        // Write the sensor's address with a write bit.
    i2c_master_write_byte(cmd, SHT40_MEASURE_CMD, true);                                  // Send the measurement command to the sensor.
    i2c_master_stop(cmd);                                                                 // Stop the I2C transaction.
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS); // Execute the I2C command.
    i2c_cmd_link_delete(cmd);                                                             // Delete the I2C command link.

    if (ret != ESP_OK)
    {
        return ret; // Return error if command execution failed.
    }

    vTaskDelay(50 / portTICK_PERIOD_MS); // Delay to allow the sensor to complete measurement.

    // Read data from the sensor.
    uint8_t data[6];
    cmd = i2c_cmd_link_create();                                                  // Create a new I2C command link.
    i2c_master_start(cmd);                                                        // Start a new I2C transaction.
    i2c_master_write_byte(cmd, (SHT40_SENSOR_ADDR << 1) | I2C_MASTER_READ, true); // Address the sensor for reading.
    i2c_master_read(cmd, data, 6, I2C_MASTER_LAST_NACK);                          // Read 6 bytes of data (temperature + humidity + CRC).
    i2c_master_stop(cmd);                                                         // Stop the I2C transaction.
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);   // Execute the read command.
    i2c_cmd_link_delete(cmd);                                                     // Delete the I2C command link.

    if (ret == ESP_OK)
    {
        // Convert the raw sensor data to temperature and humidity values.
        *temperature = -45.0 + 175.0 * ((data[0] << 8 | data[1]) / 65535.0); // Convert temperature.
        *humidity = 100.0 * ((data[3] << 8 | data[4]) / 65535.0);            // Convert humidity.
    }

    return ret; // Return the result of the read operation.
}

// Task for reading temperature and humidity data.
void read_sensor_task(void *arg)
{
    i2c_master_init(); // Initialize the I2C interface.

    while (1)
    {
        float temperature = 0.0;
        float humidity = 0.0;

        if (sht40_read_data(&temperature, &humidity) == ESP_OK)
        {                                                                                  // Read sensor data.
            ESP_LOGI(TAG, "Temperature: %.2f°C, Humidity: %.2f%%", temperature, humidity); // Log the sensor data.

            // Prepare data string.
            char data_str[100];
            snprintf(data_str, sizeof(data_str), "Temperature: %.2f°C, Humidity: %.2f%%", temperature, humidity);

            // Write data to file.
            write_data_to_file("/spiffs/sensor_data.txt", data_str);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to read from SHT40 sensor");
        }

        vTaskDelay(pdMS_TO_TICKS(2000)); // Wait for 2 seconds before reading again.
    }
}

// Task for reading ADC values and saving them to a file.
void read_adc_task(void *arg)
{
    adc_init(); // Initialize ADC.

    while (1)
    {
        // Read raw ADC value from ADC1_CHANNEL.
        int adc_value_0 = adc1_get_raw(ADC1_CHANNEL); // Read raw ADC value from Channel 0.

        // Read raw ADC value from ADC2_CHANNEL.
        int adc_value_1;
        esp_err_t ret = adc2_get_raw(ADC2_CHANNEL, ADC_WIDTH_BIT_12, &adc_value_1); // Read ADC2 channel 0.
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to read ADC2 channel 0");
            adc_value_1 = 0; // Assign default value or handle error as needed.
        }

        // Convert raw values to voltage (assuming 12-bit resolution, 3.3V reference).
        float voltage_0 = (adc_value_0 * 3.3) / 4095.0;
        float voltage_1 = (adc_value_1 * 3.3) / 4095.0;

        // Prepare data string.
        char data_str[100];
        snprintf(data_str, sizeof(data_str), "ADC1 Channel 0: %.2f V, ADC2 Channel 0: %.2f V", voltage_0, voltage_1);

        // Write data to file.
        write_data_to_file("/spiffs/adc_data.txt", data_str);

        // Log the voltage readings.
        ESP_LOGI(TAG, "%s", data_str);

        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second.
    }
}

// Event handler for Wi-Fi events.
static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect(); // Attempt to connect to the Wi-Fi.
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < MAX_RETRY)
        {
            esp_wifi_connect(); // Retry connecting to Wi-Fi.
            s_retry_num++;
            ESP_LOGI(TAG, "Retry to connect to the AP");
        }
        else
        {
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT); // Clear the connected bit.
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data; // Get the IP address event.
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip)); // Log the IP address.
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT); // Set the connected bit.
    }
}

// Initialize Wi-Fi as a station (client).
void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate(); // Create the event group for Wi-Fi events.

    // Initialize TCP/IP stack.
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default()); // Create default event loop.
    esp_netif_create_default_wifi_sta();              // Create default Wi-Fi station (client).

    // Configure Wi-Fi with credentials.
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Set up Wi-Fi station mode and event handlers.
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS}};
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));               // Set Wi-Fi mode to station (client).
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config)); // Set Wi-Fi configuration.

    // Register event handlers.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL));  // Register handler for Wi-Fi events.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL)); // Register handler for IP events.

    ESP_ERROR_CHECK(esp_wifi_start()); // Start the Wi-Fi driver.

    ESP_LOGI(TAG, "wifi_init_sta finished.");
}

// Main function.
void app_main(void)
{
    // Initialize non-volatile storage (NVS).
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize SPIFFS filesystem.
    init_spiffs();

    // Test writing
    write_data_to_file("/spiffs/test.txt", "Hello, SPIFFS!");

    // Test reading
    FILE *f = fopen("/spiffs/test.txt", "r");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Couldn't open test.txt for reading");
    }
    else
    {
        char line[64];
        fgets(line, sizeof(line), f);
        ESP_LOGI(TAG, "The test.txt content: %s", line);
        fclose(f);
    }

    // Initialize Wi-Fi connection.
    wifi_init_sta();

    // Create tasks.
    xTaskCreate(read_sensor_task, "read_sensor_task", 4096, NULL, 5, NULL);
    xTaskCreate(read_adc_task, "read_adc_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "System ready");
}

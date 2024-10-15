#include <stdio.h>                 // Стандартная библиотека ввода-вывода
#include "driver/i2c.h"            // Библиотека драйвера I2C
#include "driver/adc.h"            // Библиотека драйвера ADC
#include "freertos/FreeRTOS.h"     // FreeRTOS определения и макросы
#include "freertos/task.h"         // FreeRTOS управление задачами
#include "freertos/queue.h"        // FreeRTOS управление очередями
#include "esp_http_server.h"       // Библиотека HTTP сервера
#include "esp_log.h"               // Библиотека логирования
#include "esp_wifi.h"              // Библиотека Wi-Fi
#include "esp_event.h"             // Библиотека обработки событий
#include "nvs_flash.h"             // Библиотека NVS
#include "esp_system.h"            // Системные функции ESP32
#include "freertos/event_groups.h" // FreeRTOS группы событий

#include <string.h>                // Для работы со строками
#include <stdlib.h>                // Для динамического выделения памяти

// Определения для I2C и датчика SHT40
#define I2C_MASTER_NUM 0          // I2C порт
#define I2C_MASTER_SCL_IO 2       // GPIO для SCL
#define I2C_MASTER_SDA_IO 3       // GPIO для SDA
#define I2C_MASTER_FREQ_HZ 100000 // Частота I2C
#define SHT40_SENSOR_ADDR 0x44    // Адрес SHT40
#define SHT40_MEASURE_CMD 0xFD    // Команда измерения

// Учетные данные Wi-Fi
#define WIFI_SSID "Nist"
#define WIFI_PASS "vistnist21"
#define MAX_RETRY 5 // Максимальное количество попыток подключения

// Определение каналов ADC
#define ADC1_CHANNEL_0 ADC1_CHANNEL_0 // GPIO1 - ADC1 channel 0
#define ADC2_CHANNEL_0 ADC2_CHANNEL_0 // GPIO11 - ADC2 channel 0

// Группа событий и теги для логирования
static EventGroupHandle_t s_wifi_event_group;
static const int WIFI_CONNECTED_BIT = BIT0; // Бит для индикации подключения
static const char *TAG_WIFI = "wifi_station";
static const char *TAG_SHT40 = "SHT40_SENSOR";
static const char *TAG_WEB = "WEB_SERVER";
static int s_retry_num = 0; // Счетчик попыток подключения

// Структура для хранения данных с датчиков
typedef struct
{
    float temperature;
    float humidity;
} sensor_data;

// Структура для хранения данных ADC
typedef struct
{
    float voltage_0;
    float voltage_1;
} adc_data;

// Структура для полной записи данных
typedef struct
{
    float temperature;
    float humidity;
    float voltage_0;
    float voltage_1;
    unsigned long timestamp;
} sensor_record;

// Объявление очередей
QueueHandle_t temp_queue;     // Очередь для температуры и влажности
QueueHandle_t temp_queue_adc; // Очередь для данных ADC

// Буфер для записи данных
#define MAX_RECORDS 1000
sensor_record records_buffer[MAX_RECORDS];
size_t records_count = 0;
bool is_recording = false;
SemaphoreHandle_t buffer_mutex; // Мьютекс для защиты буфера

// HTML код для отображения данных и кнопок СТАРТ/СТОП
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

        // Функция для начала записи
        async function startRecording() {
            try {
                const response = await fetch('/start', {method: 'POST'});
                const text = await response.text();
                alert(text);
            } catch (error) {
                console.error('Error starting recording:', error);
            }
        }

        // Функция для остановки записи и скачивания CSV
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
                    alert('Данные успешно скачаны');
                } else {
                    const text = await response.text();
                    alert(text);
                }
            } catch (error) {
                console.error('Error stopping recording:', error);
            }
        }

        // Обновление данных каждые 2 секунды
        setInterval(fetchData, 2000);
        window.onload = fetchData;
    </script>
</head>
<body>
    <h1>ESP32 Sensor Dashboard</h1>
    <p>Temperature: <span id="temperature">--</span></p>
    <p>Humidity: <span id="humidity">--</span></p>
    <p>ADC Voltage 1: <span id="voltage1">--</span></p>
    <p>ADC Voltage 2: <span id="voltage2">--</span></p>
    <button onclick="startRecording()">СТАРТ</button>
    <button onclick="stopRecording()">СТОП</button>
</body>
</html>
)rawliteral";

// Инициализация I2C
void i2c_master_init()
{
    // Конфигурация параметров I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,               // Режим I2C мастер
        .sda_io_num = I2C_MASTER_SDA_IO,       // Пин SDA
        .sda_pullup_en = GPIO_PULLUP_ENABLE,   // Включить подтяжку SDA
        .scl_io_num = I2C_MASTER_SCL_IO,       // Пин SCL
        .scl_pullup_en = GPIO_PULLUP_ENABLE,   // Включить подтяжку SCL
        .master.clk_speed = I2C_MASTER_FREQ_HZ // Частота I2C
    };
    // Инициализация параметров I2C
    i2c_param_config(I2C_MASTER_NUM, &conf);
    // Установка драйвера I2C
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// Инициализация ADC
void adc_init()
{
    // Конфигурация ширины ADC1 (разрешение 12-бит)
    adc1_config_width(ADC_WIDTH_BIT_12);
    // Конфигурация аттенюации для канала ADC1
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_12);

    // Конфигурация аттенюации для канала ADC2
    // Примечание: ADC2 используется Wi-Fi, возможности ограничены
    adc2_config_channel_atten(ADC2_CHANNEL_0, ADC_ATTEN_DB_12);
}

// Функция чтения температуры и влажности с SHT40
static esp_err_t sht40_read_data(float *temperature, float *humidity)
{
    // Создание команды для I2C
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                                // Старт I2C
    i2c_master_write_byte(cmd, (SHT40_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);        // Адрес устройства с битом записи
    i2c_master_write_byte(cmd, SHT40_MEASURE_CMD, true);                                  // Отправка команды измерения
    i2c_master_stop(cmd);                                                                 // Стоп I2C
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS); // Выполнение команды
    i2c_cmd_link_delete(cmd);                                                             // Удаление команды

    if (ret != ESP_OK)
    {
        return ret; // Возврат ошибки при неудаче
    }

    vTaskDelay(50 / portTICK_PERIOD_MS); // Задержка для завершения измерения

    // Чтение данных с сенсора
    uint8_t data[6];
    cmd = i2c_cmd_link_create();                                                  // Создание новой команды
    i2c_master_start(cmd);                                                        // Старт I2C
    i2c_master_write_byte(cmd, (SHT40_SENSOR_ADDR << 1) | I2C_MASTER_READ, true); // Адрес устройства с битом чтения
    i2c_master_read(cmd, data, 6, I2C_MASTER_LAST_NACK);                          // Чтение 6 байт данных
    i2c_master_stop(cmd);                                                         // Стоп I2C
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);   // Выполнение команды
    i2c_cmd_link_delete(cmd);                                                     // Удаление команды

    if (ret == ESP_OK)
    {
        // Конвертация сырых данных в температуру и влажность
        *temperature = -45.0 + 175.0 * ((data[0] << 8 | data[1]) / 65535.0); // Температура
        *humidity = 100.0 * ((data[3] << 8 | data[4]) / 65535.0);            // Влажность
    }

    return ret; // Возврат результата
}

// Задача для чтения данных с SHT40
void read_sensor_task(void *arg)
{
    i2c_master_init(); // Инициализация I2C

    while (1)
    {
        float temperature = 0.0;
        float humidity = 0.0;

        if (sht40_read_data(&temperature, &humidity) == ESP_OK) // Чтение данных
        {
            ESP_LOGI(TAG_SHT40, "Temperature: %.2f°C, Humidity: %.2f%%", temperature, humidity); // Лог данных

            sensor_data temp_data = {temperature, humidity};
            if (xQueueSend(temp_queue, &temp_data, portMAX_DELAY) != pdPASS) // Отправка в очередь
            {
                ESP_LOGE(TAG_SHT40, "Failed to send data to queue"); // Лог ошибки
            }
        }
        else
        {
            ESP_LOGE(TAG_SHT40, "Failed to read from SHT40 sensor"); // Лог ошибки
        }

        vTaskDelay(pdMS_TO_TICKS(2000)); // Задержка 2 секунды
    }
}

// Задача для чтения данных с ADC
void read_adc_task(void *arg)
{
    adc_init(); // Инициализация ADC

    while (1)
    {
        // Чтение сырого значения ADC1
        int adc_value_0 = adc1_get_raw(ADC1_CHANNEL_0);

        // Чтение сырого значения ADC2
        int adc_value_1;
        esp_err_t ret = adc2_get_raw(ADC2_CHANNEL_0, ADC_WIDTH_BIT_12, &adc_value_1);
        if (ret != ESP_OK)
        {
            ESP_LOGE("ADC2", "Failed to read ADC2 channel 0");
            adc_value_1 = 0; // Значение по умолчанию
        }

        // Конвертация в напряжение (предполагается 3.3V опорное напряжение)
        float voltage_0 = (adc_value_0 * 3.3) / 4095.0;
        float voltage_1 = (adc_value_1 * 3.3) / 4095.0;

        // Создание структуры с данными ADC
        adc_data data_r = {voltage_0, voltage_1};

        if (xQueueSend(temp_queue_adc, &data_r, portMAX_DELAY) != pdPASS) // Отправка в очередь
        {
            ESP_LOGE("ADC", "Failed to send data to queue"); // Лог ошибки
        }

        // Логирование данных
        ESP_LOGI("ADC", "ADC1 Channel 0: %.2f V, ADC2 Channel 0: %.2f V", voltage_0, voltage_1);

        vTaskDelay(pdMS_TO_TICKS(1000)); // Задержка 1 секунда
    }
}

// Функция добавления записи в буфер
void add_record(float temperature, float humidity, float voltage_0, float voltage_1)
{
    if (is_recording && records_count < MAX_RECORDS)
    {
        // Захват мьютекса для безопасного доступа к буферу
        if (xSemaphoreTake(buffer_mutex, portMAX_DELAY) == pdTRUE)
        {
            records_buffer[records_count].temperature = temperature;
            records_buffer[records_count].humidity = humidity;
            records_buffer[records_count].voltage_0 = voltage_0;
            records_buffer[records_count].voltage_1 = voltage_1;
            records_buffer[records_count].timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
            records_count++;
            ESP_LOGI(TAG_WEB, "Record added: %zu", records_count);
            xSemaphoreGive(buffer_mutex); // Освобождение мьютекса
        }
        else
        {
            ESP_LOGE(TAG_WEB, "Failed to take buffer mutex");
        }
    }
}

// Задача для записи данных из очередей в буфер
void record_data_task(void *arg)
{
    sensor_data temp_data;
    adc_data adc_data_r;

    while (1)
    {
        // Получение данных из очередей
        if (xQueueReceive(temp_queue, &temp_data, portMAX_DELAY) == pdPASS &&
            xQueueReceive(temp_queue_adc, &adc_data_r, portMAX_DELAY) == pdPASS)
        {
            // Добавление записи в буфер, если запись активна
            add_record(temp_data.temperature, temp_data.humidity, adc_data_r.voltage_0, adc_data_r.voltage_1);
        }

        // Минимальная задержка для предотвращения занятости CPU
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Обработчик HTTP-запроса для получения данных
esp_err_t data_get_handler(httpd_req_t *req)
{
    sensor_data temp_data;
    adc_data adc_data_r;

    char response[256];

    // Получение данных из очередей
    if (xQueueReceive(temp_queue, &temp_data, pdMS_TO_TICKS(100)) == pdPASS &&
        xQueueReceive(temp_queue_adc, &adc_data_r, pdMS_TO_TICKS(100)) == pdPASS)
    {
        // Форматирование данных в JSON
        snprintf(response, sizeof(response),
                 "{\"temperature\": \"%.2f\", \"humidity\": \"%.2f\", \"voltage ADC1\": \"%.2f\", \"voltage ADC2\": \"%.2f\"}",
                 temp_data.temperature, temp_data.humidity, adc_data_r.voltage_0, adc_data_r.voltage_1);
    }
    else
    {
        // Ошибка при получении данных
        snprintf(response, sizeof(response), "{\"error\": \"Failed to get sensor data\"}");
    }

    httpd_resp_set_type(req, "application/json");                            // Установка типа ответа
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");             // Разрешение всех источников
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET");          // Разрешение метода GET
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type"); // Разрешение заголовков
    httpd_resp_send(req, response, strlen(response));                        // Отправка ответа

    return ESP_OK; // Возврат успешного ответа
}

// Обработчик главной HTML страницы
esp_err_t index_html_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");                // Установка типа контента
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache"); // Отключение кэширования
    httpd_resp_send(req, index_html, strlen(index_html)); // Отправка HTML страницы
    return ESP_OK;
}

// Обработчик запроса на старт записи
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
        records_count = 0; // Сброс счетчика записей
        ESP_LOGI(TAG_WEB, "Recording started");
        httpd_resp_send(req, "Recording started", HTTPD_RESP_USE_STRLEN);
    }
    else
    {
        httpd_resp_send(req, "Already recording", HTTPD_RESP_USE_STRLEN);
    }
    return ESP_OK;
}

// Обработчик запроса на остановку записи и скачивание CSV
esp_err_t stop_recording_handler(httpd_req_t *req)
{
    if (is_recording)
    {
        is_recording = false;
        ESP_LOGI(TAG_WEB, "Recording stopped");

        // Формирование CSV данных
        // Заголовки
        size_t csv_size = 256; // Начальный размер
        char *csv_data = (char *)malloc(csv_size);
        if (!csv_data)
        {
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        strcpy(csv_data, "Timestamp,Temperature,Humidity,Voltage_ADC1,Voltage_ADC2\n");

        // Добавление записей
        for (size_t i = 0; i < records_count; i++)
        {
            // Оценка необходимого размера
            size_t needed = strlen(csv_data) + 100;
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

            // Добавление строки записи
            char line[100];
            snprintf(line, sizeof(line), "%lu,%.2f,%.2f,%.2f,%.2f\n",
                     records_buffer[i].timestamp,
                     records_buffer[i].temperature,
                     records_buffer[i].humidity,
                     records_buffer[i].voltage_0,
                     records_buffer[i].voltage_1);
            strcat(csv_data, line);
        }

        // Отправка CSV данных
        httpd_resp_set_type(req, "text/csv"); // Установка типа контента
        httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=data.csv"); // Заголовок для скачивания
        httpd_resp_send(req, csv_data, strlen(csv_data)); // Отправка данных

        free(csv_data); // Освобождение памяти
    }
    else
    {
        httpd_resp_send(req, "Not recording", HTTPD_RESP_USE_STRLEN);
    }
    return ESP_OK;
}

// Функция конфигурации и запуска веб-сервера
void start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG(); // Конфигурация по умолчанию

    // URI обработчик для главной страницы
    httpd_uri_t index_uri = {
        .uri = "/", // Путь
        .method = HTTP_GET,
        .handler = index_html_handler, // Обработчик
        .user_ctx = NULL};

    // URI обработчик для данных сенсоров
    httpd_uri_t sensor_data_uri = {
        .uri = "/data",
        .method = HTTP_GET,
        .handler = data_get_handler, // Обработчик
        .user_ctx = NULL};

    // URI обработчик для старта записи
    httpd_uri_t start_uri = {
        .uri = "/start",
        .method = HTTP_POST,
        .handler = start_recording_handler, // Обработчик
        .user_ctx = NULL};

    // URI обработчик для остановки записи
    httpd_uri_t stop_uri = {
        .uri = "/stop",
        .method = HTTP_POST,
        .handler = stop_recording_handler, // Обработчик
        .user_ctx = NULL};

    if (httpd_start(&server, &config) == ESP_OK) // Запуск сервера
    {
        httpd_register_uri_handler(server, &index_uri);       // Регистрация обработчика главной страницы
        httpd_register_uri_handler(server, &sensor_data_uri); // Регистрация обработчика данных
        httpd_register_uri_handler(server, &start_uri);       // Регистрация обработчика старта записи
        httpd_register_uri_handler(server, &stop_uri);        // Регистрация обработчика остановки записи
        ESP_LOGI(TAG_WEB, "Web server started");
    }
    else
    {
        ESP_LOGE(TAG_WEB, "Failed to start web server");
    }
}

// Задача для веб-сервера
void web_server_task(void *arg)
{
    start_webserver(); // Запуск веб-сервера

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(10000)); // Задержка для поддержания задачи
    }
}

// Обработчик событий Wi-Fi
static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect(); // Попытка подключения к Wi-Fi
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < MAX_RETRY)
        {
            esp_wifi_connect(); // Повтор подключения
            s_retry_num++;
            ESP_LOGI(TAG_WIFI, "Retry to connect to the AP");
        }
        else
        {
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT); // Сброс бита подключения
        }
        ESP_LOGI(TAG_WIFI, "Connect to the AP failed");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;       // Получение IP
        ESP_LOGI(TAG_WIFI, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip)); // Лог IP
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT); // Установка бита подключения
    }
}

// Инициализация Wi-Fi в режиме станции
void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate(); // Создание группы событий

    // Инициализация TCP/IP стека
    esp_netif_init();
    esp_event_loop_create_default();     // Создание дефолтного цикла событий
    esp_netif_create_default_wifi_sta(); // Создание дефолтного Wi-Fi клиента

    // Конфигурация Wi-Fi с учетными данными
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    // Настройка режима Wi-Fi на станцию
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS}};
    esp_wifi_set_mode(WIFI_MODE_STA);               // Установка режима станции
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config); // Установка конфигурации

    // Регистрация обработчиков событий
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id);  // Обработчик Wi-Fi событий
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip); // Обработчик получения IP

    esp_wifi_start(); // Запуск Wi-Fi

    // Ожидание подключения
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

// Функция конфигурации и запуска веб-сервера
void start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG(); // Конфигурация по умолчанию

    // URI обработчик для главной страницы
    httpd_uri_t index_uri = {
        .uri = "/", // Путь
        .method = HTTP_GET,
        .handler = index_html_handler, // Обработчик
        .user_ctx = NULL};

    // URI обработчик для данных сенсоров
    httpd_uri_t sensor_data_uri = {
        .uri = "/data",
        .method = HTTP_GET,
        .handler = data_get_handler, // Обработчик
        .user_ctx = NULL};

    // URI обработчик для старта записи
    httpd_uri_t start_uri = {
        .uri = "/start",
        .method = HTTP_POST,
        .handler = start_recording_handler, // Обработчик
        .user_ctx = NULL};

    // URI обработчик для остановки записи
    httpd_uri_t stop_uri = {
        .uri = "/stop",
        .method = HTTP_POST,
        .handler = stop_recording_handler, // Обработчик
        .user_ctx = NULL};

    if (httpd_start(&server, &config) == ESP_OK) // Запуск сервера
    {
        httpd_register_uri_handler(server, &index_uri);       // Регистрация обработчика главной страницы
        httpd_register_uri_handler(server, &sensor_data_uri); // Регистрация обработчика данных
        httpd_register_uri_handler(server, &start_uri);       // Регистрация обработчика старта записи
        httpd_register_uri_handler(server, &stop_uri);        // Регистрация обработчика остановки записи
        ESP_LOGI(TAG_WEB, "Web server started");
    }
    else
    {
        ESP_LOGE(TAG_WEB, "Failed to start web server");
    }
}

// Задача для веб-сервера
void web_server_task(void *arg)
{
    start_webserver(); // Запуск веб-сервера

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(10000)); // Задержка для поддержания задачи
    }
}

// Обработчик HTTP-запроса для получения данных
esp_err_t data_get_handler(httpd_req_t *req)
{
    sensor_data temp_data;
    adc_data adc_data_r;

    char response[256];

    // Получение данных из очередей
    if (xQueueReceive(temp_queue, &temp_data, pdMS_TO_TICKS(100)) == pdPASS &&
        xQueueReceive(temp_queue_adc, &adc_data_r, pdMS_TO_TICKS(100)) == pdPASS)
    {
        // Форматирование данных в JSON
        snprintf(response, sizeof(response),
                 "{\"temperature\": \"%.2f\", \"humidity\": \"%.2f\", \"voltage ADC1\": \"%.2f\", \"voltage ADC2\": \"%.2f\"}",
                 temp_data.temperature, temp_data.humidity, adc_data_r.voltage_0, adc_data_r.voltage_1);
    }
    else
    {
        // Ошибка при получении данных
        snprintf(response, sizeof(response), "{\"error\": \"Failed to get sensor data\"}");
    }

    httpd_resp_set_type(req, "application/json");                            // Установка типа ответа
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");             // Разрешение всех источников
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET");          // Разрешение метода GET
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type"); // Разрешение заголовков
    httpd_resp_send(req, response, strlen(response));                        // Отправка ответа

    return ESP_OK; // Возврат успешного ответа
}

// Обработчик главной HTML страницы
esp_err_t index_html_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");                // Установка типа контента
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache"); // Отключение кэширования
    httpd_resp_send(req, index_html, strlen(index_html)); // Отправка HTML страницы
    return ESP_OK;
}

// Обработчик запроса на старт записи
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
        records_count = 0; // Сброс счетчика записей
        ESP_LOGI(TAG_WEB, "Recording started");
        httpd_resp_send(req, "Recording started", HTTPD_RESP_USE_STRLEN);
    }
    else
    {
        httpd_resp_send(req, "Already recording", HTTPD_RESP_USE_STRLEN);
    }
    return ESP_OK;
}

// Обработчик запроса на остановку записи и скачивание CSV
esp_err_t stop_recording_handler(httpd_req_t *req)
{
    if (is_recording)
    {
        is_recording = false;
        ESP_LOGI(TAG_WEB, "Recording stopped");

        // Формирование CSV данных
        // Заголовки
        size_t csv_size = 256; // Начальный размер
        char *csv_data = (char *)malloc(csv_size);
        if (!csv_data)
        {
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        strcpy(csv_data, "Timestamp,Temperature,Humidity,Voltage_ADC1,Voltage_ADC2\n");

        // Добавление записей
        for (size_t i = 0; i < records_count; i++)
        {
            // Оценка необходимого размера
            size_t needed = strlen(csv_data) + 100;
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

            // Добавление строки записи
            char line[100];
            snprintf(line, sizeof(line), "%lu,%.2f,%.2f,%.2f,%.2f\n",
                     records_buffer[i].timestamp,
                     records_buffer[i].temperature,
                     records_buffer[i].humidity,
                     records_buffer[i].voltage_0,
                     records_buffer[i].voltage_1);
            strcat(csv_data, line);
        }

        // Отправка CSV данных
        httpd_resp_set_type(req, "text/csv"); // Установка типа контента
        httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=data.csv"); // Заголовок для скачивания
        httpd_resp_send(req, csv_data, strlen(csv_data)); // Отправка данных

        free(csv_data); // Освобождение памяти
    }
    else
    {
        httpd_resp_send(req, "Not recording", HTTPD_RESP_USE_STRLEN);
    }
    return ESP_OK;
}

// Функция конфигурации и запуска веб-сервера
void start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG(); // Конфигурация по умолчанию

    // URI обработчик для главной страницы
    httpd_uri_t index_uri = {
        .uri = "/", // Путь
        .method = HTTP_GET,
        .handler = index_html_handler, // Обработчик
        .user_ctx = NULL};

    // URI обработчик для данных сенсоров
    httpd_uri_t sensor_data_uri = {
        .uri = "/data",
        .method = HTTP_GET,
        .handler = data_get_handler, // Обработчик
        .user_ctx = NULL};

    // URI обработчик для старта записи
    httpd_uri_t start_uri = {
        .uri = "/start",
        .method = HTTP_POST,
        .handler = start_recording_handler, // Обработчик
        .user_ctx = NULL};

    // URI обработчик для остановки записи
    httpd_uri_t stop_uri = {
        .uri = "/stop",
        .method = HTTP_POST,
        .handler = stop_recording_handler, // Обработчик
        .user_ctx = NULL};

    if (httpd_start(&server, &config) == ESP_OK) // Запуск сервера
    {
        httpd_register_uri_handler(server, &index_uri);       // Регистрация обработчика главной страницы
        httpd_register_uri_handler(server, &sensor_data_uri); // Регистрация обработчика данных
        httpd_register_uri_handler(server, &start_uri);       // Регистрация обработчика старта записи
        httpd_register_uri_handler(server, &stop_uri);        // Регистрация обработчика остановки записи
        ESP_LOGI(TAG_WEB, "Web server started");
    }
    else
    {
        ESP_LOGE(TAG_WEB, "Failed to start web server");
    }
}

// Задача для веб-сервера
void web_server_task(void *arg)
{
    start_webserver(); // Запуск веб-сервера

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(10000)); // Задержка для поддержания задачи
    }
}

// Обработчик событий Wi-Fi
static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect(); // Попытка подключения к Wi-Fi
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < MAX_RETRY)
        {
            esp_wifi_connect(); // Повтор подключения
            s_retry_num++;
            ESP_LOGI(TAG_WIFI, "Retry to connect to the AP");
        }
        else
        {
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT); // Сброс бита подключения
        }
        ESP_LOGI(TAG_WIFI, "Connect to the AP failed");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;       // Получение IP
        ESP_LOGI(TAG_WIFI, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip)); // Лог IP
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT); // Установка бита подключения
    }
}

// Инициализация Wi-Fi в режиме станции
void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate(); // Создание группы событий

    // Инициализация TCP/IP стека
    esp_netif_init();
    esp_event_loop_create_default();     // Создание дефолтного цикла событий
    esp_netif_create_default_wifi_sta(); // Создание дефолтного Wi-Fi клиента

    // Конфигурация Wi-Fi с учетными данными
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    // Настройка режима Wi-Fi на станцию
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS}};
    esp_wifi_set_mode(WIFI_MODE_STA);               // Установка режима станции
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config); // Установка конфигурации

    // Регистрация обработчиков событий
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id);  // Обработчик Wi-Fi событий
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip); // Обработчик получения IP

    esp_wifi_start(); // Запуск Wi-Fi

    // Ожидание подключения
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

// Задача для записи данных из очередей в буфер
void record_data_task(void *arg)
{
    sensor_data temp_data;
    adc_data adc_data_r;

    while (1)
    {
        // Получение данных из очередей
        if (xQueueReceive(temp_queue, &temp_data, portMAX_DELAY) == pdPASS &&
            xQueueReceive(temp_queue_adc, &adc_data_r, portMAX_DELAY) == pdPASS)
        {
            // Добавление записи в буфер, если запись активна
            add_record(temp_data.temperature, temp_data.humidity, adc_data_r.voltage_0, adc_data_r.voltage_1);
        }

        // Минимальная задержка для предотвращения занятости CPU
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Основная функция
void app_main(void)
{
    // Инициализация NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Инициализация Wi-Fi
    wifi_init_sta();

    // Инициализация очередей
    temp_queue = xQueueCreate(10, sizeof(sensor_data));     // Очередь для температурных данных
    temp_queue_adc = xQueueCreate(10, sizeof(adc_data)); // Очередь для ADC данных

    // Проверка успешности создания очередей
    if (temp_queue == NULL || temp_queue_adc == NULL)
    {
        ESP_LOGE(TAG_WEB, "Failed to create queues");
        return;
    }

    // Инициализация мьютекса
    buffer_mutex = xSemaphoreCreateMutex();
    if (buffer_mutex == NULL)
    {
        ESP_LOGE(TAG_WEB, "Failed to create buffer mutex");
        return;
    }

    // Создание задач для чтения данных
    xTaskCreate(read_sensor_task, "read_sensor_task", 4096, NULL, 5, NULL);
    xTaskCreate(read_adc_task, "read_adc_task", 4096, NULL, 5, NULL);

    // Создание задачи для записи данных
    xTaskCreate(record_data_task, "record_data_task", 4096, NULL, 5, NULL);

    // Создание задачи для веб-сервера
    xTaskCreate(web_server_task, "web_server_task", 8192, NULL, 5, NULL);

    ESP_LOGI(TAG_WIFI, "System ready");
}

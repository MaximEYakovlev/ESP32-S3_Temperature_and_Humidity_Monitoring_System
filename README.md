# ESP32-S3 Temperature and Humidity Monitoring System

This project uses the **ESP32-S3** microcontroller to read temperature and humidity data from the **SHT40** sensor via the **I2C** interface. The data is then served through an embedded web server using **Wi-Fi**. The web server allows clients to access the sensor readings in real-time by querying a specific endpoint.

## Features

- **I2C Communication**: Interface with the SHT40 sensor to retrieve temperature and humidity data.
- **Wi-Fi Connectivity**: Connect the ESP32-S3 to a Wi-Fi network as a station (client).
- **HTTP Web Server**: Serve real-time temperature and humidity data through a web interface.
- **FreeRTOS Integration**: Leverages FreeRTOS tasks and queues for parallel operations.

## Components

- **ESP32-S3**: Microcontroller for running the program.
- **SHT40 Sensor**: Measures temperature and humidity.
- **FreeRTOS**: Handles concurrent tasks such as reading sensor data and serving HTTP requests.
- **I2C**: Communication protocol to interface with the sensor.
- **Wi-Fi**: For network connectivity.
  
## Requirements

- **ESP-IDF (v4.x or higher)**: The official Espressif IoT Development Framework.
- **SHT40 Sensor**: Temperature and humidity sensor from Sensirion.
- **ESP32-S3**: Microcontroller with Wi-Fi capabilities.

## Hardware Setup

- **I2C SDA** pin connected to GPIO 3 on the ESP32-S3.
- **I2C SCL** pin connected to GPIO 2 on the ESP32-S3.
- Power and ground connections for the SHT40 sensor.
- A Wi-Fi network for ESP32-S3 to connect to.

## Installation

1. Clone the repository or copy the program files into your ESP-IDF project structure.
2. Set up your ESP-IDF environment. Refer to the [ESP-IDF setup guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html).
3. Modify the Wi-Fi SSID and password in the program code:
    ```c
    #define WIFI_SSID "your_ssid"
    #define WIFI_PASS "your_password"
    ```
4. Build the project using ESP-IDF:
    ```bash
    idf.py build
    ```
5. Flash the program onto the ESP32-S3:
    ```bash
    idf.py -p /dev/ttyUSB0 flash
    ```
6. Monitor the output:
    ```bash
    idf.py monitor
    ```

## Usage

Once the ESP32-S3 is connected to the Wi-Fi network, it will start reading temperature and humidity data from the SHT40 sensor and serve it via an HTTP web server.

### Accessing Sensor Data

- The web server will be running at the ESP32-S3's assigned IP address.
- Navigate to `http://<ESP32_IP>/sensor_data` to retrieve the current temperature and humidity in JSON format:
    ```json
    {
      "temperature": "24.50",
      "humidity": "45.60"
    }
    ```

### Example

If the IP address of the ESP32-S3 is `192.168.1.100`, you can get the sensor data by visiting:
[http://192.168.1.100/sensor_data](http://192.168.1.100/sensor_data)


## FreeRTOS Tasks

- **Sensor Task**: Periodically reads temperature and humidity from the SHT40 sensor and sends the data to a FreeRTOS queue.
- **Web Server Task**: Handles HTTP requests and serves the sensor data by retrieving it from the queue.

## License

This project is licensed under the MIT License.


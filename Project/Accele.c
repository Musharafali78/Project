#include "stdio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"

#define SPI_CLK_PIN 18       // SPI Clock Pin (SCL)
#define SPI_MISO_PIN 19      // SPI MISO Pin (SDO)
#define SPI_MOSI_PIN 23      // SPI MOSI Pin (SDA)
#define SPI_CS_PIN 5         // SPI Chip Select (CS)

// MPU9250 Register Addresses
#define ACCEL_XOUT_H 0x3B   // Accelerometer X High byte
#define ACCEL_XOUT_L 0x3C   // Accelerometer X Low byte
#define ACCEL_YOUT_H 0x3D   // Accelerometer Y High byte
#define ACCEL_YOUT_L 0x3E   // Accelerometer Y Low byte
#define ACCEL_ZOUT_H 0x3F   // Accelerometer Z High byte
#define ACCEL_ZOUT_L 0x40   // Accelerometer Z Low byte

spi_device_handle_t spi;

// SPI Initialization Function
esp_err_t spi_init() {
    esp_err_t ret;

    spi_bus_config_t bus_cfg = {
        .miso_io_num = SPI_MISO_PIN,
        .mosi_io_num = SPI_MOSI_PIN,
        .sclk_io_num = SPI_CLK_PIN,
        .max_transfer_sz = 64,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };
    ret = spi_bus_initialize(HSPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE("SPI", "Failed to initialize SPI bus");
        return ret;
    }

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 1000000,   // 1 MHz SPI clock
        .mode = 0,                   // SPI mode 0
        .spics_io_num = SPI_CS_PIN,   // Chip select pin
        .queue_size = 7,
        .flags = 0
    };
    ret = spi_bus_add_device(HSPI_HOST, &dev_cfg, &spi);
    if (ret != ESP_OK) {
        ESP_LOGE("SPI", "Failed to add device to SPI bus");
        return ret;
    }

    return ret;
}

// SPI Read Register Function
void spi_read_register(uint8_t reg_addr, uint8_t *data, uint8_t length) {
   // memset(&t, 0, sizeof(t));
    int16_t ax,ay,az,tmp,gyro;

    uint8_t rx_data[7] = {0};

    spi_transaction_t t = {
        .length = length * 8,        // Include register address byte
        .tx_buffer = data,
        .rx_buffer = rx_data,
    };
    spi_device_transmit(spi, &t);

    ax = ((int16_t)rx_data[1] << 8) | rx_data[2];
    ay = ((int16_t)rx_data[3] << 8) | rx_data[4];
    az = ((int16_t)rx_data[5] << 8) | rx_data[6];
    //tmp = ((int16_t)rx_data[7] << 8) | rx_data[8];
    //gyro = ((int16_t)rx_data[9] << 8) | rx_data[10];

    float data_x_axix = ax / 16384.0; 
    float data_y_axix = ay / 16384.0; 
    float data_z_axix = az / 16384.0; 

    //float Temperature = ((tmp/333.87)+21);
    //float Gyroscope = gyro / 131;

    printf("Value of X_Axis:%f\n",data_x_axix);
    printf("Value of y_Axis:%f\n",data_y_axix);
    printf("Value of z_Axis:%f\n",data_z_axix);
    printf("################################ \n");

    //printf("Temperatur:: %0.2f\n",Temperature);
    //printf("Gyroscope: %.2f\n",Gyroscope);

}

// Function to Read Accelerometer Data
void read_accel_data(uint8_t reg_addr) {
    uint8_t accel_data[7] = {reg_addr | 0x80};

    spi_read_register(ACCEL_XOUT_H, accel_data,7);
    
}

void app_main(void) {
    esp_err_t ret = spi_init();
    if (ret != ESP_OK) {
        ESP_LOGE("SPI", "SPI initialization failed");
        return;
    }

    while (1) {
        read_accel_data(0x3B);
        vTaskDelay(500/ portTICK_PERIOD_MS);
    }
}


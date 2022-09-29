#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "bmp280.h"

#define I2C_MASTER_SCL_IO           22      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          1000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define BMP280_SENSOR_ADDR               0x76        /*!< Slave address of the MPU9250 sensor */
#define BMP280_SENSOR_ADDR1              0x77        /*!< Slave address of the MPU9250 sensor */
#define BMP280_WHO_AM_I_REG_ADDR           0xD0        /*!< Register addresses of the "who am I" register */

#define BMP280_PWR_MGMT_1_REG_ADDR        0xE0        /*!< Register addresses of the power managment register */
#define BMP280_RESET_BIT                  0xB6

void delay_ms(uint32_t period_ms);
int8_t i2c_reg_write_addr0(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t i2c_reg_read_addr0(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t i2c_reg_write_addr1(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t i2c_reg_read_addr1(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t spi_reg_write(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t spi_reg_read(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
void print_rslt(const char api_name[], int8_t rslt, char *tag);
static esp_err_t i2c_master_init(void);
int getDelay(uint8_t numero_tarefa);

TaskHandle_t th_pressure;
TaskHandle_t th_temperature;
TaskHandle_t th_send_data;

SemaphoreHandle_t sh_pressure = NULL;
SemaphoreHandle_t sh_temperature = NULL;

float var_temperature = 0.0;
float var_pressure = 0.0;

SemaphoreHandle_t getSemaphore(uint8_t numero_tarefa) {
    switch (numero_tarefa){
        case 0: 
            return sh_pressure;
        case 1: 
            return sh_temperature;
    };
    return 0; //olhar com carinho pra isso
}

int getDelay(uint8_t numero_tarefa) {
    switch (numero_tarefa){
        case 0: 
            return 1000;
        case 1: 
            return 500;
    };
    return 0; //olhar com carinho pra isso
}

TaskHandle_t getTask(uint8_t numero_tarefa) {
    switch (numero_tarefa){
        case 0: 
            return th_pressure;
        case 1: 
            return th_temperature;
    };
    return 0; //olhar com carinho pra isso
}

//TODO: Analisar a possibilidade de não repetir todo o código da lógica de cada task (Pressure, Temperature, SendData)
//Método genérico que eu e a Tabata estávamos tentando;
//Tentativa
void Bmp280Task(void *parametros)
{
    int numero_tarefa = (uint8_t)parametros;
    printf("Inicio da tarefa %d\n", numero_tarefa);

    static char *tag = "";

    uint8_t data[2];
    int8_t rslt;
    struct bmp280_dev bmp;
    struct bmp280_config conf;
    struct bmp280_uncomp_data ucomp_data;
    double temp;

    bmp.delay_ms = delay_ms;
    bmp.intf = BMP280_I2C_INTF;
    switch (numero_tarefa) {
        case 0:
            bmp.read = i2c_reg_read_addr0;
            bmp.write = i2c_reg_write_addr0;
            bmp.dev_id = BMP280_I2C_ADDR_PRIM;
            break;
        case 1:
            bmp.read = i2c_reg_read_addr1;
            bmp.write = i2c_reg_write_addr1;
            bmp.dev_id = BMP280_I2C_ADDR_SEC;
            break;
        default:
            return;
    }

    rslt = bmp280_init(&bmp);
    print_rslt(" bmp280_init status", rslt, tag);

    /* Always read the current settings before writing, especially when
     * all the configuration is not modified
     */
    rslt = bmp280_get_config(&conf, &bmp);
    print_rslt(" bmp280_get_config status", rslt, tag);

    /* configuring the temperature oversampling, filter coefficient and output data rate */
    /* Overwrite the desired settings */
    conf.filter = BMP280_FILTER_COEFF_2;

    /* Temperature oversampling set at 4x */
    conf.os_temp = BMP280_OS_2X;

    /* Setting the output data rate as 1HZ(1000ms) */
    conf.odr = BMP280_ODR_1000_MS;
    rslt = bmp280_set_config(&conf, &bmp);
    print_rslt(" bmp280_set_config status", rslt, tag);

    /* Always set the power mode after setting the configuration */
    rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);
    print_rslt(" bmp280_set_power_mode status", rslt, tag);
    
    for (;;)
    {
        if (getSemaphore(numero_tarefa) != NULL)
        {
            if (xSemaphoreTake(getSemaphore(numero_tarefa), 10))
            {
                rslt = bmp280_get_uncomp_data(&ucomp_data, &bmp);
                rslt = bmp280_get_comp_temp_double(&temp, ucomp_data.uncomp_temp, &bmp);
                ESP_LOGI(tag, "Task %d - %f\r", numero_tarefa, temp);
                xSemaphoreGive(getSemaphore(numero_tarefa));
                bmp.delay_ms(getDelay(numero_tarefa));
            }
            else
            {
                printf("Tarefa pressão nao pegou a sinaleira\n");
            }
        }
    }
    vTaskDelete(getTask(numero_tarefa));
}
void app_main(void)
{
    sh_pressure = xSemaphoreCreateBinary();
    xSemaphoreGive(sh_pressure);

    sh_temperature = xSemaphoreCreateBinary();
    xSemaphoreGive(sh_temperature);

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI("", "I2C initialized successfully");
    //Temperature and Pressure Task
    xTaskCreate(Bmp280Task, "tarefaTemperature", 2048, (void *)0, 0, &th_temperature);
    xTaskCreate(Bmp280Task, "tarefaPressao", 2048, (void *)1, 0, &th_pressure);
}

void delay_ms(uint32_t period_ms)
{
    const TickType_t xDelay = period_ms / portTICK_PERIOD_MS;
    vTaskDelay(xDelay);
}

int8_t i2c_reg_write_addr0(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
    uint8_t write_buf[2] = {reg_addr, *reg_data};

    return i2c_master_write_to_device(I2C_MASTER_NUM, BMP280_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

int8_t i2c_reg_read_addr0(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, BMP280_SENSOR_ADDR, &reg_addr, 1, reg_data, length, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

int8_t i2c_reg_write_addr1(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
    uint8_t write_buf[2] = {reg_addr, *reg_data};

    return i2c_master_write_to_device(I2C_MASTER_NUM, BMP280_I2C_ADDR_SEC, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

int8_t i2c_reg_read_addr1(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, BMP280_I2C_ADDR_SEC, &reg_addr, 1, reg_data, length, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

void print_rslt(const char api_name[], int8_t rslt, char *tag)
{
    if (rslt != BMP280_OK)
    {
        printf("%s\t", api_name);
        if (rslt == BMP280_E_NULL_PTR)
        {
           ESP_LOGI(tag, "Error [%d] : Null pointer error\r\n", rslt);
        }
        else if (rslt == BMP280_E_COMM_FAIL)
        {
            ESP_LOGI(tag, "Error [%d] : Bus communication failed\r\n", rslt);
        }
        else if (rslt == BMP280_E_IMPLAUS_TEMP)
        {
            ESP_LOGI(tag, "Error [%d] : Invalid Temperature\r\n", rslt);
        }
        else if (rslt == BMP280_E_DEV_NOT_FOUND)
        {
            ESP_LOGI(tag, "Error [%d] : Device not found\r\n", rslt);
        }
        else
        {
           ESP_LOGI(tag, "Error [%d] : Unknown error code\r\n", rslt);
        }
    }
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

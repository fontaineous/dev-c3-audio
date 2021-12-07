#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"




// Board Specifics
#define I2C_MASTER_NUM 0
#define I2C_MASTER_SDA_IO 0
#define I2C_MASTER_SCL_IO 1
#define I2C_MASTER_FREQ_HZ I2C_CLK_FREQ_MAX        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define I2C_CLK_LIMIT_RTC                 (20 * 1000 * 1000 / 20)   /*!< Limited by RTC, no more than RTC/20*/


// General I2C
#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

// SGTL5000 specifics 
#define SGTL5000_I2C_ADDR_CS_LOW 0x0A  // CTRL_ADR0_CS pin low (normal configuration)
#define SGTL5000_I2C_ADDR_CS_HIGH 0x2A // CTRL_ADR0_CS  pin high


#define SGTL5000_CHIP_ID_OS 0x0000
// 15:8 PARTID		0xA0 - 8 bit identifier for SGTL5000
// 7:0  REVID		0x00 - revision number for SGTL5000.


/**
 * @brief 
 * 
 */
/*
static esp_err_t i2c_slave_init(void)
{
    int i2c_slave_port = I2C_SLAVE_NUM;
    i2c_config_t conf_slave = {
        .sda_io_num = I2C_SLAVE_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SLAVE_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .mode = I2C_MODE_SLAVE,
        .slave.addr_10bit_en = 0,
        .slave.slave_addr = ESP_SLAVE_ADDR,
    };
    esp_err_t err = i2c_param_config(i2c_slave_port, &conf_slave);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(i2c_slave_port, conf_slave.mode, I2C_SLAVE_RX_BUF_LEN, I2C_SLAVE_TX_BUF_LEN, 0);
}
*/


/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        //.master.clk_speed = I2C_MASTER_FREQ_HZ,
        .master.clk_speed = I2C_CLK_LIMIT_RTC,
        .clk_flags = I2C_SCLK_SRC_FLAG_AWARE_DFS,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };

    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

    
void app_main(void)
{
    // init i2c
    ESP_ERROR_CHECK(i2c_master_init());

    //size_t size = 1;
    i2c_port_t i2c_num = I2C_MASTER_NUM;
    uint8_t data_h, data_l;
    uint8_t data_h1, data_l1;

    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SGTL5000_I2C_ADDR_CS_LOW << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, SGTL5000_CHIP_ID_OS, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, SGTL5000_CHIP_ID_OS, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SGTL5000_I2C_ADDR_CS_LOW << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &data_h, ACK_VAL);
    i2c_master_read_byte(cmd, &data_l, NACK_VAL);
    //i2c_master_read_byte(cmd, &data_h1, ACK_VAL);
    //i2c_master_read_byte(cmd, &data_l1, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    /*
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SGTL5000_I2C_ADDR_CS_LOW << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, SGTL5000_CHIP_ID_OS, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, SGTL5000_CHIP_ID_OS, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    //if (ret != ESP_OK) {
        //return ret;
    //}
    vTaskDelay(30 / portTICK_RATE_MS);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SGTL5000_I2C_ADDR_CS_LOW << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &data_h, ACK_VAL);
    i2c_master_read_byte(cmd, &data_l, NACK_VAL);
    i2c_master_read_byte(cmd, &data_h1, ACK_VAL);
    i2c_master_read_byte(cmd, &data_l1, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    */

    while (1) {
           printf("hellosss !!\n"); 
           printf("ret:  %x\n", ret);
           printf("i2c data:  %x%x\n", data_h, data_l);
           //printf("i2c data:  %x%x\n", data_h1, data_l1);
           vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

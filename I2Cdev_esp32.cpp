// I2Cdev library collection - Main I2C device class
// Abstracts bit and byte I2C R/W functions into a convenient class
// ESP32 port by Enes Abdulhalik
// Based on Arduino's I2Cdev by Jeff Rowberg <jeff@rowberg.net>
//
// Changelog:
//      2023-08-01 - Initial release


/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2015 Jeff Rowberg, Nicolas Baldeck

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include <esp_log.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "sdkconfig.h"

#include "I2Cdev.h"

#define I2C_NUM I2C_NUM_0

#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)   do { esp_err_t rc = (x); if (rc != ESP_OK) { ESP_LOGE("err", "esp_err_t = %d", rc); /*assert(0 && #x);*/} } while(0);

/** Default constructor.
 */
I2Cdev::I2Cdev(uint8_t devAddr, gpio_num_t sda_pin, gpio_num_t scl_pin, i2c_mode_t mode, uint32_t freq) 
	: m_devAddr(devAddr), m_sda_pin(sda_pin), m_scl_pin(scl_pin), m_mode(mode), m_freq(freq) 
{
	i2c_config_t conf;
	conf.mode = m_mode;
	conf.sda_io_num = m_sda_pin;
	conf.scl_io_num = m_scl_pin;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = m_freq;
    conf.clk_flags = 0;

	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, m_mode, 0, 0, 0));
}

bool I2Cdev::read_bit(uint8_t regAddr, uint8_t bitNum, uint8_t *data) {
	uint8_t b;
    uint8_t count = read_byte(regAddr, &b);

	if(count)
    	*data = b & (1 << bitNum);
    
	return count;
}

bool I2Cdev::read_bits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data) {
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t count, b;
    if ((count = read_byte(regAddr, &b)) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *data = b;
    }
    return count;
}

bool I2Cdev::read_byte(uint8_t regAddr, uint8_t *data) {
    return read_bytes(regAddr, 1, data);
}


uint8_t I2Cdev::read_bytes(uint8_t regAddr, uint8_t length, uint8_t *data) {
	i2c_cmd_handle_t cmd;
	select_reg(regAddr);

	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (m_devAddr << 1) | I2C_MASTER_READ, 1));

	if(length>1)
		ESP_ERROR_CHECK(i2c_master_read(cmd, data, length-1, I2C_MASTER_ACK));

	ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+length-1, I2C_MASTER_NACK));

	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM, cmd, 1000/portTICK_PERIOD_MS));
	i2c_cmd_link_delete(cmd);

	return length;
}

void I2Cdev::select_reg(uint8_t regAddr) {
	i2c_cmd_handle_t cmd;

	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (m_devAddr << 1) | I2C_MASTER_WRITE, 1));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, regAddr, 1));
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM, cmd, 1000/portTICK_PERIOD_MS));
	i2c_cmd_link_delete(cmd);
}

/** write a single bit in an 8-bit device register.
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::write_bit(uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t b;
    read_byte(regAddr, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return write_byte(regAddr, b);
}

/** Write multiple bits in an 8-bit device register.
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::write_bits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t b = 0;
    if (read_byte(regAddr, &b) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        b &= ~(mask); // zero all important bits in existing byte
        b |= data; // combine data with existing byte
        return write_byte(regAddr, b);
    } else {
        return false;
    }
}

/** Write single byte to an 8-bit device register.
 * @param regAddr Register address to write to
 * @param data New byte value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::write_byte(uint8_t regAddr, uint8_t data) {
	i2c_cmd_handle_t cmd;

	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (m_devAddr << 1) | I2C_MASTER_WRITE, 1));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, regAddr, 1));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, data, 1));
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM, cmd, 1000/portTICK_PERIOD_MS));
	i2c_cmd_link_delete(cmd);

	return true;
}

/** Write single byte to an 8-bit device register.
 * @param regAddr Register address to write to
 * @param length Number of bytes to write
 * @param data Array of bytes to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::write_bytes(uint8_t regAddr, uint8_t length, uint8_t *data){
	i2c_cmd_handle_t cmd;

	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (m_devAddr << 1) | I2C_MASTER_WRITE, 1));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, regAddr, 1));
	ESP_ERROR_CHECK(i2c_master_write(cmd, data, length-1, 0));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, data[length-1], 1));
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM, cmd, 1000/portTICK_PERIOD_MS));
	i2c_cmd_link_delete(cmd);
	return true;
}
// I2Cdev library collection - Main I2C device class
// Abstracts bit and byte I2C R/W functions into a convenient class
// Linux and ESP32 port by Enes Abdulhalik
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

#ifndef I2CDEV_H
#define I2CDEV_H

#ifdef ESP_PLATFORM
#include <driver/i2c.h>
#else
#include <stdint.h>
#endif

class I2Cdev
{
public:
#ifdef ESP_PLATFORM
    I2Cdev(uint8_t devAddr, gpio_num_t sda_pin, gpio_num_t scl_pin, i2c_mode_t mode, uint32_t freq);
    void select_reg(uint8_t regAddr);
#else
    I2Cdev(int bus, int addr);
#endif

    ~I2Cdev();

    /**
     * @brief Read a single bit from an 8-bit device register. 
     * All of the bits in the result other than the one 
     * scanned will be 0.
     * 
     * @param reg_addr 8-bit address of the register in the i2c device.
     * @param bit_num Bit position to read (0-7).
     * @param data Container for single bit value.
     * @return Status of read operation (true = success).
     * 
     * @note Bit numbers: 76543210
     */
    bool read_bit(uint8_t regAddr, uint8_t bitNum, uint8_t *data);

    /**  
     * @brief Read multiple bits from an 8-bit device register.
     * 
     * @param reg_addr Register reg_addr to read from.
     * @param bitStart First bit position to read (0-7).
     * @param length Number of bits to read (not more than 8).
     * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05).
     * @return Status of read operation (true = success).
     * 
     * @note Bit numbers: 76543210
     */
    bool read_bits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);

    /**
     * @brief Read single byte from an 8-bit device register.
     * 
     * @param reg_addr Register reg_addr to read from.
     * @param data Container for byte value read from device.
     * @return Status of read operation (true = success).
     */
    bool read_byte(uint8_t regAddr, uint8_t *data);

    /**
     * @brief Read multiple bytes from an 8-bit device register.
     * 
     * @param reg_addr First register reg_addr to read from.
     * @param length Number of bytes to read.
     * @param data Buffer to store read data in.
     * @return Numbers of bytes received.
     */
    uint8_t read_bytes(uint8_t regAddr, uint8_t length, uint8_t *data);

    /** 
     * @brief write a single bit in an 8-bit device register.
     * 
     * @param reg_addr Register reg_addr to write to.
     * @param bit_num Bit position to write (0-7).
     * @param value New bit value to write.
     * @return Status of operation (true = success).
     * 
     * @note Bit numbers: 76543210
     */
    bool write_bit(uint8_t regAddr, uint8_t bitNum, uint8_t data);

    /**
     * @brief Write multiple bits in an 8-bit device register.
     * 
     * @param reg_addr Register reg_addr to write to.
     * @param bitStart First bit position to write (0-7).
     * @param length Number of bits to write (not more than 8).
     * @param data Right-aligned value to write.
     * @return Status of operation (true = success).
     * 
     * @note Bit numbers: 76543210
     */
    bool write_bits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);

    /** 
     * @brief Write single byte directly to the device.
     * 
     * @param data New byte value to write.
     * @return Status of operation (true = success).
     */
    bool write_byte(uint8_t data);

    /** 
     * @brief Write single byte to an 8-bit device register.
     * 
     * @param reg_addr Register address to write to.
     * @param data New byte value to write.
     * @return Status of operation (true = success).
     */
    bool write_byte(uint8_t regAddr, uint8_t data);

    /** 
     * @brief Write single byte to an 8-bit device register.
     * 
     * @param reg_addr Register address to write to.
     * @param length Number of bytes to write.
     * @param data Array of bytes to write.
     * @return Status of operation (true = success).
     */
    bool write_bytes(uint8_t regAddr, uint8_t length, uint8_t *data);


protected:

#ifdef ESP_PLATFORM
    uint8_t m_devAddr;
    gpio_num_t m_sda_pin;
    gpio_num_t m_scl_pin;
    i2c_mode_t m_mode;
    uint32_t m_freq;
#else
    int fd; // The file descriptor of the I2C bus.
#endif
};


#endif
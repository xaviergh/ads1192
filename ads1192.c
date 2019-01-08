/*! 
* @file ads1192.h
*
* @brief Library for the TI ADS1192 with SPI interface.
*
* @author Xavier Garcia Herrera.
*
* @par
* Version 1.0 2014
*/
#include "ads1192.h"

/*!
*\fn void ads1192_spi_init (void)
* @brief Init microcontroller SPI.
* @return nothing.
*/
void
ads1192_spi_init (void)
{
    //For 2.7 < DVDD < 3.6 max speed 20MHz
    //For 1.6 < DVDD < 2.7 max speed 15MHz
    //<--- SPI INIT --->
    //add SPI init code here
}

/*!
*\fn void ads1192_init (ads1192_mode_t ads1192_mode)
* @brief Init ads1192 with the specified ads1192_mode_t.
* @return nothing.
*/
void
ads1192_init (ads1192_mode_t ads1192_mode)
{
    //CLKSEL is tied to VDD to use internal clock
    /*
    pone CS en alto
    dependiendo del modo escribe en los registros el modo
    TODO: add config for sample rate, PGA and decimation 
    */
    ADS1192_CS = 1;
    ADS1192_START = 0;

    ads1192_spi_init();

    spi_write(ADS1192_CMD_SDATAC); //Stop Read Data Continuous to config registers
}

/*!
*\fn uint16_t ads1192_read_reg (uint8_t ads1192_reg)
* @brief Read ads1192 register.
* @return 16 bit register value.
*/
uint16_t
ads1192_read_reg (uint8_t ads1192_reg)
{
    //for multibyte commands there have to be a delay
    //between bytes of 4*tCLK, CLK being the ads clock
    //for the internal clock, 512Khz
    //there should be a delay of appx 8uS
}

/*!
*\fn void ads1192_write_reg (uint8_t ads1192_reg, uint8_t ads1192_reg_value)
* @brief write the desired value to a ads1192 register.
* @param[in] uint8_t 12bit DAC value.
* @param[in] uint8_t 12bit DAC value.
* @return nothing.
*/
void
ads1192_write_reg (uint8_t ads1192_reg, uint8_t ads1192_reg_value)
{
    //for multibyte commands there have to be a delay
    //between bytes of 4*tCLK, CLK being the ads clock
    //for the internal clock, 512Khz
    //there should be a delay of appx 8uS

    dac_value = dac_value && 0x0FFF; //clean the 12bit part and set normal mode 0b00

    i2c_start(); 
    i2c_write(SLAVE_ADDR | SLAVE_WRITE);
    i2c_write((dac_value >> 8)); //msb
    i2c_write(dac_value); //lsb
    i2c_stop();
}

/*!
*\fn void dac121c085_burst_write (uint16_t dac_value)
* @brief Write continuous 12bit value to the DAC in normal mode.
* @param[in] dac_value pointer to the 12bit values to write.
* @param[in] len length of the array to the dac_value.
* @return nothing.
*/
void
ads1192_burst_read (uint8_t ads1192_reg, uint8_t *ads1192_reg_value, uint8_t len);
{
    //TODO
}

/*!
*\fn void dac121c085_burst_write (uint16_t dac_value)
* @brief Write continuous 12bit value to the DAC in normal mode.
* @param[in] dac_value pointer to the 12bit values to write.
* @param[in] len length of the array to the dac_value.
* @return nothing.
*/
void 
ads1192_burst_write (uint16_t *ads1192_reg_value, uint8_t len)
{
    //TODO
}


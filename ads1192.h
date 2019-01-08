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
#ifndef _LIBADS1192_H
#define _LIBADS1192_H

#include <stdint.h> //generic typedefs
#include <stdbool.h> //bool typedef

#ifdef __XC8
#define spi_write 	SPI1_Exchange8bit
#define spi_read	SPI1_Exchange8bit
#define delay_ms	__delay_ms
#endif

/*! @name ADS1192 defines
** @{
*/
/*!
	Pin definitions (assign the associated pins)
*/
#define ADS1192_DIN	    SDO1_LAT	   /*!< ADS1192 Data Input - SPI MOSI */
#define ADS1192_SCLK	SCK1_LAT	   /*!< ADS1192 Clock Input - SPI SCLK */
#define ADS1192_DOUT	P1A_LAT        /*!< ADS1192 Data Output - SPI MISO */
#define ADS1192_CS		TLC_XLAT_LAT   /*!< ADS1192 Chip Select Pin */
#define ADS1192_DRDY	TLC_MODE_LAT   /*!< ADS1192  Data Ready Pin - active low */
#define ADS1192_START	TLC_BLANK_LAT  /*!< ADS1192 Start Pin - Low to High to start conversion */
/** @} */

/*!
	ADS1192 modes
	TODO: Add ECG related modes
*/
typedef enum
{
	ADS1192_IN1_IN2_NO_ECG_MODE, /*!< IN1 and IN2 normal mode with no ECG features */
	ADS1192_IN3_IN2_NO_ECG_MODE, /*!< IN3 in IN1 and IN2 normal mode with no ECG features */
	ADS1192_IN1_IN3_NO_ECG_MODE, /*!< IN1 and IN3 in IN2 normal mode with no ECG features */
	ADS1192_IN1_ONLY_NO_ECG_MODE, /*!< IN1 only normal mode with no ECG features */
	ADS1192_IN2_ONLY_NO_ECG_MODE, /*!< IN2 only normal mode with no ECG features */
	ADS1192_IN3_IN1_ONLY_NO_ECG_MODE, /*!< IN3 in IN1 only normal mode with no ECG features */
	ADS1192_IN3_IN2_ONLY_NO_ECG_MODE /*!< IN3 in IN2 only normal mode with no ECG features */

} ads1192_mode_t;

/*! @name ADS1192 SPI Command definitions
** @{
*/
/*!
	System Commands
*/
#define ADS1192_CMD_WAKEUP   	0x02 /*!< Wake-up from standby mode */
#define ADS1192_CMD_STANDBY  	0x04 /*!< Enter standby mode */
#define ADS1192_CMD_RESET	 	0x06 /*!< Reset the device */
#define ADS1192_CMD_START   	0x08 /*!< Start/restart (synchronize) conversions */
#define ADS1192_CMD_STOP    	0x0A /*!< Stop conversion */
#define ADS1192_CMD_OFFSETCAL   0x1A /*!< Channel offset calibration */
/*!
	Data Read Commands
*/
#define ADS1192_CMD_RDATAC   	0x10 /*!< Enable Read Data Continuous mode */
									 /*!< When in RDATAC mode RREG command is ignored */
#define ADS1192_CMD_SDATAC  	0x11 /*!< Stop Read Data Continuously mode */
#define ADS1192_CMD_RDATA	 	0x12 /*!< Read data by command; supports multiple read back. */
/*!
	Register Read Commands
*/
#define ADS1192_CMD_RREG_0   	0x20 /*!< Read register starting address */
#define ADS1192_CMD_RREG_1   	0x00 /*!< Number of registers to read */
#define ADS1192_CMD_WREG_0  	0x40 /*!< Write register starting address */
#define ADS1192_CMD_WREG_1  	0x00 /*!< Number of registers to write */
/** @} */

/*! @name ADS1192 Register Map
** @{
*/
/*!
	Device Settings Registers (read-only)
*/
#define ADS1192_REG_ID   	0x00 /*!< ID Control Register. Default (RESET): 0xXX */
								 /*!< | REV_ID7 | REV_ID6 | REV_ID5 | 1 | 0 | 0 | REV_ID1 | REV_ID0 | */
/*!
	Goblal Setting Across Channels
*/
#define ADS1192_REG_CONFIG1	0x01 /*!< Configuration Register 1. Default (RESET) 0x02 */								 
								 /*!< | SINGLE_SHOT | 0 | 0 | 0 | 0 | DR2 | DR1 | DR0 | */
#define ADS1192_REG_CONFIG2 0x02 /*!< Configuration Register 2. Default (RESET) 0x80 */
								 /*!< | 1 | PDB_LOFF_COMP | PDB_REFBUF | VREF_4V | CLK_EN | 0 | INT_TEST | TEST_FREQ | */
#define ADS1192_REG_LOFF   	0x03 /*!< Lead-Off Control Register. Default (RESET) 0x10 */
								 /*!< | COMP_TH2 | COMP_TH1 | COMP_TH0 | 1 | ILEAD_OFF1 | ILEAD_OFF0 | 0 | FLEAD_OFF | */
/*!
	Channel-Specific Settings
*/
#define ADS1192_REG_CH1SET   	0x04 /*!< Channel 1 Settings. Default (RESET) 0x00 */
									 /*!< | PD1 | GAIN1_2 | GAIN1_1 | GAIN1_0 | MUX1_3 | MUX1_2 | MUX1_1 | MUX1_0 | */
#define ADS1192_REG_CH2SET   	0x05 /*!< Channel 2 Settings. Default (RESET) 0x00 */
									 /*!< | PD2 | GAIN2_2 | GAIN2_1 | GAIN2_0 | MUX2_3 | MUX2_2 | MUX2_1 | MUX2_0 | */
#define ADS1192_REG_RLD_SENS   	0x06 /*!< Right Leg Drive Sense Selection. Default (RESET) 0x00 */
									 /*!< | 0 | 0 | PDB_RLD | RLD_LOFF_SENS | RLD2N | RLD2P | RLD1N | RLD1P | */
#define ADS1192_REG_LOFF_SENS  	0x07 /*!< Lead-Off Sense Selection. Default (RESET) 0x00 */
									 /*!< | 0 | 0 | FLIP2 | FLIP1 | LOFF2N | LOFF2P | LOFF1N | LOFF1P | */
#define ADS1192_REG_LOFF_STAT  	0x08 /*!< Lead-Off Status. Default (RESET) 0x00 */
									 /*!< | 0 | CLK_DIV | 0 | RLD_STAT (RO) | IN2N_OFF | IN2P_OFF | IN1N_OFF | IN1P_OFF | */
/*!
	GPIO and Other Registers
*/
#define ADS1192_REG_MISC1   	0x10 /*!< Miscellaneous Control Register 1. Default (RESET) 0x00 */
									 /*!< | 0 | 0 | 0 | 0 | 0 | 0 | 1 | 0 | */
#define ADS1192_REG_MISC2   	0x10 /*!< Miscellaneous Control Register 2. Default (RESET) 0x02 */
									 /*!< | CALIB_ON | 0 | 0 | 0 | 0 | 0 | RLDREF_INT | 0 | */
#define ADS1192_REG_GPIO	   	0x10 /*!< General-Purpose I/O Register. Default (RESET) 0x0C */
									 /*!< | 0 | 0 | 0 | 0 | GPIOC2 | GPIOC1 | GPIOD2 | GPIOD1 | */
/** @} */

/*! @name ADS1192 Register Configurations
** @{
*/
/*!
	CONFIG1: Configuration Register 1
*/
#define ADS1192_CONFIG1_SINGLE_SHOT	0x80 /*!< Enable single-shot mode */
#define ADS1192_CONFIG1_CONT_CONV	0x00 /*!< Enable continuous conversion mode */
#define ADS1192_CONFIG1_DR_125SPS	0x00 /*!< Configure 125SPS data rate */
#define ADS1192_CONFIG1_DR_250SPS	0x01 /*!< Configure 250SPS data rate */
#define ADS1192_CONFIG1_DR_500SPS	0x02 /*!< Configure 500SPS data rate */
#define ADS1192_CONFIG1_DR_1kSPS	0x03 /*!< Configure 1kSPS data rate */
#define ADS1192_CONFIG1_DR_2kSPS	0x04 /*!< Configure 2kSPS data rate */
#define ADS1192_CONFIG1_DR_4kSPS	0x05 /*!< Configure 4kSPS data rate */
#define ADS1192_CONFIG1_DR_8kSPS	0x06 /*!< Configure 8kSPS data rate */
#define ADS1192_CONFIG1_DR_DISABLED	0x07 /*!< Data Rate not used */
/*!
	CONFIG2: Configuration Register 2
*/
#define ADS1192_CONFIG2_LOFF_COMP_ON	0xC0 /*!< Enable Lead-off comparators */
#define ADS1192_CONFIG2_LOFF_COMP_OFF	0x80 /*!< Disable Lead-off comparators */
#define ADS1192_CONFIG2_REFBUFF_ON		0xA0 /*!< Enable Reference buffer */
#define ADS1192_CONFIG2_REFBUFF_OFF		0x80 /*!< Disable Reference buffer */
#define ADS1192_CONFIG2_VREF_4V			0x90 /*!< Set 4.033V Reference */
#define ADS1192_CONFIG2_VREF_2V			0x80 /*!< Set 2.42V Reference */
#define ADS1192_CONFIG2_CLK_OUT			0x88 /*!< Enable oscillator clock output */
#define ADS1192_CONFIG2_NO_CLK_OUT		0x80 /*!< Disable oscillator clock output */
#define ADS1192_CONFIG2_INT_TEST		0x82 /*!< Test signal ON */
#define ADS1192_CONFIG2_NO_INT_TEST		0x80 /*!< Test signal OFF */
#define ADS1192_CONFIG2_TEST_FREQ_1Hz	0x81 /*!< Set test signal frequency to 1Hz */
#define ADS1192_CONFIG2_TEST_FREQ_DC	0x80 /*!< Set test signal frequency to DC */
/*!
	LOFF: Lead-Off Control Register
*/
#define ADS1192_LOFF_COMP_TH_95		0x10 /*!< Set Lead-off comparator threshold to 95% */
#define ADS1192_LOFF_COMP_TH_92_5	0x30 /*!< Set Lead-off comparator threshold to 92.5% */
#define ADS1192_LOFF_COMP_TH_90		0x50 /*!< Set Lead-off comparator threshold to 90% */
#define ADS1192_LOFF_COMP_TH_87_5	0x70 /*!< Set Lead-off comparator threshold to 87.5% */
#define ADS1192_LOFF_COMP_TH_85		0x90 /*!< Set Lead-off comparator threshold to 85% */
#define ADS1192_LOFF_COMP_TH_80		0xB0 /*!< Set Lead-off comparator threshold to 80% */
#define ADS1192_LOFF_COMP_TH_75		0xD0 /*!< Set Lead-off comparator threshold to 75% */
#define ADS1192_LOFF_COMP_TH_70		0xF0 /*!< Set Lead-off comparator threshold to 70% */
#define ADS1192_LOFF_ILEAD_OFF_6n	0x10 /*!< Set Lead-off current to 6nA */
#define ADS1192_LOFF_ILEAD_OFF_22n	0x14 /*!< Set Lead-off current to 22nA */
#define ADS1192_LOFF_ILEAD_OFF_6u	0x18 /*!< Set Lead-off current to 6uA */
#define ADS1192_LOFF_ILEAD_OFF_22u	0x1C /*!< Set Lead-off current to 22uA */
#define ADS1192_LOFF_FLEAD_OFF_DC	0x10 /*!< Set Lead-off detect at DC */
#define ADS1192_LOFF_FLEAD_OFF_AC	0x11 /*!< Set Lead-off detect at AC */
/*!
	CH1SET: Channel 1 Settings
*/
#define ADS1192_CH1SET_POWER_DOWN	0x80 /*!< Set Channel 1 power down */
#define ADS1192_CH1SET_NORMAL		0x00 /*!< Set Channel 1 normal operation */
#define ADS1192_CH1SET_PGA_1		0x10 /*!< Set Channel 1 PGA gain to 1 */
#define ADS1192_CH1SET_PGA_2		0x20 /*!< Set Channel 1 PGA gain to 2 */
#define ADS1192_CH1SET_PGA_3		0x30 /*!< Set Channel 1 PGA gain to 3 */
#define ADS1192_CH1SET_PGA_4		0x40 /*!< Set Channel 1 PGA gain to 4 */
#define ADS1192_CH1SET_PGA_6		0x00 /*!< Set Channel 1 PGA gain to 6 */
#define ADS1192_CH1SET_PGA_8		0x50 /*!< Set Channel 1 PGA gain to 8 */
#define ADS1192_CH1SET_PGA_12		0x60 /*!< Set Channel 1 PGA gain to 12 */
#define ADS1192_CH1SET_NORMAL_IN	0x00 /*!< Set Channel 1 normal electrode input */
#define ADS1192_CH1SET_SHORTED_IN	0x01 /*!< Set Channel 1 input shorted */
#define ADS1192_CH1SET_RLD_MEASURE	0x02 /*!< Set Channel 1 to RLD Measure */
#define ADS1192_CH1SET_MVDD_IN		0x03 /*!< Set Channel 1 to measure supply */
#define ADS1192_CH1SET_TEMP_IN		0x04 /*!< Set Channel 1 temperature sensor */
#define ADS1192_CH1SET_TEST_IN		0x05 /*!< Set Channel 1 test signal */
#define ADS1192_CH1SET_RLD_PIN		0x06 /*!< Set Channel 1 positive input to RLDIN */
#define ADS1192_CH1SET_RLD_NIN		0x07 /*!< Set Channel 1 negative input to RLDIN */
#define ADS1192_CH1SET_RLD_PIN_NIN	0x08 /*!< Set Channel 1 both inputs to RLDIN */
#define ADS1192_CH1SET_USE_IN3		0x09 /*!< Set Channel 1 to use IN3P and IN3N */
/*!
	CH2SET: Channel 2 Settings
*/
#define ADS1192_CH2SET_POWER_DOWN	0x80 /*!< Set Channel 2 power down */
#define ADS1192_CH2SET_NORMAL		0x00 /*!< Set Channel 2 normal operation */
#define ADS1192_CH2SET_PGA_1		0x10 /*!< Set Channel 2 PGA gain to 1 */
#define ADS1192_CH2SET_PGA_2		0x20 /*!< Set Channel 2 PGA gain to 2 */
#define ADS1192_CH2SET_PGA_3		0x30 /*!< Set Channel 2 PGA gain to 3 */
#define ADS1192_CH2SET_PGA_4		0x40 /*!< Set Channel 2 PGA gain to 4 */
#define ADS1192_CH2SET_PGA_6		0x00 /*!< Set Channel 2 PGA gain to 6 */
#define ADS1192_CH2SET_PGA_8		0x50 /*!< Set Channel 2 PGA gain to 8 */
#define ADS1192_CH2SET_PGA_12		0x60 /*!< Set Channel 2 PGA gain to 12 */
#define ADS1192_CH2SET_NORMAL_IN	0x00 /*!< Set Channel 2 normal electrode input */
#define ADS1192_CH2SET_SHORTED_IN	0x01 /*!< Set Channel 2 input shorted */
#define ADS1192_CH2SET_RLD_MEASURE	0x02 /*!< Set Channel 2 to RLD Measure */
#define ADS1192_CH2SET_MVDD_IN		0x03 /*!< Set Channel 2 to measure supply */
#define ADS1192_CH2SET_TEMP_IN		0x04 /*!< Set Channel 2 temperature sensor */
#define ADS1192_CH2SET_TEST_IN		0x05 /*!< Set Channel 2 test signal */
#define ADS1192_CH2SET_RLD_PIN		0x06 /*!< Set Channel 2 positive input to RLDIN */
#define ADS1192_CH2SET_RLD_NIN		0x07 /*!< Set Channel 2 negative input to RLDIN */
#define ADS1192_CH2SET_RLD_PIN_NIN	0x08 /*!< Set Channel 2 both inputs to RLDIN */
#define ADS1192_CH2SET_USE_IN3		0x09 /*!< Set Channel 2 to use IN3P and IN3N */
/*!
	RLD_SENS: Right Leg Drive Sense Selection
*/
#define ADS1192_RLD_SENS_BUFFER_ON		0x20 /*!< Enable RLD buffer */
#define ADS1192_RLD_SENS_BUFFER_OFF		0x00 /*!< Disable RLD buffer */
#define ADS1192_RLD_SENS_LOFF_SENSE_ON	0x10 /*!< Enable RLD lead-off sense */
#define ADS1192_RLD_SENS_LOFF_SENSE_OFF	0x00 /*!< Disable RLD lead-off sense */
#define ADS1192_RLD_SENS_RLD_IN2N		0x08 /*!< RLD connected to IN2N */
#define ADS1192_RLD_SENS_RLD_IN2N_NC	0x00 /*!< RLD not connected */
#define ADS1192_RLD_SENS_RLD_IN2P		0x04 /*!< RLD connected to IN2P */
#define ADS1192_RLD_SENS_RLD_IN2P_NC	0x00 /*!< RLD not connected */
#define ADS1192_RLD_SENS_RLD_IN1N		0x02 /*!< RLD connected to IN1N */
#define ADS1192_RLD_SENS_RLD_IN1N_NC	0x00 /*!< RLD not connected */
#define ADS1192_RLD_SENS_RLD_IN1P		0x01 /*!< RLD connected to IN1P */
#define ADS1192_RLD_SENS_RLD_IN1P_NC	0x00 /*!< RLD not connected */
/*!
	LOFF_SENS: Lead-Off Sense Selection
*/
#define ADS1192_LOFF_SENS_CH2_CURRENT_ON	0x20 /*!< Enable lead-off current for channel 2 */
#define ADS1192_LOFF_SENS_CH2_CURRENT_OFF	0x00 /*!< Disable lead-off current for channel 2 */
#define ADS1192_LOFF_SENS_CH1_CURRENT_ON	0x10 /*!< Enable lead-off current for channel 1 */
#define ADS1192_LOFF_SENS_CH1_CURRENT_OFF	0x00 /*!< Disable lead-off current for channel 1 */
#define ADS1192_LOFF_SENS_LOFF_IN2N_ON		0x08 /*!< Enable lead-off detection from IN2N */
#define ADS1192_LOFF_SENS_LOFF_IN2N_OFF		0x00 /*!< Disable lead-off detection from IN2N */
#define ADS1192_LOFF_SENS_LOFF_IN2P_ON		0x04 /*!< Enable lead-off detection from IN2P */
#define ADS1192_LOFF_SENS_LOFF_IN2P_OFF		0x00 /*!< Disable lead-off detection from IN2P */
#define ADS1192_LOFF_SENS_LOFF_IN1N_ON		0x02 /*!< Enable lead-off detection from IN1N */
#define ADS1192_LOFF_SENS_LOFF_IN1N_OFF		0x00 /*!< Disable lead-off detection from IN1N */
#define ADS1192_LOFF_SENS_LOFF_IN1P_ON		0x01 /*!< Enable lead-off detection from IN1P */
#define ADS1192_LOFF_SENS_LOFF_IN1P_OFF		0x20 /*!< Disable lead-off detection from IN1P */
/*!
	LOFF_STAT: Lead-Off Status
*/
#define ADS1192_LOFF_STAT_CLK_DIV_4		0x00 /*!< Set fMOD/4 when external clock is 512kHz */
#define ADS1192_LOFF_STAT_CLK_DIV_16	0x40 /*!< Set fMOD/4 when external clock is 2.048MHz */
#define ADS1192_LOFF_STAT_RLD_CON		0x00 /*!< Read-only. RLD is connected */
#define ADS1192_LOFF_STAT_RLD_NC		0x10 /*!< Read-only. RLD is not connected */
#define ADS1192_LOFF_STAT_IN2N_CON		0x00 /*!< Read-only. channel 2 negative electrode connected */
#define ADS1192_LOFF_STAT_IN2N_NC		0x08 /*!< Read-only. channel 2 negative electrode not connected */
#define ADS1192_LOFF_STAT_IN2P_CON		0x00 /*!< Read-only. channel 2 positive electrode connected */
#define ADS1192_LOFF_STAT_IN2P_NC		0x04 /*!< Read-only. channel 2 positive electrode not connected */
#define ADS1192_LOFF_STAT_IN1N_CON		0x00 /*!< Read-only. channel 1 negative electrode connected */
#define ADS1192_LOFF_STAT_IN1N_NC		0x02 /*!< Read-only. channel 1 negative electrode not connected */
#define ADS1192_LOFF_STAT_IN1P_CON		0x00 /*!< Read-only. channel 1 positive electrode connected */
#define ADS1192_LOFF_STAT_IN1P_NC		0x01 /*!< Read-only. channel 1 positive electrode not connected */
/*!
	MISC1: Miscellaneous Control Register 1
*/
#define ADS1192_MISC1_DEFAULT	0x02 /*!< Register must be configured to that value */
/*!
	MISC2: Miscellaneous Control Register 2
*/
#define ADS1192_MISC2_CALIB_ON		0x80 /*!< Enable offset calibration */
#define ADS1192_MISC2_CALIB_OFF		0x00 /*!< Disable offset calibration */
#define ADS1192_MISC2_RLDREF_INT	0x02 /*!< RLDREF is fed internally */
#define ADS1192_MISC2_RLDREF_EXT	0x00 /*!< RLDREF is external */
/*!
	GPIO: General-Purpose I/O Register
*/
#define ADS1192_GPIO_SET_1_IN		0x04 /*!< Set GPIO 1 as input */
#define ADS1192_GPIO_SET_1_OUT		0x00 /*!< Set GPIO 1 as output */
#define ADS1192_GPIO_SET_2_IN		0x08 /*!< Set GPIO 2 as input */
#define ADS1192_GPIO_SET_2_OUT		0x00 /*!< Set GPIO 2 as output */
#define ADS1192_GPIO_1_HIGH			0x01 /*!< Write/Read GPIO 1 High */
#define ADS1192_GPIO_1_LOW			0x00 /*!< Write/Read GPIO 1 Low */
#define ADS1192_GPIO_2_HIGH			0x02 /*!< Write/Read GPIO 2 High */
#define ADS1192_GPIO_2_LOW			0x00 /*!< Write/Read GPIO 2 Low */

/** @} */


/* Function prototypes*/
void 
ads1192_spi_init (void);
void 
ads1192_init (ads1192_mode_t ads1192_mode);
uint16_t
ads1192_read_reg (uint8_t ads1192_reg);
void 
ads1192_write_reg (uint8_t ads1192_reg, uint8_t ads1192_reg_value);
void
ads1192_burst_read (uint8_t ads1192_reg, uint8_t *ads1192_reg_value, uint8_t len);
void
ads1192_burst_write (uint16_t *ads1192_reg_value, uint8_t len);


#endif /* _LIBADS1192_H */

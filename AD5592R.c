#include "AD5592R.h"
#include <stdint.h>
#include "nrf_drv_spi.h"
#include "nrf_delay.h"
#include "SEGGER_RTT.h"       // debugging
/**
 * Select the SPI channel.
 * Parameters:
 * 	ch = channel number
 */

#define SPI_INSTANCE  1 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

static uint8_t       m_tx_buf[2];          /**< TX buffer. */
static uint8_t       m_rx_buf[2];//sizeof(TEST_STRING) + 1];    /**< RX buffer. */

void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi_xfer_done = true;
    if (m_rx_buf[0] != 0)
    {
        //NRF_LOG_INFO(" Received:");
        printf(m_rx_buf, strlen((const char *)m_rx_buf));
    }
}

void ad5592r_dac_out(uint8_t pin, uint16_t milivolts)
{
//	if(!((analogOutPins >> pin ) & 0x1))
//	{
//		setAsDAC(analogOutPins | (0x1 << pin));
//	}
//	spiComs(AD5592_DAC_WRITE_MASK | 		/* DAC write command */
//	((pin <<12) & AD5592_DAC_ADDRESS_MASK)|	/* Set which pin to write */
//	a2d(milivolts));						/* Load digital value */
        uint16_t set_volt = milivolts / 0.6105f;
        
        uint16_t volt_buf = 0x8000 | set_volt;
        //SEGGER_RTT_printf(0,"volt_buf0: %d, \n", volt_buf);

spi_xfer_done = false;
        m_tx_buf[1]= (0xFF & (volt_buf));//0xFF; 
        m_tx_buf[0]= (0xFF & (volt_buf>>8));//0x8F; //800 in dac 0 

//        m_tx_buf[1]= 0xFF; 
//        m_tx_buf[0]= 0x8F; //800 in dac 0 
        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, 2, m_rx_buf, 2)); // sw reset
        //SEGGER_RTT_printf(0, "800 in dac 0  \n");
spi_xfer_done = false;
        m_tx_buf[1]= 0x18; 
        m_tx_buf[0]= 0x08; //read dac 3
        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, 2, m_rx_buf, 2)); // sw reset
        //SEGGER_RTT_printf(0,"out1: %d, \n", m_rx_buf[0]);
        //SEGGER_RTT_printf(0,"out2: %d, \n", m_rx_buf[1]);

spi_xfer_done = false;
        m_tx_buf[1]= 0x00; 
        m_tx_buf[0]= 0x00; //read dac 3
        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, 2, m_rx_buf, 2)); // int ref
        //SEGGER_RTT_printf(0,"out3: %d, \n", m_rx_buf[0]);
        //SEGGER_RTT_printf(0,"out4: %d, \n", m_rx_buf[1]);
}



/**
 * Initialize the SPI for using the AD5592. Does not set channel. Do that
 * after calling this function by calling setAD5592Ch().
 * Parameters:
 * 	none
 */
void ad5592r_init()
{
nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = 11;//29;//
    spi_config.miso_pin = 17;//30;//
    spi_config.mosi_pin = 12;//27;//
    spi_config.sck_pin  = 13;//25;//
    spi_config.frequency    = NRF_DRV_SPI_FREQ_8M;                     
    spi_config.mode         = NRF_DRV_SPI_MODE_1;    //MODE 0 for flash 1 for DAC                  
    spi_config.bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
    
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));

    SEGGER_RTT_printf(0, "SPI start");
        spi_xfer_done = false;
        //nrf_gpio_pin_clear(30);
        m_tx_buf[1]= 0xAC; 
        m_tx_buf[0]= 0x7D;
        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, 2, m_rx_buf, 2)); // sw reset
        spi_xfer_done = true;
        nrf_delay_ms(10);

        SEGGER_RTT_printf(0, "sw reset\n");


        m_tx_buf[1]= 0x00; 
        m_tx_buf[0]= 0x5A; //set internal ref
        spi_xfer_done = false;
        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, 2, m_rx_buf, 2)); // ch 1 out
        spi_xfer_done = true;
        SEGGER_RTT_printf(0, "set internal ref\n");

        m_tx_buf[1]= 0xFF; 
        m_tx_buf[0]= 0x28; //set all dac output 
        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, 2, m_rx_buf, 2)); // ch 1 out
        SEGGER_RTT_printf(0, "set all dac output \n");

        m_tx_buf[1]= 0x00; 
        m_tx_buf[0]= 0x38; //ldac immediate
        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, 2, m_rx_buf, 2)); // sw reset
        SEGGER_RTT_printf(0, "dac immediate \n");

//        m_tx_buf[1]= 0xFF; 
//        m_tx_buf[0]= 0x8F; //800 in dac 0 
//        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, 2, m_rx_buf, 2)); // sw reset
//        SEGGER_RTT_printf(0, "800 in dac 0  \n");
//
//        m_tx_buf[1]= 0x18; 
//        m_tx_buf[0]= 0x08; //read dac 3
//        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, 2, m_rx_buf, 2)); // sw reset
//        SEGGER_RTT_printf(0,"out1: %d, \n", m_rx_buf[0]);
//        SEGGER_RTT_printf(0,"out2: %d, \n", m_rx_buf[1]);
//
//
//        m_tx_buf[1]= 0x00; 
//        m_tx_buf[0]= 0x00; //read dac 3
//        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, 2, m_rx_buf, 2)); // int ref
//        SEGGER_RTT_printf(0,"out3: %d, \n", m_rx_buf[0]);
//        SEGGER_RTT_printf(0,"out4: %d, \n", m_rx_buf[1]);





}
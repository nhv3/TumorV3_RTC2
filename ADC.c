#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "boards.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define SAMPLES_IN_BUFFER 1
volatile uint8_t state = 1;
#define SAADC_CHANNEL 0
#define OVERSAMP 16
static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(3);
static nrf_saadc_value_t     m_buffer_pool[2][SAMPLES_IN_BUFFER];
static nrf_ppi_channel_t     m_ppi_channel;
static uint32_t              m_adc_evt_counter;
/**
 * @brief Not used in this example, but driver API requiers a callback function to be proivded.
 */
void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

        int i;
        //SEGGER_RTT_printf(0,"ADC event number: %d", (int)m_adc_evt_counter);

        for (i = 0; i < SAMPLES_IN_BUFFER; i++)
        {
            //SEGGER_RTT_printf(0,"ADC val %d\n", p_event->data.done.p_buffer[i]);
            int16_t adc_val = p_event->data.done.p_buffer[i];
        }
        m_adc_evt_counter++;
    }
}

void saadc_init3(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN3);

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);

    err_code = nrf_drv_saadc_channel_init(SAADC_CHANNEL, &channel_config);
    APP_ERROR_CHECK(err_code);
}


float return_adc3(void)
{
    ret_code_t err_code;
    nrf_saadc_value_t sample;
    saadc_init3();
    float sum = 0;
    for (int i = 0; i < OVERSAMP; i++)
      {
      err_code = nrfx_saadc_sample_convert(SAADC_CHANNEL, &sample);
      APP_ERROR_CHECK(err_code);
      sum += sample;
      nrf_delay_ms(5);
      }
      float adc_avg = sum/OVERSAMP;
      return adc_avg;
}

void saadc_init7(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN7);

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);

    err_code = nrf_drv_saadc_channel_init(SAADC_CHANNEL, &channel_config);
    APP_ERROR_CHECK(err_code);
}


float return_adc7(void)
{
    ret_code_t err_code;
    nrf_saadc_value_t sample;
    saadc_init7();
    float sum = 0;
    for (int i = 0; i < OVERSAMP; i++)
      {
      err_code = nrfx_saadc_sample_convert(SAADC_CHANNEL, &sample);
      APP_ERROR_CHECK(err_code);
      sum += sample;
      nrf_delay_ms(5);
      }
      float adc_avg = sum/OVERSAMP;
      return adc_avg;
}


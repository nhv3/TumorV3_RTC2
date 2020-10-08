
#ifndef MA_ADC_H__
#define MA_ADC_H__

#ifdef __cplusplus
extern "C" {
#endif
#include "SEGGER_RTT.h"    
#include "nrf_drv_saadc.h"

void saadc_callback(nrf_drv_saadc_evt_t const * p_event);
void saadc_init3(void);
float return_adc3(void);
void saadc_init7(void);
float return_adc7(void);
#ifdef __cplusplus
}
#endif

#endif

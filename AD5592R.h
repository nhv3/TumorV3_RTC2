#ifndef AD5592R_H_
#define AD5592R_H_

#include <stdint.h>

#define AD5592_CNTRL_ADDRESS_MASK	0x7800	/* Control register bit mask */
#define	AD5592_NOP					0x0000	/* No operation */
#define	AD5592_DAC_READBACK			0x0800	/* Selects and enables DAC read back */
#define	AD5592_ADC_READ				0x1000	/* Selects ADCs for conversion */
#define	AD5592_GP_CNTRL				0x1800	/* General purpose control register */
#define	AD5592_ADC_PIN_SELECT		0x2000	/* Selects which pins are ADC inputs */
#define AD5592_DAC_PIN_SELECT		0x2800	/* Selects which pins are DAC outputs */
#define	AD5592_PULL_DOWN_SET		0x3000	/* Selects which pins have 85kOhm pull-down resistor to GND */
#define	AD5592_CNTRL_REG_READBACK	0x3800	/* Read back control registers and/or set LDAC */
#define	AD5592_GPIO_WRITE_CONFIG	0x4000	/* Selects which pins are GPIO outputs */
#define	AD5592_GPIO_WRITE_DATA		0x4800	/* Writes data to the GPIO outputs */
#define	AD5592_GPIO_READ_CONFIG		0x5000	/* Selects which pins are GPIO inputs */
#define AD5592_GPIO_READ_INPUT		0x5400	/* Read GPIO inputs */
#define	AD5592_POWER_DWN_REF_CNTRL	0x5800	/* Powers down DACs and enables/disables the reference */
#define	AD5592_GPIO_DRAIN_CONFIG	0x6000	/* Selects open-drain or push/pull for GPIO outputs */
#define AD5592_THREE_STATE_CONFIG	0x6800	/* Selects which pins are three-state */
#define	AD5592_SW_RESET				0x7DAC	/* Software reset of the AD5592 */

/**
 * Pins
 */
#define AD5592_IO0	0x01
#define AD5592_IO1	0x02
#define AD5592_IO2	0x04
#define	AD5592_IO3	0x08
#define	AD5592_IO4	0x10
#define	AD5592_IO5	0x20
#define	AD5592_IO6	0x40
#define	AD5592_IO7	0x80

#define AD5592_PIN_SELECT_MASK		0x00FF	/* Pin select bit mask */

/**
 * DAC register definitions.
 */
#define AD5592_DAC_WRITE_MASK		0x8000	/* DAC write bit mask */
#define AD5592_DAC_ADDRESS_MASK		0x7000	/* DAC pin address bit mask */
#define AD5592_DAC_VALUE_MASK		0x0FFF	/* DAC output value bit mask */

void ad5592r_dac_out(uint8_t pin, uint16_t milivolts);
void ad5592r_init();

#endif
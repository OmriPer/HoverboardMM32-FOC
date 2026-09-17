/*
 * SPIN27 glue for EFeru's BLDC controller. See Inc/foc_config.h. GPL-3.0.
 */
#ifndef FOC_EFERU_H
#define FOC_EFERU_H

#include <stdint.h>

/* Call once after TIM1_init() and adc_Init(), before the ADC interrupt runs. */
void FOC_Init(void);

/* Call from the ADC end-of-conversion interrupt, once per PWM period, with the raw
 * 12-bit readings of IPHASEAPIN and IPHASEBPIN. */
void FOC_Isr(uint16_t adcPhaseA, uint16_t adcPhaseB);

/* Map pinstorage DRIVEMODE (0..6) onto the controller type and mode. Returns 0 if invalid. */
uint8_t FOC_ApplyDriveMode(uint16_t driveMode);

/* Diagnostics, readable with the debugger. */
extern uint32_t foc_isrCount;       // controller steps executed
extern uint16_t foc_maxIsrTicks;    // longest FOC_Isr run in TIM1 ticks (10.4 ns each); one PWM period = 6000
extern uint8_t  foc_errCode;        // controller z_errCode (hall/blocked-motor diagnostics)
extern uint8_t  foc_calibrated;     // 1 once phase current offsets are measured

#endif

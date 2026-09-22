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

/* Filtered controller iq (torque current) in 0.01 A, for the RemoteUartBus answer. 0 outside FOC modes. */
int16_t FOC_GetIqCentiAmps(void);

/* Change the current limit (0.1 A) and speed limit (rpm) at runtime, RAM only; 0 = leave unchanged.
 * Clamped to FOC_I_MOT_LIMIT / FOC_N_MOT_LIMIT. FOC_I_MOT_MAX / FOC_N_MOT_MAX apply after reset. */
void FOC_SetLimits(uint16_t currentDeciAmps, uint16_t speedRpm);

/* Diagnostics, readable with the debugger. */
extern uint32_t foc_isrCount;       // controller steps executed
extern uint16_t foc_maxIsrTicks;    // longest FOC_Isr run in TIM1 ticks (10.4 ns each); one PWM period = 6000
extern uint8_t  foc_errCode;        // controller z_errCode (hall/blocked-motor diagnostics)
extern uint8_t  foc_calibrated;     // 1 once phase current offsets are measured
extern uint8_t  foc_hallInvert;     // runtime hall polarity, starts at FOC_HALL_INVERT
extern uint8_t  foc_hallOrder;      // runtime hall order 0..5, starts at FOC_HALL_ORDER
extern int16_t  foc_curA, foc_curB;            // phase currents [ADC counts, offset removed]
extern int16_t  foc_curAFilt, foc_curBFilt;    // same, low-pass filtered (~4 ms)
extern volatile uint16_t foc_testTicks;        // bench test: static vector active while > 0
extern volatile int16_t  foc_testDc[3];        // bench test duty per phase, DC_phaX scale

#endif

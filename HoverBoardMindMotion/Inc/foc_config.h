/*
 * Motor control with EFeru's BLDC controller (hoverboard-firmware-hack-FOC).
 *
 * BLDC_controller.c/.h, BLDC_controller_data.c and rtwtypes.h are copied unmodified from
 * https://github.com/EFeru/hoverboard-firmware-hack-FOC (commit 4f141cb), GPL-3.0.
 * Because they are included, this firmware as a whole is distributed under GPL-3.0
 * (see LICENSE.BLDC_controller-GPL-3.0).
 *
 * This file holds the settings for the SPIN27 glue in Src/foc_eferu.c. Board pins and
 * DRIVEMODE come from pinstorage in Src/main.c.
 */
#ifndef FOC_CONFIG_H
#define FOC_CONFIG_H

/* Build switch: 1 = EFeru controller runs in the ADC interrupt, replacing HALLModuleCalc(),
 * commutate() and the unfinished FOC path. 0 = original motor control. */
#define FOC_EFERU 1

/* Control loop rate. The controller constants (speed estimate, error timers) are tuned for
 * 16 kHz, and the ADC interrupt runs once per PWM period, so PWM_RES in pinstorage must be
 * 96000000 / 2 / FOC_PWM_FREQ = 3000 (center-aligned TIM1 on a 96 MHz clock). */
#define FOC_PWM_FREQ          16000
#define FOC_PWM_RES           (96000000 / 2 / FOC_PWM_FREQ)   // 3000

/* The controller's duty output DC_phaX is scaled for a 2000-count period ([-1000, 1000]). */
#define FOC_CTRL_PWM_RES      2000

/* Minimum low-side on-time kept around the current sampling point (EFeru uses 110 of 2000
 * in FOC), scaled to FOC_PWM_RES. */
#define FOC_PWM_MARGIN        (110 * FOC_PWM_RES / FOC_CTRL_PWM_RES)

/* TIM1 dead time in timer ticks (1 tick = 10.4 ns at 96 MHz): 64 = ~670 ns, close to EFeru's
 * 48 ticks at 64 MHz. The EG2123A adds its own dead time and interlock on top. */
#define FOC_DEAD_TIME         64

/* Samples used to calibrate the phase current offsets at startup (outputs stay off). */
#define FOC_OFFSET_SAMPLES    2000

/* Hall inputs, measured on this board in SIN voltage mode at command +-80 (wheel lifted):
 *   invert 1, order 4:  +31.9 / -33.3 rpm, smooth both ways, command sign = measured sign  <- used
 *   invert 0, order 4:  -33.2 / +32.1 rpm, smooth but 180 degrees off (sign reversed)
 *   invert 0, order 3:  stall / -27.1 rpm
 *   invert 1, order 0:  about -25 rpm forward, stutters in reverse (first bring-up default) */
#define FOC_HALL_INVERT       1
/* Order in which HALLAPIN/HALLBPIN/HALLCPIN feed the controller's hall A/B/C (0..5, see
 * foc_eferu.c). PinFinder's A/B/C follow its own commutation table, which need not match
 * the controller's phase convention. Both values can be changed at runtime from the debugger
 * (foc_hallInvert, foc_hallOrder); put the working pair here once found. */
#define FOC_HALL_ORDER        4     // controller A,B,C = HALLC,HALLA,HALLB = PB9,PB8,PB4

/* Phase current amplifiers: the low-side current signals go through the MCU's built-in op-amps,
 * with the same circuit as MindMotion's SPIN27 motor kit (MM32SPIN27PS_SCH.pdf, gain ~5, output
 * biased to mid-rail):
 *   OP1: + PA4, - PA5, output PA6 (ADC ch6)   measures controller phase C (TIM1 CH3)
 *   OP2: + PB0, - PB1, output PB2 (ADC ch10)  measures controller phase B (TIM1 CH2)
 * Phases measured on this board with static voltage vectors (foc_testDc); positive = current
 * into the motor, same as EFeru. The outputs are only valid while the low-side FETs conduct
 * (sampled at the PWM bottom, TIM1 CC4 = 1).
 * The op-amps are off after reset, so FOC_Init() enables them. IPHASEAPIN/IPHASEBPIN in
 * pinstorage must be PB2/PA6. */
#define FOC_OPAMP_ENABLE      1

/* Phase current scaling. The controller expects A2BIT_CONV counts per ampere on i_phaAB/i_phaBC.
 * Gain is NOT calibrated yet: current = (offset - adc) * NUM / DEN.
 * Set these after comparing phase currents against a known current (see bring-up notes). */
#define FOC_A2BIT_CONV        50
#define FOC_CUR_GAIN_NUM      1
#define FOC_CUR_GAIN_DEN      1
/* Which phases IPHASEA/IPHASEB measure: 0 = {iA, iB}, 1 = {iB, iC}, 2 = {iA, iC}. Measured: {iB, iC}. */
#define FOC_CUR_PHASE_SEL     1

/* Controller limits (same meaning as EFeru config.h). */
#define FOC_DIAG_ENA          1       // motor diagnostics (hall errors, blocked motor)
#define FOC_I_MOT_MAX         15      // [A] FOC current limit. Meaningless until currents are calibrated.
#define FOC_N_MOT_MAX         1000    // [rpm] speed limit
#define FOC_FIELD_WEAK_ENA    0       // field weakening / phase advance off
#define FOC_FIELD_WEAK_MAX    5       // [A]
#define FOC_PHASE_ADV_MAX     25      // [deg] SIN only
#define FOC_FIELD_WEAK_HI     1000
#define FOC_FIELD_WEAK_LO     750

#endif

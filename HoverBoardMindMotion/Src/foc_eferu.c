/*
 * SPIN27 glue for EFeru's BLDC controller (hoverboard-firmware-hack-FOC, GPL-3.0).
 *
 * Runs the controller once per PWM period from the ADC interrupt:
 *   ADC phase currents + hall pins + RemoteUartBus speed  ->  BLDC_controller_step()
 *   -> duty cycles on TIM1 CCR1..3, speed and odometry back to the RemoteUartBus answer.
 *
 * Structure follows EFeru's bldc.c (DMA1_Channel1_IRQHandler) and util.c (BLDC_Init),
 * adapted to this board's timer setup. Settings are in Inc/foc_config.h.
 *
 * This file is part of a GPL-3.0 licensed firmware (see LICENSE.BLDC_controller-GPL-3.0).
 */
#ifdef TARGET_MM32SPIN25
#include "HAL_device.h"
#include "spin25-redefine.h"
#else
#include "mm32_device.h"
#endif
#include "hal_tim.h"
#include "hal_rcc.h"
#include "hal_op.h"
#include "../Inc/pinout.h"
#include "../Inc/hardware.h"
#include "../Inc/bldc.h"
#include "BLDC_controller.h"
#include "rtwtypes.h"
#include "../Inc/foc_config.h"
#include "../Inc/foc_eferu.h"

/* Controller type and mode numbers (EFeru config.h). */
#define CTRL_TYP_COM   0
#define CTRL_TYP_SIN   1
#define CTRL_TYP_FOC   2
#define CTRL_MOD_VLT   1
#define CTRL_MOD_SPD   2
#define CTRL_MOD_TRQ   3

/* Controller instance. rtP_Left holds EFeru's default parameters (BLDC_controller_data.c). */
extern P rtP_Left;
static RT_MODEL rtM_motor_;
static RT_MODEL *const rtM_motor = &rtM_motor_;
static DW   rtDW_motor;
static ExtU rtU_motor;
static ExtY rtY_motor;

/* Firmware globals (main.c / bldc.c). */
extern int     speed;        // RemoteUartBus command, -1000..1000 (set to 0 after SERIAL_TIMEOUT)
extern int     realspeed;    // sent back in the RemoteUartBus answer
extern int32_t iOdom;        // hall steps, sent back in the RemoteUartBus answer
extern uint8_t lowbatperm;   // set by speedupdate() at low battery

uint32_t foc_isrCount    = 0;
uint16_t foc_maxIsrTicks = 0;
uint8_t  foc_errCode     = 0;
uint8_t  foc_calibrated  = 0;

/* Hall mapping, writable at runtime (e.g. VS Code Live Watch) for bring-up.
 * foc_hallInvert: 1 = invert all three hall inputs (180 degree shift).
 * foc_hallOrder:  which board pins feed the controller's hall A, B, C:
 *   0 = A,B,C  1 = A,C,B  2 = B,A,C  3 = B,C,A  4 = C,A,B  5 = C,B,A
 *   (A/B/C = HALLAPIN/HALLBPIN/HALLCPIN from pinstorage). Odd swaps reverse the sequence. */
uint8_t  foc_hallInvert  = FOC_HALL_INVERT;
uint8_t  foc_hallOrder   = FOC_HALL_ORDER;
/* Bench test for current sensor calibration, driven from the debugger only.
 * While foc_testTicks > 0 the controller is bypassed and foc_testDc[] (DC_phaX scale, each
 * limited to +-FOC_TEST_DC_MAX) is applied as a static voltage vector. foc_testTicks counts
 * down once per PWM period (16000 per second), so the test ends by itself. */
#define FOC_TEST_DC_MAX 100
volatile uint16_t foc_testTicks = 0;
volatile int16_t  foc_testDc[3] = {0, 0, 0};

/* Phase currents in ADC counts (offset removed, gain applied), and the same low-pass filtered
 * over about 64 PWM periods (4 ms), for reading with the debugger. */
int16_t foc_curA = 0;
int16_t foc_curB = 0;
int16_t foc_curAFilt = 0;
int16_t foc_curBFilt = 0;
static int32_t curAAcc = 0;
static int32_t curBAcc = 0;

static const uint8_t hallOrderTab[6][3] = {{0,1,2},{0,2,1},{1,0,2},{1,2,0},{2,0,1},{2,1,0}};

static uint8_t  ctrlModReq = CTRL_MOD_VLT;
static uint16_t offsetCount = 0;
static int16_t  offsetA = 2048;
static int16_t  offsetB = 2048;
static int8_t   hallPosPrev = -1;

uint8_t FOC_ApplyDriveMode(uint16_t driveMode)
{
    /* pinstorage DRIVEMODE: 0 COM_VOLT, 1 COM_SPEED, 2 SINE_VOLT, 3 SINE_SPEED,
     *                       4 FOC_VOLT, 5 FOC_SPEED, 6 FOC_TORQUE.
     * The controller only offers speed and torque mode with FOC; COM_SPEED and SINE_SPEED
     * therefore run as voltage mode. */
    switch (driveMode) {
        case COM_VOLT:
        case COM_SPEED:  rtP_Left.z_ctrlTypSel = CTRL_TYP_COM; ctrlModReq = CTRL_MOD_VLT; break;
        case SINE_VOLT:
        case SINE_SPEED: rtP_Left.z_ctrlTypSel = CTRL_TYP_SIN; ctrlModReq = CTRL_MOD_VLT; break;
        case FOC_VOLT:   rtP_Left.z_ctrlTypSel = CTRL_TYP_FOC; ctrlModReq = CTRL_MOD_VLT; break;
        case FOC_SPEED:  rtP_Left.z_ctrlTypSel = CTRL_TYP_FOC; ctrlModReq = CTRL_MOD_SPD; break;
        case FOC_TORQUE: rtP_Left.z_ctrlTypSel = CTRL_TYP_FOC; ctrlModReq = CTRL_MOD_TRQ; break;
        default: return 0;
    }
    return 1;
}

void FOC_Init(void)
{
    /* Same parameter setup as EFeru util.c BLDC_Init(). */
    rtP_Left.b_angleMeasEna     = 0;
    rtP_Left.z_selPhaCurMeasABC = FOC_CUR_PHASE_SEL;
    rtP_Left.b_diagEna          = FOC_DIAG_ENA;
    rtP_Left.i_max              = (FOC_I_MOT_MAX * FOC_A2BIT_CONV) << 4;
    rtP_Left.n_max              = FOC_N_MOT_MAX << 4;
    rtP_Left.b_fieldWeakEna     = FOC_FIELD_WEAK_ENA;
    rtP_Left.id_fieldWeakMax    = (FOC_FIELD_WEAK_MAX * FOC_A2BIT_CONV) << 4;
    rtP_Left.a_phaAdvMax        = FOC_PHASE_ADV_MAX << 4;
    rtP_Left.r_fieldWeakHi      = FOC_FIELD_WEAK_HI << 4;
    rtP_Left.r_fieldWeakLo      = FOC_FIELD_WEAK_LO << 4;
    if (!FOC_ApplyDriveMode(DRIVEMODE)) {
        FOC_ApplyDriveMode(SINE_VOLT);
    }

#if FOC_OPAMP_ENABLE
    /* Phase current amplifiers (see foc_config.h): inputs and outputs analog, then enable. */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_OPAMP, ENABLE);
    pinMode(PA4, INPUT_ADC);
    pinMode(PA5, INPUT_ADC);
    pinMode(PA6, INPUT_ADC);
    pinMode(PB0, INPUT_ADC);
    pinMode(PB1, INPUT_ADC);
    pinMode(PB2, INPUT_ADC);
    OPAMP_Configure(OPAMP1, ENABLE);
    OPAMP_Configure(OPAMP2, ENABLE);
#endif

    rtM_motor->defaultParam = &rtP_Left;
    rtM_motor->dwork        = &rtDW_motor;
    rtM_motor->inputs       = &rtU_motor;
    rtM_motor->outputs      = &rtY_motor;
    BLDC_controller_initialize(rtM_motor);

    /* The controller timing assumes FOC_PWM_FREQ. Enforce the matching TIM1 period even if
     * PWM_RES in pinstorage differs (TIM1_init() used PWM_RES). */
    PWM_RES  = FOC_PWM_RES;
    TIM1->ARR = FOC_PWM_RES;

    /* Outputs stay off until the current offsets are calibrated. */
    TIM1->CCR1 = FOC_PWM_RES / 2;
    TIM1->CCR2 = FOC_PWM_RES / 2;
    TIM1->CCR3 = FOC_PWM_RES / 2;
    TIM_CtrlPWMOutputs(TIM1, DISABLE);
}

/* Position on the center-aligned counter triangle, 0 .. 2*ARR, for timing measurement. */
static uint16_t timerTrianglePos(void)
{
    uint16_t cnt = (uint16_t)TIM1->CNT;
    return (TIM1->CR1 & TIM_CR1_DIR) ? (uint16_t)(2 * FOC_PWM_RES - cnt) : cnt;
}

static int16_t clamp16(int32_t v, int32_t lo, int32_t hi)
{
    return (int16_t)(v < lo ? lo : (v > hi ? hi : v));
}

void FOC_Isr(uint16_t adcPhaseA, uint16_t adcPhaseB)
{
    uint16_t tStart = timerTrianglePos();

    /* 1. Phase current offsets, measured with the outputs off (EFeru bldc.c method). */
    if (offsetCount < FOC_OFFSET_SAMPLES) {
        offsetA = (int16_t)((adcPhaseA + offsetA) / 2);
        offsetB = (int16_t)((adcPhaseB + offsetB) / 2);
        offsetCount++;
        if (offsetCount == FOC_OFFSET_SAMPLES) {
            foc_calibrated = 1;
        }
        return;
    }

    /* 2. Inputs. */
    uint8_t pins[3] = { digitalRead(HALLAPIN) ? 1 : 0, digitalRead(HALLBPIN) ? 1 : 0, digitalRead(HALLCPIN) ? 1 : 0 };
    const uint8_t *order = hallOrderTab[foc_hallOrder < 6 ? foc_hallOrder : 0];
    uint8_t hallA = pins[order[0]];
    uint8_t hallB = pins[order[1]];
    uint8_t hallC = pins[order[2]];
    if (foc_hallInvert) {
        hallA ^= 1; hallB ^= 1; hallC ^= 1;
    }

    int32_t curA = ((int32_t)offsetA - adcPhaseA) * FOC_CUR_GAIN_NUM / FOC_CUR_GAIN_DEN;
    int32_t curB = ((int32_t)offsetB - adcPhaseB) * FOC_CUR_GAIN_NUM / FOC_CUR_GAIN_DEN;

    foc_curA = clamp16(curA, -32768, 32767);
    foc_curB = clamp16(curB, -32768, 32767);
    curAAcc += foc_curA - (curAAcc >> 6);
    curBAcc += foc_curB - (curBAcc >> 6);
    foc_curAFilt = (int16_t)(curAAcc >> 6);
    foc_curBFilt = (int16_t)(curBAcc >> 6);

    uint8_t enable = (!lowbatperm && !foc_errCode) ? 1 : 0;

    if (foc_testTicks) {
        foc_testTicks--;
        volatile uint32_t *ccr[3] = { &TIM1->CCR1, &TIM1->CCR2, &TIM1->CCR3 };
        for (uint8_t i = 0; i < 3; i++) {
            int32_t dc = clamp16(foc_testDc[i], -FOC_TEST_DC_MAX, FOC_TEST_DC_MAX);
            *ccr[i] = (uint16_t)(FOC_PWM_RES / 2 - dc * FOC_PWM_RES / FOC_CTRL_PWM_RES);
        }
        TIM_CtrlPWMOutputs(TIM1, enable ? ENABLE : DISABLE);
        return;
    }

    rtU_motor.b_motEna     = enable;
    rtU_motor.z_ctrlModReq = ctrlModReq;
    rtU_motor.r_inpTgt     = clamp16(speed, -1000, 1000);
    rtU_motor.b_hallA      = hallA;
    rtU_motor.b_hallB      = hallB;
    rtU_motor.b_hallC      = hallC;
    rtU_motor.i_phaAB      = clamp16(curA, -32768, 32767);
    rtU_motor.i_phaBC      = clamp16(curB, -32768, 32767);
    rtU_motor.i_DCLink     = 0;   // no DC link current measurement on this board

    /* 3. Controller step. */
    BLDC_controller_step(rtM_motor);
    foc_errCode = rtY_motor.z_errCode;

    /* 4. Outputs. The controller duty is +-1000 for a 2000-count period, centred on 50%.
     * TIM1 runs in PWM2 mode (high side on while CNT > CCR), so a positive duty needs a
     * smaller CCR. The margin keeps the low sides on around the sampling point at CNT = 1. */
    int32_t margin = (rtP_Left.z_ctrlTypSel == CTRL_TYP_FOC) ? FOC_PWM_MARGIN : 0;
    TIM1->CCR1 = (uint16_t)clamp16(FOC_PWM_RES / 2 - (int32_t)rtY_motor.DC_phaA * FOC_PWM_RES / FOC_CTRL_PWM_RES,
                                   margin, FOC_PWM_RES - margin);
    TIM1->CCR2 = (uint16_t)clamp16(FOC_PWM_RES / 2 - (int32_t)rtY_motor.DC_phaB * FOC_PWM_RES / FOC_CTRL_PWM_RES,
                                   margin, FOC_PWM_RES - margin);
    TIM1->CCR3 = (uint16_t)clamp16(FOC_PWM_RES / 2 - (int32_t)rtY_motor.DC_phaC * FOC_PWM_RES / FOC_CTRL_PWM_RES,
                                   margin, FOC_PWM_RES - margin);

    /* Outputs off when disabled: with the fixed idle states every MOSFET is off (coasting). */
    TIM_CtrlPWMOutputs(TIM1, enable ? ENABLE : DISABLE);

    /* 5. Telemetry for the RemoteUartBus answer. */
    realspeed = rtY_motor.n_mot;   // [rpm]
    int8_t hallPos = rtConstP.vec_hallToPos_Value[(hallA << 2) | (hallB << 1) | hallC];
    if (hallPosPrev >= 0 && hallPos != hallPosPrev) {
        int8_t d = hallPos - hallPosPrev;
        if (d == 1 || d == -5) {
            iOdom++;
        } else if (d == -1 || d == 5) {
            iOdom--;
        }
    }
    hallPosPrev = hallPos;

    /* 6. Diagnostics. */
    foc_isrCount++;
    uint16_t tEnd = timerTrianglePos();
    uint16_t elapsed = (uint16_t)((tEnd + 2 * FOC_PWM_RES - tStart) % (2 * FOC_PWM_RES));
    if (elapsed > foc_maxIsrTicks) {
        foc_maxIsrTicks = elapsed;
    }
}

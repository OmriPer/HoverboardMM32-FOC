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
    uint8_t hallA = digitalRead(HALLAPIN) ? 1 : 0;
    uint8_t hallB = digitalRead(HALLBPIN) ? 1 : 0;
    uint8_t hallC = digitalRead(HALLCPIN) ? 1 : 0;
    if (FOC_HALL_INVERT) {
        hallA ^= 1; hallB ^= 1; hallC ^= 1;
    }

    int32_t curA = ((int32_t)offsetA - adcPhaseA) * FOC_CUR_GAIN_NUM / FOC_CUR_GAIN_DEN;
    int32_t curB = ((int32_t)offsetB - adcPhaseB) * FOC_CUR_GAIN_NUM / FOC_CUR_GAIN_DEN;

    uint8_t enable = (!lowbatperm && !foc_errCode) ? 1 : 0;

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

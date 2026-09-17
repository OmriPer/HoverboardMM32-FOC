#ifdef TARGET_MM32SPIN25
#include "HAL_device.h"                 // Device header
#include "spin25-redefine.h" 
#else
#include "mm32_device.h"                // Device header
#endif
#include "RTE_Components.h"             // Component selection
#include "hal_gpio.h"
#include "hal_rcc.h"
#include "hal_adc.h"           
#include "hal_tim.h"
#include "hal_iwdg.h"
#include "../Inc/delay.h"
#include "../Inc/pinout.h"  
#include "../Inc/initialize.h"
#include "../Inc/uart.h"
#include "../Inc/bldc.h"
#include "../Inc/sim_eeprom.h"
#include "../Inc/hardware.h"
#include "../Inc/calculation.h"

#include "../Inc/ipark.h"
#include "../Inc/FOC_Math.h"
#include "../Inc/pwm_gen.h"
#include "../Inc/PID.h"
#include "../Inc/hallhandle.h"
#include "../Inc/foc_config.h"
#include "../Inc/foc_eferu.h"

uint8_t step=1;//very importatnt to set to 1 or it will not work
uint32_t millis;
uint32_t lastCommutation;
uint32_t lastupdate;
uint32_t lastflicker;
uint32_t iOdom;
bool uart;
bool adc;
bool comm=1;
int8_t dir=1;
uint8_t uartBuffer=0;
u8 sRxBuffer[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int vbat;
int itotal;
int fvbat;
int fitotal;
uint8_t hallposprev=1;
int speed=0;
int pwm=0;
int realspeed=0;
int frealspeed=0;
uint8_t  wState;
extern u32 SystemCoreClock;
uint8_t flicker=0;
uint8_t uarten=1;
uint8_t halltimen=1;
extern uint8_t lowbatcount;
extern uint8_t lowbatperm;
////////////////////////////////////////////////////////////////////////////////////////////
//  compile device specefic firmware for mass produce
//  change EEPROMEN to 0
//  EEPROMEN 0: the pinstorage initializer below is the whole configuration.
//  The flash settings page is not read at all, so a config saved by PinFinder
//  (or leftover flash contents) cannot silently override it.
#define EEPROMEN 0
/*
 * Pin configuration for the MM32SPIN27PF master board, PCB "H217776A-JK 2021-05-11".
 * Every value below was measured on this board; see the notes per entry.
 * Pin values are indices into pins[] (names from Inc/hardware.h), 0xFFFF = not present.
 */
uint16_t pinstorage[64]={
	PB8,     // [0]  HALLA   hall sensor A. Order confirmed by PinFinder hall detection with the motor driven.
	PB4,     // [1]  HALLB
	PB9,     // [2]  HALLC   (PB4/PB8/PB9 all on TIM3 for hall speed sensing)
	PA15,    // [3]  LEDR    PA15 lights the LEDs (pulse test). No other LED pins found, so G/B are unset:
	0xFFFF,  // [4]  LEDG    the battery-colour display in main() needs all three and is skipped.
	0xFFFF,  // [5]  LEDB
	0xFFFF,  // [6]  LEDU    not used by this firmware
	0xFFFF,  // [7]  LEDD    not used by this firmware
	PC14,    // [8]  BUZZER  pulse test: buzzer sounds when driven high
	PB11,    // [9]  BUTTON  active high. Board has no pull-down, so io_init() enables the internal one.
	PC13,    // [10] LATCH   self-hold. Must be a push-pull output driven high (internal pull-up cannot hold it).
	0xFFFF,  // [11] (unused)
	PA1,     // [12] VBAT    ADC ch1. Reads 35.7 V at 36 V supply with divider 31.
	0xFFFF,  // [13] ITOTAL  none: no ADC channel responds to total current on this board.
	PD0,     // [14] TX      UART1 TX1 header (verified with UartTest firmware)
	PD1,     // [15] RX      UART1 RX1 header
	PA4,     // [16] IPHASEA phase current amp (0.39 V offset, bidirectional). Only used by FOC; not calibrated.
	PB0,     // [17] IPHASEB phase current amp, pair of PA4.
	0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,   // [18]-[22] unused
	0xFFFF,  // [23] OCP     no overcurrent comparator output found (PA7 pull-low test: no pin responded)
	0xFFFF,  // [24] OCPREF
	0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,   // [25]-[31] unused
	0xDCAB,  // [32] MAGIC_NUMBER (only checked when EEPROMEN is 1)
	31,      // [33] VBAT_DIVIDER   calibrated: 31*36/35.7 = 31.3
	0,       // [34] ITOTAL_DIVIDER 0 = itotal always 0. There is no ITOTAL pin, and analogRead() on a
	         //      missing pin returns garbage that could trip SOFT_ILIMIT.
	0,       // [35]
	19200,   // [36] BAUD
	3000,    // [37] PWM_RES        TIM1 period: 96MHz/2/3000 = 16 kHz, the rate the EFeru controller is tuned for
	         //      (FOC_PWM_RES in Inc/foc_config.h must match).
	1,       // [38] SLAVE_ID       RemoteUartBus address: commands for other ids are ignored, answers carry this id
	30,      // [39] WINDINGS
	1,       // [40] INVERT_LOWSIDE 1: EG2123A gate driver LIN input is active low (datasheet + motor test)
	65535,   // [41] SOFT_ILIMIT    irrelevant while ITOTAL_DIVIDER is 0
	0,       // [42] AWDG           0: analog watchdog would watch the missing ITOTAL channel
	1,       // [43]
	2,       // [44] DRIVEMODE      with FOC_EFERU (Inc/foc_config.h) mapped onto the EFeru controller:
	         //      0/1 commutation, 2/3 sinusoidal (voltage), 4 FOC voltage, 5 FOC speed, 6 FOC torque.
	         //      2 for first bring-up: it does not depend on the (not yet calibrated) phase currents.
	42000,   // [45] BAT_FULL       mV
	32000,   // [46] BAT_EMPTY      mV. Below this for 10 checks with the motor stopped, the motor is disabled
	         //      until reboot and the buzzer beeps. Keep a bench supply above 32 V.
	1000,    // [47] SERIAL_TIMEOUT [ms] without a valid command before the speed command drops to 0
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0             // [48]-[63]
};
////////////////////////////////////////////////////////////////////////////////////////////


s32 main(void){
	DELAY_Init();
	if(EEPROMEN&&!restorecfg()){    //EEPROMEN checked first: with 0 the flash config is never loaded    //if data in eeprom is not valid, do not boot up
		RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOA, ENABLE);
		RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOB, ENABLE);
		RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOC, ENABLE);
		RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOD, ENABLE);
		while(1){
			for(uint8_t i=0;i<33;i++){
				if(i!=10&&i!=11){
					pinMode(i, INPUT_PULLUP);
				}
			}	
			DELAY_Ms(1000);
			for(uint8_t i=0;i<33;i++){
				if(i!=10&&i!=11){
					pinMode(i, INPUT_PULLDOWN);
				}
			}	
			DELAY_Ms(1000);
		}
	}
	
	//initialize normal gpio
	io_init();
	//hall gpio init
	HALL_Init();
	//hall timer init
	halltimen=HALLTIM_Init(65535, SystemCoreClock/1000000);//sysclock is 72mhz
	//initialize 6 bldc pins
	BLDC_init();
	//initialize timer
	TIM1_init(PWM_RES, 0);
#if FOC_EFERU
	FOC_Init();    //EFeru controller; keeps outputs off until phase current offsets are calibrated
#endif
	//systick config
	//timer1 commutation interrupt config
	NVIC_Configure(TIM1_BRK_UP_TRG_COM_IRQn, 1);
	//vbat
	adc_Init();
	//adc interrupt
	NVIC_Configure(ADC_COMP_IRQn, 0);
	#ifdef WATCHDOG//watchdog
	Iwdg_Init(IWDG_Prescaler_32, 0xff);
	#endif
	//serial1.begin(19200);
	uarten=UART_GPIO_Init();
	UARTX_Init((uint32_t)BAUD,uarten);
	//uart interrupt
	//uart dma
	if(uarten==1){
		DMA_NVIC_Config(DMA1_Channel3, (u32)&UART1->RDR, (u32)sRxBuffer, 1);
		NVIC_Configure(DMA1_Channel2_3_IRQn, 1);
	}else{
		DMA_NVIC_Config(DMA1_Channel5, (u32)&UART2->RDR, (u32)sRxBuffer, 1);
		NVIC_Configure(DMA1_Channel4_5_IRQn, 1);
	}
	//latch on power
	if(LATCHPIN<PINCOUNT){    //have latch
		DELAY_Ms(100);    //some board the micro controller can reset in time and turn back on
		digitalWrite(LATCHPIN, 1);
		while(digitalRead(BUTTONPIN)&&BUTTONPIN<PINCOUNT){    //wait while release button
			if(BUZZERPIN<PINCOUNT){    //have buzzer
				digitalWrite(BUZZERPIN, 1);
				DELAY_Ms(2);
				digitalWrite(BUZZERPIN, 0);
				DELAY_Ms(2);
			}else{
				__NOP();
				__NOP();
			}
			IWDG_ReloadCounter();
		}	
	}
	if(BUZZERPIN<PINCOUNT){
		for(int i=0;i<5;i++){    //power on melody
			digitalWrite(BUZZERPIN, 1);
			DELAY_Ms(50);
			digitalWrite(BUZZERPIN, 0);
			DELAY_Ms(50);
		}
	}
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);    //Software start conversion
	UART_SendString("hello World\n\r");    //debug uart
	
	PWM_GEN_init(&pwm_gen);
	InitNormalization(300,4000,4000,&RP);
	HALLModuleInit(&HALL1);
	InitPI();
	MovingAvgInit(&IDData);
	MovingAvgInit(&IQData);
	MovingAvgInit(&SpeedFdk);
	PID_Init();
  while(1) {
		if(LEDRPIN<PINCOUNT&&LEDGPIN<PINCOUNT&&LEDBPIN<PINCOUNT){
			uint16_t bat_0 = BAT_EMPTY;
			uint16_t bat_10 = BAT_EMPTY+((float)(BAT_FULL-BAT_EMPTY)/100*10);
			uint16_t bat_20 = BAT_EMPTY+((float)(BAT_FULL-BAT_EMPTY)/100*20);
			uint16_t bat_50 = BAT_EMPTY+((float)(BAT_FULL-BAT_EMPTY)/100*50);
			uint16_t bat_60 = BAT_EMPTY+((float)(BAT_FULL-BAT_EMPTY)/100*60);
			uint16_t bat_70 = BAT_EMPTY+((float)(BAT_FULL-BAT_EMPTY)/100*70);
			uint16_t bat_100 = BAT_FULL;
			if(BAT_FULL>20000&&BAT_FULL<65000&&BAT_EMPTY>20000&&BAT_EMPTY<65000&&BAT_FULL>BAT_EMPTY){
				if(vbat*10>bat_70){
					digitalWrite(LEDRPIN,0);
					digitalWrite(LEDGPIN,1);
					digitalWrite(LEDBPIN,0);
				}else if(vbat*10>bat_60){
					digitalWrite(LEDRPIN,0);
					digitalWrite(LEDGPIN,1);
					digitalWrite(LEDBPIN,1);
				}else if(vbat*10>bat_50){
					digitalWrite(LEDRPIN,0);
					digitalWrite(LEDGPIN,0);
					digitalWrite(LEDBPIN,1);	
				}else if(vbat*10>bat_20){
					digitalWrite(LEDRPIN,1);
					digitalWrite(LEDGPIN,0);
					digitalWrite(LEDBPIN,1);
				}else if(vbat*10>bat_10){
					digitalWrite(LEDRPIN,1);
					digitalWrite(LEDGPIN,0);
					digitalWrite(LEDBPIN,0);
				}else{
					digitalWrite(LEDRPIN,flicker);
					digitalWrite(LEDGPIN,0);
					digitalWrite(LEDBPIN,0);
				}
			}else{
				digitalWrite(LEDRPIN,digitalRead(HALLAPIN));
				digitalWrite(LEDGPIN,digitalRead(HALLBPIN));
				digitalWrite(LEDBPIN,digitalRead(HALLCPIN));
			}
		}
		if(BUZZERPIN<PINCOUNT){
			if(lowbatcount>=10||lowbatperm){
				digitalWrite(BUZZERPIN,flicker);
			}else{
				digitalWrite(BUZZERPIN,0);
			}
		}
		if(millis-lastupdate>1){//speed pid loop
			speedupdate();
		  lastupdate=millis;
		}
		if(millis-lastflicker>700){//speed pid loop
			flicker=!flicker;
		  lastflicker=millis;
		}
		if(LATCHPIN<PINCOUNT&&BUTTONPIN<PINCOUNT){
			if(digitalRead(BUTTONPIN)){    //button press for shutdown
				TIM1->CCR1=0;    //shut down motor
				TIM1->CCR2=0;
				TIM1->CCR3=0;
				if(BUZZERPIN<PINCOUNT){
					for(int i=0;i<3;i++){    //power off melody
						digitalWrite(BUZZERPIN, 1);
						DELAY_Ms(150);
						digitalWrite(BUZZERPIN, 0);
						DELAY_Ms(150);
					}
				}
				while(digitalRead(BUTTONPIN)) {    //wait for release
					__NOP();
					IWDG_ReloadCounter();
				}
				digitalWrite(LATCHPIN, 0);    //last line to ever be executed
				while(1);//incase the hardware failed...
			}
		}
		IWDG_ReloadCounter();    //feed the watchdog
  }//main loop	
}


#ifndef CONFIG_H
#define CONFIG_H

#define BIDIRECTIONAL
#define STEP_DIR
//#define UART_COMM
#define USB_COMM

#define APB2_CLK					rcc_ahb_frequency
#define APB1_CLK					rcc_apb1_frequency
#define TIM4_CLK					APB1_CLK

#define PWM_FREQ					20000
#define PWM_WIDTH				8
#define PWM_MAX_VAL				((uint16_t) ((1 << PWM_WIDTH) - 1))

#define PWM1_P					TIM4_CCR3
#define PWM1_N					TIM4_CCR4
#define PWM2_P					TIM4_CCR1
#define PWM2_N					TIM4_CCR2

#define ENCODER_0				TIM2_CNT
#define ENCODER_1				TIM3_CNT

#define MAIN_SPEED_UP			GPIO12
#define MAIN_SPEED_DOWN			GPIO13
#define SCROLL_SPEED_UP			GPIO14
#define SCROLL_SPEED_DOWN		GPIO15
#define ALL_SPEED_STOP			GPIO8

#define STEP_0_PIN				GPIO12
#define DIR_0_PIN				GPIO13
#define STEP_1_PIN				GPIO14
#define DIR_1_PIN				GPIO15

#endif /* CONFIG_H */


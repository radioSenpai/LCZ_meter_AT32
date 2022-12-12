#ifndef INT_DRIVER_H_INCLUDED
#define INT_DRIVER_H_INCLUDED

/*---------------------------------------------------------------------------*/

#include "common_head.h"
#include "board.h"
#include "interrupt_manager.h"

/*---------------------------------------------------------------------------*/

#if !defined(TICKS_PER_SECOND)
#define TICKS_PER_SECOND 200
#endif

/*---------------------------------------------------------------------------*/
//common for drivers

typedef enum {
    status_all_right=0,status_inv_param,status_no_send,
    status_no_receive,status_interface_fail
} status_t;

//deprecated use enum values
static const status_t all_right=0;
static const status_t problem_init=1;
static const status_t problem_send=2;
static const status_t problem_rcv=3;
static const status_t no_rcv=4;


/*---------------------------------------------------------------------------*/
//internal driver module


typedef uint8_t rstStatus_t ;//deprecated //TODO: declare status as macro

static const rstStatus_t watch_dog=0;
static const rstStatus_t brown_out=1;
static const rstStatus_t exteral_rst=2;
static const rstStatus_t power_on_rst=3;

rstStatus_t getRstStatus (void);

/*---------------------------------------------------------------------------*/

#define BUT_CHECK_SWITCH    (1<<0)
#define BUT_RESET           (1<<1)

void init_gpio(void);
uint8_t read_gpio (uint8_t port,uint8_t gpio);//
uint8_t state_gpio (uint8_t port,uint8_t gpio);//
uint8_t direction_gpio (uint8_t port,uint8_t gpio);//
//0 port a
//1 port b
//2 port c
//3 port d
void wkp_gpio(void);
void stb_gpio(void);

void init_but (void);
uint8_t read_this_but(uint8_t type);

/*---------------------------------------------------------------------------*/

//TWI control macros
#define SEND_START TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWSTA)
#define SEND_STOP TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWSTO)
#define CONTINUE_TXRX TWCR=(1<<TWINT)|(1<<TWEN)
#define ACK_RETURN TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWEA)
#define NACK_RETURN TWCR=(1<<TWINT)|(1<<TWEN)

#define WAIT_UNTIL_TWI_PROGRESS do {} while(!(TWCR&(1<<TWINT)))
#define WAIT_UNTIL_STOP_SEND do {} while ((TWCR&(1<<TWSTO)))

status_t init_i2c(void);
status_t send_data_i2c (uint8_t address,const uint8_t * data,size_t numb);
status_t receive_data_i2c (uint8_t address,uint8_t * data,size_t numb);
status_t send_stop_i2c (void);

/*---------------------------------------------------------------------------*/

status_t init_uart (void);
status_t stb_uart (void);
status_t wku_uart(void);
status_t send_uart(const char data);
status_t rcv_uart (uint8_t * data);
status_t rcv_uart_non_stop (uint8_t * data);

/*---------------------------------------------------------------------------*/
//timers

#define INIT_TIM1_NONE 0x00
#define INIT_TIM1_PULSE_CNT 0x01//not available now
#define INIT_TIM1_ADC_TRIG 0x02
#define INIT_TIM1_PWM 0x03
#define INIT_TIM1_VEL 0x04
#define INIT_TIM1_WITH_OVF_INT 0x80
#define INIT_TIM1_WITH_COMPA_INT 0x40
#define INIT_TIM1_WITH_COMPB_INT 0x20//for future functionality
#define INIT_TIM1_WITH_CAPT_INT 0x10//for future functionality

#define SET_TIM1_MAX_PWM_VALUE 0xff


status_t init_tim1 (uint8_t type);
void set_tim1 (uint16_t compA,uint16_t compB);


void init_tim0 (void);//not work now
void beep (uint8_t beep);//not work now


/*---------------------------------------------------------------------------*/

#define INIT_ADC_NONE 0x00
#define INIT_ADC_COMPB1 0x01
#define INIT_ADC_OVF1 0x02
#define INIT_ADC_SIMPLE 0x03
#define INIT_ADC_WITH_INT 0x80

#define MAX_ADC_VAL 0x3ff//right align
#define MAX_ADC_VAL_LEFT (int16_t)0x7fc0
#define MIN_ADC_VAL_LEFT (int16_t)0xffc0

#define MUX_ADC_VCC (1<<REFS0)
#define MUX_ADC_VREF (1<<REFS0)|(1<<REFS1)

#define MUX_ADC_DIFF_1_11 (1<<MUX4)|(1<<MUX0)
#define MUX_ADC_DIFF_1_01 (1<<MUX4)
#define MUX_ADC_DIFF_10_00 (1<<MUX3)
#define MUX_ADC_DIFF_10_10 (1<<MUX3)|(1<<MUX0)
#define MUX_ADC_DIFF_200_00 (1<<MUX3)|(1<<MUX1)
#define MUX_ADC_DIFF_200_10 (1<<MUX3)|(1<<MUX1)|(1<<MUX0)

#define MUX_ADC_DIFF_1_22 (1<<MUX4)|(1<<MUX3)|(1<<MUX1)
#define MUX_ADC_DIFF_1_32 (1<<MUX4)|(1<<MUX3)|(1<<MUX1)|(1<<MUX0)
#define MUX_ADC_DIFF_10_22 (1<<MUX3)|(1<<MUX2)
#define MUX_ADC_DIFF_10_32 (1<<MUX3)|(1<<MUX2)|(1<<MUX0)
#define MUX_ADC_DIFF_200_22 (1<<MUX3)|(1<<MUX2)|(1<<MUX1)
#define MUX_ADC_DIFF_200_32 (1<<MUX3)|(1<<MUX2)|(1<<MUX1)|(1<<MUX0)

#define MUX_ADC_SING_5 (1<<MUX2)|(1<<MUX0)
#define MUX_ADC_SING_6 (1<<MUX1)|(1<<MUX2)
#define MUX_ADC_SING_7 (1<<MUX1)|(1<<MUX2)|(1<<MUX0)

#define MUX_LEFT_ADJ (1<<ADLAR)//use with differential channel


status_t init_adc (uint8_t type);
void set_mux_adc (uint8_t channel);
void start_adc (void);
void wait_adc (void);
int16_t get_adc (void);

/*---------------------------------------------------------------------------*/
//SPI

#define INIT_SPI_CCB_MODE 4

//LC72131 only
status_t init_spi(uint8_t mode);//0-3 - classical SPI 4- CCB mode (sanyo)
void exchange_data_spi(uint8_t * data,uint8_t bytes,uint8_t mode);

/*---------------------------------------------------------------------------*/

#define GLUCOSE_PORT_INIT \
INDICATOR_CLOCK_DIR|=(1<<INDICATOR_CLOCK_PIN);\
INDICATOR_DATA_DIR&=~(1<<INDICATOR_DATA_PIN);\
INDICATOR_DATA_PORT|=(1<<INDICATOR_DATA_PIN)


#define GLUCOSE_DATA_HIGH \
INDICATOR_DATA_DIR&=~(1<<INDICATOR_DATA_PIN);\
INDICATOR_DATA_PORT|=(1<<INDICATOR_DATA_PIN)

#define GLUCOSE_DATA_LOW \
INDICATOR_DATA_PORT&=~(1<<INDICATOR_DATA_PIN);\
INDICATOR_DATA_DIR|=(1<<INDICATOR_DATA_PIN)


#define GLUCOSE_CLOCK_HIGH \
INDICATOR_CLOCK_PORT|=(1<<INDICATOR_CLOCK_PIN)

#define GLUCOSE_CLOCK_LOW \
INDICATOR_CLOCK_PORT&=~(1<<INDICATOR_CLOCK_PIN)


void init_port_glucose(void);

/*---------------------------------------------------------------------------*/
//TIM 2 (RTC)


void get_time(uint8_t * hours,uint8_t * minutes);//maybe move this to interrupt module?

#define TIM2_INIT_NONE 0x0
#define TIM2_INIT_AS_RTC 0x1
#define TIM2_INIT_WITH_COMP_INT 0x80

void tim2_init (uint8_t type);
uint8_t tim2_get (void);


/*---------------------------------------------------------------------------*/

#endif // INT_DRIVER_H_INCLUDED

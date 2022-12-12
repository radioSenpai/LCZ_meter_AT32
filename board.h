#ifndef BOARD_H_INCLUDED
#define BOARD_H_INCLUDED

/*---------------------------------------------------------------------------*/
//this defines may change

//unused pins

#define UNUSED_PINS_PORTA_MASK (1<<4)
#define UNUSED_PINS_PORTB_MASK (1<<1|1<<2|1<<3)
#define UNUSED_PINS_PORTC_MASK 0
#define UNUSED_PINS_PORTD_MASK (1<<2)

//buttons
#define BUT_SYS_IN PIND//exit_cnt
#define BUT_1_IN PINC//next_cnt
#define BUT_2_IN PINC//previous_ct
#define BUT_3_IN PINC//enter_cnt

#define BUT_SYS_PORT PORTD
#define BUT_1_PORT PORTC
#define BUT_2_PORT PORTC
#define BUT_3_PORT PORTC

#define BUT_SYS_DIR DDRD
#define BUT_1_DIR DDRC
#define BUT_2_DIR DDRC
#define BUT_3_DIR DDRC

#define BUT_SYS_PIN 3
#define BUT_1_PIN 4
#define BUT_2_PIN 3
#define BUT_3_PIN 2


//buzzer
#define BEEP_PORT PORTB

#define BEEP_DIR DDRB

#define BEEP_PIN 3//PORT B

//voltage measure switch (LC meter)
//define what are we measure: source voltage or load voltage
#define VV_SWITCH_PORT PORTB

#define VV_SWITCH_DIR DDRB

#define VV_SWITCH_PIN 0


//low battery voltage pin
#define LBV_IN PINC

#define LBV_PORT PORTC

#define LBV_DIR DDRC

#define LBV_PIN 5


//RTC quartz
#define RTC_PORT PORTC

#define RTC_DIR DDRC

#define RTC_PIN_1 6
#define RTC_PIN_2 7


//ADC input pins
#define ADC_GPI_1_PORT PORTA
#define ADC_GPI_2_PORT PORTA
#define ADC_GPI_3_PORT PORTA

#define ADC_GPI_1_DIR DDRA
#define ADC_GPI_2_DIR DDRA
#define ADC_GPI_3_DIR DDRA

#define ADC_GPI_1_PIN 7 //ADC7
#define ADC_GPI_2_PIN 6 //ADC6
#define ADC_GPI_3_PIN 5 //ADC5


//glucose indicator
#define INDICATOR_DATA_PORT PORTD
#define INDICATOR_CLOCK_PORT PORTD

#define INDICATOR_DATA_DIR DDRD
#define INDICATOR_CLOCK_DIR DDRD

#define INDICATOR_DATA_PIN 7
#define INDICATOR_CLOCK_PIN 6

//SPI
#define SPI_LC7_CS_PORT PORTB

#define SPI_LC7_CS_DIR DDRB

#define SPI_LC7_CS_PIN 4


//LC72131 GPIO
#define LC7_ANALOG_FRONT_POWER 1
#define LC7_CAP_IND 2
#define LC7_CURRENT_SRC 3

//LC72131
#define LC7_PORT_NUMB 1
#define LC7_DO_PIN 6


/*---------------------------------------------------------------------------*/
//no change

//UART
#define UART_TX_PORT PORTD
#define UART_RX_PORT PORTD

#define UART_TX_DIR DDRD
#define UART_RX_DIR DDRD

#define UART_TX_PIN 1//PORT D
#define UART_RX_PIN 0//PORT D

//TWI
#define TWI_DATA_PORT PORTC//TWI aka I2C
#define TWI_CLOCK_PORT PORTC

#define TWI_DATA_DIR DDRC
#define TWI_CLOCK_DIR DDRC

#define TWI_DATA_PIN 1
#define TWI_CLOCK_PIN 0

//SPI
#define SPI_MISO_PORT PORTB
#define SPI_MOSI_PORT PORTB
#define SPI_CLK_PORT PORTB

#define SPI_MISO_DIR DDRB
#define SPI_MOSI_DIR DDRB
#define SPI_CLK_DIR DDRB

#define SPI_MISO_PIN 6
#define SPI_MOSI_PIN 5
#define SPI_CLK_PIN 7

/*---------------------------------------------------------------------------*/

#endif // BOARD_H_INCLUDED

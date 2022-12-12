
#include "int_driver.h"
#include <util/twi.h>
#include <avr/io.h>

rstStatus_t getRstStatus (void)
{
    uint8_t status=MCUCSR&0xf;

    MCUCSR&=~0xf;

    switch (status)
    {
    case 1<<WDRF:
        {
            return power_on_rst;

            break;
        }
    case 1<<BORF:
        {
            return brown_out;

            break;
        }
    case 1<<EXTRF:
        {
            return exteral_rst;

            break;
        }
    case 1<<PORF:
        {
            return power_on_rst;

            break;
        }
    default:
        {
            break;
        }
    }

    return power_on_rst;
}

/*---------------------------------------------------------------------------*/

void init_gpio (void)
{

    PORTA|=UNUSED_PINS_PORTA_MASK;
    PORTB|=UNUSED_PINS_PORTB_MASK;
    PORTC|=UNUSED_PINS_PORTC_MASK;
    PORTD|=UNUSED_PINS_PORTD_MASK;


    return ;
}

uint8_t read_gpio (uint8_t port,uint8_t gpio)
{
    uint8_t retVal=0;

    switch (port)
    {
    case 0:
        {
            if (PINA&(1<<gpio))
            {
                retVal=1;
            }
            break;
        }
    case 1:
        {
            if (PINB&(1<<gpio))
            {
                retVal=1;
            }
            break;
        }
    case 2:
        {
            if (PINC&(1<<gpio))
            {
                retVal=1;
            }
            break;
        }
    case 3:
        {
            if (PIND&(1<<gpio))
            {
                retVal=1;
            }
            break;
        }
    default:
        {
            break;
        }
    }



    return retVal;
}

uint8_t state_gpio (uint8_t port,uint8_t gpio)
{
    uint8_t retVal=0;

    switch (port)
    {
    case 0:
        {
            if (PORTA&(1<<gpio))
            {
                retVal=1;
            }
            break;
        }
    case 1:
        {
            if (PORTB&(1<<gpio))
            {
                retVal=1;
            }
            break;
        }
    case 2:
        {
            if (PORTC&(1<<gpio))
            {
                retVal=1;
            }
            break;
        }
    case 3:
        {
            if (PORTD&(1<<gpio))
            {
                retVal=1;
            }
            break;
        }
    default:
        {
            break;
        }
    }



    return retVal;
}

uint8_t direction_gpio (uint8_t port,uint8_t gpio)
{
    uint8_t retVal=0;

    switch (port)
    {
    case 0:
        {
            if (DDRA&(1<<gpio))
            {
                retVal=1;
            }
            break;
        }
    case 1:
        {
            if (DDRB&(1<<gpio))
            {
                retVal=1;
            }
            break;
        }
    case 2:
        {
            if (DDRC&(1<<gpio))
            {
                retVal=1;
            }
            break;
        }
    case 3:
        {
            if (DDRD&(1<<gpio))
            {
                retVal=1;
            }
            break;
        }
    default:
        {
            break;
        }
    }



    return retVal;
}


void wkp_gpio(void)
{
    #if DISABLE_GPIO==1
    return;
    #endif // DISABLE_GPIO

    ADC_GPI_1_DIR&=~(1<<ADC_GPI_1_PIN);
    ADC_GPI_2_DIR&=~(1<<ADC_GPI_2_PIN);
    ADC_GPI_3_DIR&=~(1<<ADC_GPI_3_PIN);

    return;
}

void stb_gpio(void)
{
    #if DISABLE_GPIO==1
    return;
    #endif // DISABLE_GPIO

    ADC_GPI_1_DIR|=1<<ADC_GPI_1_PIN;
    ADC_GPI_2_DIR|=1<<ADC_GPI_2_PIN;
    ADC_GPI_3_DIR|=1<<ADC_GPI_3_PIN;

    return;
}

/*---------------------------------------------------------------------------*/

void init_but (void)
{

    BUT_SYS_DIR&=~(1<<BUT_SYS_PIN);
    BUT_1_DIR&=~(1<<BUT_1_PIN);
    BUT_2_DIR&=~(1<<BUT_2_PIN);
    BUT_3_DIR&=~(1<<BUT_3_PIN);

    BUT_SYS_PORT|=(1<<BUT_SYS_PIN);
    BUT_1_PORT|=(1<<BUT_1_PIN);
    BUT_2_PORT|=(1<<BUT_2_PIN);
    BUT_3_PORT|=(1<<BUT_3_PIN);

    return ;
}

uint8_t read_this_but(uint8_t type)
{//[bit 0] 0 - real but state , 1 - switch
    //[bit 1] 0 - read state only , 1- read and reset
    static uint8_t last_but_state=0;
    static uint8_t state=0;

    uint8_t but=0,realBut=0;


    if (~BUT_SYS_IN&(1<<BUT_SYS_PIN))
    {
        but|=(1<<0);
    }

    if (~BUT_1_IN&(1<<BUT_1_PIN))
    {
        but|=(1<<1);
    }

    if (~BUT_2_IN&(1<<BUT_2_PIN))
    {
        but|=(1<<2);
    }

    if (~BUT_3_IN&(1<<BUT_3_PIN))
    {
        but|=(1<<3);
    }


    if (type&BUT_CHECK_SWITCH)
    {//switch
        realBut=but&(~last_but_state);
    }
    else
    {//real but state
        realBut=but;
    }

    last_but_state=but;


    if (type&BUT_CHECK_SWITCH)
    {//switch
        state|=realBut;

        realBut=state;
    }

    if (type&BUT_RESET)
    {//reset
        state=0;
    }


    return realBut;
}

/*---------------------------------------------------------------------------*/

status_t init_i2c (void)
{

    TWI_DATA_PORT|=(1<<TWI_DATA_PIN);
    TWI_CLOCK_PORT|=(1<<TWI_CLOCK_PIN);

    //TWI (I2C)
#if (F_CPU/16)<=SET_LIMIT_I2C_BUS_FRQ

    //если не можем работать с предельной частотой
    TWBR=0x00;//то установим максимально возможную

#else

    TWBR=LOW_PART_NUMB(F_CPU/(2*SET_LIMIT_I2C_BUS_FRQ)-8);

#endif
    TWSR&=0b11111100;
    TWCR=(1<<TWEN);

    return status_all_right;

}

status_t send_data_i2c (uint8_t address,const uint8_t * data,size_t numb)
{

    SEND_START;//start

    WAIT_UNTIL_TWI_PROGRESS;

    uint8_t status=(TWSR&0xf8);

    if ((status!=TW_START)&&(status!=TW_REP_START))
    {
        //////////////////////////
        DEBUG_PRINTF(PSTR("no start condition tx\n"));
        DEBUG_HEX(status);
        DEBUG_PRINTF(PSTR("\n"));
        //////////////////////////

        return status_interface_fail;
    }


    TWDR=address&(~0x01);//send address and TX

    CONTINUE_TXRX;

    WAIT_UNTIL_TWI_PROGRESS;

    if ((TWSR&0xf8)!=TW_MT_SLA_ACK)
    {
        //////////////////////////
        DEBUG_PRINTF(PSTR("no ack tx\n"));
        DEBUG_HEX(TWSR&0xf8);
        DEBUG_PRINTF(PSTR("\n"));
        //////////////////////////

        return status_no_send;
    }


    size_t counter=0;

    while (counter<numb)
    {

        TWDR=data[counter];//transmit data
        counter++;

        CONTINUE_TXRX;

        WAIT_UNTIL_TWI_PROGRESS;


        if ((TWSR&0xf8)!=TW_MT_DATA_ACK)
        {
            //////////////////////////
            DEBUG_PRINTF(PSTR("no ack data tx\n"));
            DEBUG_NUMB_PRINT(counter-1);
            DEBUG_HEX(TWSR&0xf8);
            DEBUG_PRINTF(PSTR("\n"));
            //////////////////////////

            return status_no_send;
        }

    }


    return status_all_right;
}

status_t receive_data_i2c (uint8_t address,uint8_t * data,size_t numb)
{

    SEND_START;//start

    WAIT_UNTIL_TWI_PROGRESS;

    uint8_t status=(TWSR&0xf8);

    if ((status!=TW_START)&&(status!=TW_REP_START))
    {
        //////////////////////////
        DEBUG_PRINTF(PSTR("no start cond rx\n"));
        DEBUG_HEX(TWSR&0xf8);
        DEBUG_PRINTF(PSTR("\n"));
        //////////////////////////

        return status_interface_fail;
    }


    TWDR=address|0x01;//send address and rcv

    CONTINUE_TXRX;

    WAIT_UNTIL_TWI_PROGRESS;

    if ((TWSR&0xf8)!=TW_MR_SLA_ACK)
    {
        //////////////////////////
        DEBUG_PRINTF(PSTR("no ack rx\n"));
        DEBUG_HEX(TWSR&0xf8);
        DEBUG_PRINTF(PSTR("\n"));
        //////////////////////////

        return status_no_receive;
    }


    size_t counter=0;

    while (counter<numb)
    {

        if (counter==(numb-1))
        {
            NACK_RETURN;//NACK
        }
        else
        {
            ACK_RETURN;//ACK
        }


        WAIT_UNTIL_TWI_PROGRESS;


        data[counter]=TWDR;//receive data
        counter++;

        if (counter==(numb))
        {
            if ((TWSR&0xf8)!=TW_MR_DATA_NACK)
            {
                //////////////////////////
                DEBUG_PRINTF(PSTR("no not a NACK rx\n"));
                DEBUG_NUMB_PRINT(counter-1);
                DEBUG_HEX(TWSR&0xf8);
                DEBUG_PRINTF(PSTR("\n"));
                //////////////////////////

                return status_interface_fail;
            }
        }
        else
        {
            if ((TWSR&0xf8)!=TW_MR_DATA_ACK)
            {
                //////////////////////////
                DEBUG_PRINTF(PSTR("no not a ACK rx\n"));
                DEBUG_NUMB_PRINT(counter-1);
                DEBUG_HEX(TWSR&0xf8);
                DEBUG_PRINTF(PSTR("\n"));
                //////////////////////////

                return status_interface_fail;
            }
        }
    }


    return status_all_right;
}

status_t send_stop_i2c (void)
{
    SEND_STOP;//send stop

    WAIT_UNTIL_STOP_SEND;

    return status_all_right;
}

/*---------------------------------------------------------------------------*/

status_t init_uart (void)
{
    //UART
    UBRRH=0b01111111&((F_CPU/(16*BAUD_RATE)-1)>>8);
    UBRRL=(F_CPU/(16*BAUD_RATE)-1);

    UCSRA=0;
    UCSRB=(1<<TXEN)|(1<<RXEN);
    UCSRC=(1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0)|(1<<UPM1);

    return status_all_right;

}

status_t stb_uart (void)
{
    UCSRB&=~((1<<TXEN)|(1<<RXEN));

    UART_RX_PORT&=~(1<<UART_RX_PIN);//off pull up resistor on RX pin

    return status_all_right;
}

status_t wku_uart (void)
{
    UART_RX_PORT|=(1<<UART_RX_PIN);//on pull up resistor on RX pin

    UCSRB|=((1<<TXEN)|(1<<RXEN));

    return status_all_right;
}

status_t send_uart(const char data)
{
    while (IFRB(UCSRA,UDRE))
    {
        ;
    }

    UDR=(uint8_t)data;

    return status_all_right;

}

status_t rcv_uart (uint8_t * data)
{
    while (IFRB(UCSRA,RXC))
    {
        ;
    }

    if (UCSRA&((1<<FE)|(1<<DOR)|(1<<PE)))
    {
        *data=UDR;

        return status_interface_fail;
    }

    *data=UDR;

    return status_all_right;

}

status_t rcv_uart_non_stop (uint8_t * data)
{
    if (IFRB(UCSRA,RXC))
    {
        *data='\0';

        return status_no_receive;
    }
    else
    {
        if (UCSRA&((1<<FE)|(1<<DOR)|(1<<PE)))
        {
            *data=UDR;

            return status_interface_fail;
        }

        *data=UDR;
    }

    return status_all_right;
}

/*---------------------------------------------------------------------------*/

status_t init_tim1 (uint8_t type)
{


    TIMSK&=~((1<<OCIE1A)|(1<<TOIE1)|(1<<OCIE1B));//disable interrupts
    TIFR|=1<<ICF1|1<<OCF1A|1<<OCF1B|1<<TOIE1;

    DDRD&=~((1<<5)|(1<<4));//pwm outputs , deprecated here

    switch (type&0x0f)
    {

    case INIT_TIM1_NONE:
        {
            TCCR1A=0;
            TCCR1B=0;//normal mode no clock

            break;
        }
    case INIT_TIM1_PULSE_CNT:
        {//not available now

            return status_inv_param;


            break;
        }
    case INIT_TIM1_ADC_TRIG:
        {//ADC TRIG //not available now
            //ADC trigger for voltage measure

            return status_inv_param;

            break;
        }
    case INIT_TIM1_PWM:
        {//PWM

            TCNT1=0;
            OCR1A=0;
            OCR1B=0;

            TCCR1A=1<<COM1A1|1<<COM1B1|1<<WGM10;
            TCCR1B=1<<WGM12|1<<CS10;

            DDRD|=(1<<5)|(1<<4);//pwm outputs , deprecated here


            break;
        }
    case INIT_TIM1_VEL:
        {

            OCR1A=(F_CPU/(64UL*TICKS_PER_SECOND))-1;
            OCR1B=1;

            TCCR1A=0;
            TCCR1B=(1<<CS11)|(1<<CS10)|(1<<WGM12);//CTC mode 64 prescale

            break;
        }
    default :
        {
            return status_inv_param;

            break;
        }
    }

    if (INIT_TIM1_WITH_OVF_INT&type)
    {
        TIMSK|=(1<<TOIE1);//enable interrupts
    }

    if (INIT_TIM1_WITH_COMPA_INT&type)
    {
        TIMSK|=(1<<OCIE1A);//enable interrupts
    }

    if (INIT_TIM1_WITH_COMPB_INT&type)
    {
        TIMSK|=(1<<OCIE1B);//enable interrupts
    }

    if (INIT_TIM1_WITH_CAPT_INT&type)
    {
        TIMSK|=(1<<TICIE1);//enable interrupts
    }

    return status_all_right;
}

void set_tim1 (uint16_t compA,uint16_t compB)
{
    OCR1A=compA;
    OCR1B=compB;

    return ;
}

/*---------------------------------------------------------------------------*/
//beep

void init_tim0 (void)
{
    return ;
}

void beep (uint8_t beep)
{
    (void)beep;

    return ;
}

/*---------------------------------------------------------------------------*/

status_t init_adc (uint8_t type)
{

    ACSR|=1<<ACD;//disable analog comparator

    ADCSRA=(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);//set maximum division
    SFIOR&=~(1<<ADTS0|1<<ADTS1|1<<ADTS2);

    switch (type&0x0f)
    {
    case INIT_ADC_NONE:
        {// no init
            return status_all_right;
            break ;
        }
    case INIT_ADC_SIMPLE:
        {
            ADCSRA=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);

            break;
        }
    case INIT_ADC_COMPB1:
        {
            ADCSRA=(1<<ADEN)|(1<<ADATE)|(1<<ADPS2)|(1<<ADPS1);

            SFIOR|=(1<<ADTS0)|(1<<ADTS2);//tim 1 compare match B

            break;
        }
    case INIT_ADC_OVF1:
        {//

            ADCSRA=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1);

            SFIOR|=(1<<ADTS1)|(1<<ADTS2);//tim 1 overflow

            break;
        }

    default:
        {
            return status_inv_param;
            break;
        }
    }

    ADCSRA|=(1<<ADSC);

    while (ADCSRA&(1<<ADSC))
    {
        ;
    }

    ADCW;//dummy read

    if (type&INIT_ADC_WITH_INT)
    {
        ADCSRA|=1<<ADIF;
        ADCSRA|=1<<ADIE;
    }

    return status_all_right;
}

void set_mux_adc (uint8_t channel)
{
    ADMUX=channel;

    return ;
}

void start_adc (void)
{

    ADCSRA|=(1<<ADSC);

    return;
}

void wait_adc (void)
{
    while ((ADCSRA&(1<<ADSC)))
    {
        ;
    }

    //ADCSRA|=(1<<ADSC);

    return ;
}

int16_t get_adc (void)
{
    return ADCW;
}

/*---------------------------------------------------------------------------*/

status_t init_spi(uint8_t mode)
{


    if (mode==4)
    {//CCB
        SPI_LC7_CS_PORT&=~(1<<SPI_LC7_CS_PIN);//CS to low
    }
    else
    {//classical
        SPI_LC7_CS_PORT|=(1<<SPI_LC7_CS_PIN);//CS to high
    }


    SPI_MOSI_DIR|=1<<SPI_MOSI_PIN;
    SPI_MISO_PORT|=1<<SPI_MISO_PIN;//pull up resistor enable
    SPI_CLK_DIR|=1<<SPI_CLK_PIN;
    SPI_LC7_CS_DIR|=1<<SPI_LC7_CS_PIN;


    //SPI
    SPCR=(1<<SPE)|(1<<MSTR)|(1<<SPR1)|(1<<DORD);


    return status_all_right;
}

void exchange_data_spi(uint8_t * data,uint8_t bytes,uint8_t mode)
{

    SPI_LC7_CS_PORT&=~(1<<SPI_LC7_CS_PIN);//CS to low

    uint8_t counter;

    for (counter=0;counter<bytes;counter++)
    {

        SPDR=data[counter];

        while (!(SPSR&(1<<SPIF)))
        {
            ;
        }

        data[counter]=SPDR;

        if ((mode==4)&&(counter==0))
        {
            _delay_us(1);

            SPI_LC7_CS_PORT|=(1<<SPI_LC7_CS_PIN);//CS to high

            _delay_us(1);
        }
    }

    if (mode==4)
    {//CCB
        _delay_us(1);
        SPI_LC7_CS_PORT&=~(1<<SPI_LC7_CS_PIN);//CS to low
    }
    else
    {//classical
        SPI_LC7_CS_PORT|=(1<<SPI_LC7_CS_PIN);//CS to high
    }

    return ;
}

/*---------------------------------------------------------------------------*/

void init_port_glucose (void)
{
    #if DISABLE_GPIO==1
    return;
    #endif // DISABLE_GPIO

    GLUCOSE_PORT_INIT;

    return ;
}

void send_start_glucose (void)
{
    #if DISABLE_GPIO==1
    return;
    #endif // DISABLE_GPIO

    GLUCOSE_DATA_HIGH;
    _delay_us(10);
    GLUCOSE_CLOCK_HIGH;
    _delay_us(10);
    GLUCOSE_DATA_LOW;
    _delay_us(10);
    GLUCOSE_CLOCK_LOW;

    return ;
}

void send_stop_glucose(void)
{
    #if DISABLE_GPIO==1
    return;
    #endif // DISABLE_GPIO

    GLUCOSE_DATA_LOW;
    _delay_us(10);
    GLUCOSE_CLOCK_HIGH;
    _delay_us(10);
    GLUCOSE_DATA_HIGH;
    _delay_us(10);
    GLUCOSE_CLOCK_LOW;
    return ;
}

void send_data_glucose (uint_least8_t data)//msb first
{

    #if DISABLE_GPIO==1
    return;
    #endif // DISABLE_GPIO

    uint8_t counter;

    for (counter=0;counter<8;counter++)
    {
        if (data&(1<<(7-counter)))
        {
            GLUCOSE_DATA_HIGH;
        }
        else
        {
            GLUCOSE_DATA_LOW;
        }

        _delay_us(10);
        GLUCOSE_CLOCK_HIGH;
        _delay_us(10);
        GLUCOSE_CLOCK_LOW;
        _delay_us(10);

    }

    GLUCOSE_DATA_HIGH;
    _delay_us(10);
    GLUCOSE_CLOCK_HIGH;
    _delay_us(10);
    GLUCOSE_CLOCK_LOW;
    _delay_us(10);




    return ;
}

/*---------------------------------------------------------------------------*/


void get_time(uint8_t * hours,uint8_t * minutes)
{
    (*hours)=0;
    (*minutes)=0;


    return;
}

void tim2_init (uint8_t type)
{
    (void)type;

    return;
}


/*---------------------------------------------------------------------------*/

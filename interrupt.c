
#include "interrupt_manager.h"

#include <string.h>

#include <avr/interrupt.h>
#include <avr/io.h>

/*---------------------------------------------------------------------------*/

//EMPTY_INTERRUPT(INT0_vect);
//EMPTY_INTERRUPT(INT1_vect);
EMPTY_INTERRUPT(INT2_vect);
//EMPTY_INTERRUPT(TIMER2_COMP_vect);
//EMPTY_INTERRUPT(TIMER2_OVF_vect);
EMPTY_INTERRUPT(TIMER1_CAPT_vect);
//EMPTY_INTERRUPT(TIMER1_COMPA_vect);
//EMPTY_INTERRUPT(TIMER1_COMPB_vect);
//EMPTY_INTERRUPT(TIMER1_OVF_vect);
//EMPTY_INTERRUPT(TIMER0_COMP_vect);
//EMPTY_INTERRUPT(TIMER0_OVF_vect);
EMPTY_INTERRUPT(SPI_STC_vect);
EMPTY_INTERRUPT(USART_RXC_vect);
EMPTY_INTERRUPT(USART_UDRE_vect);
EMPTY_INTERRUPT(USART_TXC_vect);
//EMPTY_INTERRUPT(ADC_vect);
EMPTY_INTERRUPT(EE_RDY_vect);
EMPTY_INTERRUPT(ANA_COMP_vect);
EMPTY_INTERRUPT(TWI_vect);
EMPTY_INTERRUPT(SPM_RDY_vect);

/*---------------------------------------------------------------------------*/

handler_t array_handlers[MAN_MAX_HANDLERS]={{NULL,MAN_NONE_SIGNAL}};


volatile uint16_t long_press_counter=0;
volatile uint8_t long_press=0;
//volatile uint8_t nesting_isr=0;

static void call_handler(uint8_t signal);

ISR(INT0_vect)
{//but

    //no use any UART functions in this interrupt


    _delay_ms(3);


    GICR&=~(1<<INT0);//interrupt (INT0) disable


    return ;
}

ISR(INT1_vect)
{//sys but


    //no use any UART functions in this interrupt

     _delay_ms(3);


    GICR&=~(1<<INT1);//interrupt (INT1) disable


    return ;
}

ISR(TIMER2_COMP_vect)
{//RTC realized in future
    call_handler(MAN_TIM2_CMP_SIGNAL);

    return;
}

ISR(TIMER2_OVF_vect)
{
    call_handler(MAN_TIM2_OVF_SIGNAL);

    return;
}


ISR(TIMER1_COMPA_vect)
{//for real time operations

    call_handler(MAN_TIM1_CMPA_SIGNAL);

    return;
}

ISR(TIMER1_COMPB_vect)
{

    call_handler(MAN_TIM1_CMPB_SIGNAL);

    return ;
}

ISR(TIMER1_OVF_vect)
{

    call_handler(MAN_TIM1_OVF_SIGNAL);

    return ;
}

volatile uint8_t tim0_comp=0;

ISR(TIMER0_COMP_vect)
{//timeout ,time stamp

    tim0_comp=1;

    return ;
}

ISR(TIMER0_OVF_vect)
{
    return;
}

ISR(ADC_vect)
{
    call_handler(MAN_ADC_SIGNAL);

    return ;
}

/*---------------------------------------------------------------------------*/
//

static void call_handler(uint8_t signal)
{

    uint8_t counter;

    for (counter=0;counter<MAN_MAX_HANDLERS;counter++)
    {
        if (array_handlers[counter].signal==signal)
        {
            (*array_handlers[counter].handler)(signal);

            #if MAN_ONE_SIGNAL_HANDLER==1
            break;
            #endif // MAN_ONE_SIGNAL_HANDLER
        }
    }

    return ;
}

static void setup_timer0 (uint8_t set_reset)
{//1 -set 0- reset

    if (set_reset==1)
    {
        tim0_comp=0;

        SFIOR|=(1<<PSR10);

        TCNT0=0;

        OCR0=(F_CPU/(256UL*1000));//1 ms period

        TIFR|=(1<<OCF0);

        TIMSK|=(1<<OCIE0);

        TCCR0|=(1<<WGM01)|(1<<CS02);//CTC mode 256 prescaler
    }

    if (set_reset==0)
    {
        TCCR0&=~((1<<CS02)|(1<<CS01)|(1<<CS00));//disable clock of tim 0

        TIMSK&=~(1<<OCIE0);
    }

    return ;
}

static uint16_t event_wait(uint16_t delay)
{//subject to change

    uint8_t event=0;

    uint8_t sreg=SREG;

    cli();

    while (delay&&(!event))
    {
        //manager_sleep_mode('I');

        //cli();

        if (tim0_comp)
        {//timer

            if (delay!=MAN_INFINITE_TIMEOUT)
            {
                delay--;
            }
            else
            {//never decremented , only events
                ;
            }

            tim0_comp=0;
        }
        else
        {//event
            event=1;
        }

        //sei();

        MCUCR&=~((1<<SM0)|(1<<SM2)|(1<<SM1));
        MCUCR|=(1<<SE);

        sei();
        asm("SLEEP");
        cli();

        MCUCR&=~((1<<SM0)|(1<<SM2)|(1<<SM1)|(1<<SE));

    }

    SREG=sreg;

    return delay;
}


static void setup_ext_interrupt (uint8_t interrupt)
{
    if (interrupt==MAN_WKP_SRC_INT0)
    {
        MCUCR&=~((1<<ISC00)|(1<<ISC01));//low level

        GICR|=(1<<INT0);//interrupt (INT0) enable
        GIFR|=(1<<INTF0);
    }
    if (interrupt==MAN_WKP_SRC_INT1)
    {
        MCUCR&=~((1<<ISC10)|(1<<ISC11));//low level

        GICR|=(1<<INT1);//interrupt (INT1) enable
        GIFR|=(1<<INTF1);
    }
    if (interrupt==MAN_WKP_SRC_INT2)
    {
        ;
    }

    return ;
}


/*---------------------------------------------------------------------------*/

void manager_start (void)
{
    sei();

    return;
}

uint8_t manager_stop (void)
{
    uint8_t sreg=SREG;

    cli();

    return sreg;
}

void manager_retrieve (uint8_t sreg)
{
    SREG=sreg;

    return ;
}

uint8_t manager_sleep_mode (char mode,uint8_t wkp_src)
{

    if (wkp_src==0)
    {//dont let you die
        return 1;
    }

    uint8_t status=0;

    uint8_t deep_sleep_mode=0;

    cli();

    switch (mode)
    {
    case 'I':
        {//IDLE

            MCUCR&=~((1<<SM0)|(1<<SM2)|(1<<SM1));
            MCUCR|=(1<<SE);

            break;
        }
    case 'S':
        {//power save

            setup_ext_interrupt(wkp_src);
            //setup_timer2_interrupt(wkp_src);

            deep_sleep_mode=1;

            MCUCR&=~(1<<SM2);
            MCUCR|=(1<<SE)|(1<<SM0)|(1<<SM1);
            break;
        }
    case 'E':
        {//extended standby

            setup_ext_interrupt(wkp_src);
            //setup_timer2_interrupt(wkp_src);

            deep_sleep_mode=1;

            MCUCR|=(1<<SE)|(1<<SM2)|(1<<SM1)|(1<<SM0);
            break;
        }
    case 'P':
        {//power down

            setup_ext_interrupt(wkp_src);

            deep_sleep_mode=1;

            MCUCR&=~((1<<SM0)|(1<<SM2));
            MCUCR|=(1<<SE)|(1<<SM1);

            break;
        }
    default :
        {
//            MCUCR&=~((1<<SM0)|(1<<SM2)|(1<<SM1));
//            MCUCR|=(1<<SE);

            status=1;

            break;
        }
    }



    if (status==0)
    {
        if (deep_sleep_mode)
        {
            ;
        }

        sei();
        asm("SLEEP");

        MCUCR&=~((1<<SM0)|(1<<SM2)|(1<<SM1)|(1<<SE));

        if (deep_sleep_mode)
        {
            ;
        }
    }
    else
    {
        sei();
    }


    return status;
}


void manager_delay_ms(uint16_t delay)
{
    setup_timer0(1);

    while (delay)
    {
        delay=event_wait(delay);
    }

    setup_timer0(0);

    return ;
}

uint8_t manager_delay_until_event_ms(uint16_t delay,event_t event)
{

    if (delay==0)
    {
        return MAN_TIMEOUT_OCCUR;
    }

    uint8_t status=MAN_NOTHING_OCCUR;


    setup_timer0(1);

    while (status==MAN_NOTHING_OCCUR)
    {

        if (event_wait(1)==0)
        {
            delay--;
        }

        if (delay==0)
        {
            status=MAN_TIMEOUT_OCCUR;
        }
        else
        {
            uint8_t retVal;
            retVal=(*event.get_event)(event.argument);

            if (retVal)
            {
                status=MAN_EVENT_OCCUR;
            }
        }
    }

    setup_timer0(0);


    return status;
}


uint8_t manager_set_handler (void (*handler)(uint8_t),uint8_t signal)
{

    if (signal==MAN_NONE_SIGNAL)
    {
        return 1;
    }

    uint8_t status=0;

    uint8_t counter;

    #if MAN_ONE_SIGNAL_HANDLER==1

    for (counter=0;counter<MAN_MAX_HANDLERS;counter++)
    {
        if (array_handlers[counter].signal==signal)
        {//this signal already have handler

            status=1;
            break;
        }
    }

    #endif // MAN_ONE_SIGNAL_HANDLER

    if (status==0)
    {
        status=1;

        uint8_t sreg=SREG;
        cli();

        for (counter=0;counter<MAN_MAX_HANDLERS;counter++)
        {
            if (array_handlers[counter].signal==MAN_NONE_SIGNAL)
            {
                array_handlers[counter].handler=handler;
                array_handlers[counter].signal=signal;

                status=0;

                break;
            }
        }

        SREG=sreg;
    }

    return status;
}

uint8_t manager_reset_handler (void (*handler)(uint8_t),uint8_t signal)
{

    if (signal==MAN_NONE_SIGNAL)
    {
        return 1;
    }

    uint8_t status=1;

    uint8_t sreg=SREG;
    cli();

    uint8_t counter;

    for (counter=0;counter<MAN_MAX_HANDLERS;counter++)
    {
        if (array_handlers[counter].signal==signal)
        {
            if (array_handlers[counter].handler==handler)
            {
                array_handlers[counter].handler=NULL;
                array_handlers[counter].signal=MAN_NONE_SIGNAL;

                status=0;

                break;
            }
        }
    }

    SREG=sreg;

    return status;
}

/*---------------------------------------------------------------------------*/

uint16_t manager_get_stack_pointer (void)
{
    uint8_t sreg=SREG;
    cli();

    uint16_t stack_pointer=0;

    stack_pointer=(uint16_t)SPH<<8;
    stack_pointer+=SPL;

    SREG=sreg;

    return stack_pointer;
}

/*---------------------------------------------------------------------------*/

static uint8_t queue_send (queue_t * queue_ptr,const void * const data)
{
    if (queue_ptr==NULL)
    {
        return MAN_QUEUE_INV_PARAM;
    }

    if (data==NULL)
    {
        return MAN_QUEUE_INV_PARAM;
    }

    if (queue_ptr->queue_full==1)
    {//overflow
        return MAN_QUEUE_OVERFLOW;
    }
    else
    {
        uint8_t in_index_tmp=queue_ptr->in_index;

        memcpy(&((uint8_t *)(queue_ptr->buffer))[(queue_ptr->element_size)*in_index_tmp],data,queue_ptr->element_size);

        in_index_tmp++;

        if (in_index_tmp>=queue_ptr->numb_of_element)
        {
            in_index_tmp=0;
        }

        if (in_index_tmp==queue_ptr->out_index)
        {//queue is full
            queue_ptr->queue_full=1;
        }

        queue_ptr->in_index=in_index_tmp;

    }

    return MAN_QUEUE_ALL_RIGHT;
}

static uint8_t queue_receive (queue_t * queue_ptr,void * const data)
{

    if (queue_ptr==NULL)
    {
        return MAN_QUEUE_INV_PARAM;
    }

    if (data==NULL)
    {
        return MAN_QUEUE_INV_PARAM;
    }

    uint8_t out_index_tmp=queue_ptr->out_index;

    if ((queue_ptr->in_index==out_index_tmp)&&(queue_ptr->queue_full==0))
    {//queue empty
        return MAN_QUEUE_EMPTY;
    }
    else
    {//new available data

        memcpy(data,&((uint8_t *)(queue_ptr->buffer))[(queue_ptr->element_size)*out_index_tmp],queue_ptr->element_size);

        out_index_tmp++;

        if (out_index_tmp>=queue_ptr->numb_of_element)
        {
            out_index_tmp=0;
        }

        queue_ptr->queue_full=0;
    }

    queue_ptr->out_index=out_index_tmp;


    return MAN_QUEUE_ALL_RIGHT;
}

/*---------------------------------------------------------------------------*/

uint8_t manager_queue_init(queue_t * queue_ptr,void * data, size_t element_size,uint8_t numb_of_element)
{

    if (queue_ptr==NULL)
    {
        return MAN_QUEUE_INV_PARAM;
    }

    uint8_t sreg=SREG;
    cli();

    queue_ptr->in_index=0;
    queue_ptr->out_index=0;
    queue_ptr->queue_full=0;

    queue_ptr->buffer=data;

    queue_ptr->numb_of_element=numb_of_element;
    queue_ptr->element_size=element_size;

    SREG=sreg;

    return MAN_QUEUE_ALL_RIGHT;
}

uint8_t manager_queue_send(queue_t * queue_ptr,const void * data,uint16_t timeout)
{
    uint8_t status=0;

    setup_timer0(1);

    do
    {
        uint8_t sreg=SREG;
        cli();

        status=queue_send (queue_ptr,data);

        SREG=sreg;

        if (status==MAN_QUEUE_OVERFLOW)
        {
            timeout=event_wait(timeout);
        }

    } while ((status==MAN_QUEUE_OVERFLOW)&&(timeout!=0));

    setup_timer0(0);

    return status;
}

uint8_t manager_queue_receive(queue_t * queue_ptr,void * data,uint16_t timeout)
{
    uint8_t status=0;

    setup_timer0(1);

    do
    {
        uint8_t sreg=SREG;
        cli();

        status=queue_receive (queue_ptr,data);

        SREG=sreg;

        if (status==MAN_QUEUE_EMPTY)
        {
            timeout=event_wait(timeout);
        }

    } while ((status==MAN_QUEUE_EMPTY)&&(timeout!=0));

    setup_timer0(0);

    return status;
}

uint8_t manager_queue_send_from_isr(queue_t * queue_ptr,const void * data)
{
    uint8_t status=0;

    status=queue_send (queue_ptr,data);

    return status;
}

uint8_t manager_queue_receive_from_isr(queue_t * queue_ptr,void * data)
{
    uint8_t status=0;

    status=queue_receive (queue_ptr,data);

    return status;
}

/*---------------------------------------------------------------------------*/

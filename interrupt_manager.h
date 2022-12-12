#ifndef INTERRUPT_MANAGER_H_INCLUDED
#define INTERRUPT_MANAGER_H_INCLUDED

/*---------------------------------------------------------------------------*/

#include "common_head.h"

/*---------------------------------------------------------------------------*/
//interrupt module

//settings

#if !defined(MAN_MAX_HANDLERS)
#define MAN_MAX_HANDLERS 10
#endif

#if !defined(MAN_ONE_SIGNAL_HANDLER)
#define MAN_ONE_SIGNAL_HANDLER 1//0 //if 1 - then handler called once even if more then one registered
#endif

/*---------------------------------------------------------------------------*/

#define MAN_NONE_SIGNAL 0

#define MAN_TIM1_CMPA_SIGNAL 1
#define MAN_TIM1_CMPB_SIGNAL 2
#define MAN_TIM1_OVF_SIGNAL 3

#define MAN_ADC_SIGNAL 4

#define MAN_TIM2_CMP_SIGNAL 5
#define MAN_TIM2_OVF_SIGNAL 6


#define MAN_INFINITE_TIMEOUT 0xffff

#define MAN_NOTHING_OCCUR 2
#define MAN_TIMEOUT_OCCUR 0
#define MAN_EVENT_OCCUR 1
#define MAN_INTERRUPT_OCCUR 3


#define MAN_QUEUE_RESET {NULL,0,0,0,0,0}
#define MAN_QUEUE_ALL_RIGHT 0
#define MAN_QUEUE_EMPTY 1
#define MAN_QUEUE_OVERFLOW 2
#define MAN_QUEUE_INV_PARAM 3


#define MAN_SLEEP_IDLE 'I'
#define MAN_SLEEP_STANDBY 'S'
#define MAN_SLEEP_EXT_STANDBY 'E'
#define MAN_SLEEP_POWERDOWN 'P'

//#define MAN_WKP_SRC_All 0//not valid
#define MAN_WKP_SRC_OTHER 0x80

#define MAN_WKP_SRC_INT0 0x1//no use
#define MAN_WKP_SRC_INT1 0x2//no use
#define MAN_WKP_SRC_INT2 0x4//no use

#define MAN_WKP_SRC_TIM2_CMP 0x8//no use
#define MAN_WKP_SRC_TIM2_OVF 0x10//no use


typedef struct//handler_t
{
    void (*handler)(uint8_t);
    uint8_t signal ;
} handler_t;

typedef struct//event_t
{
    uint8_t (*get_event)(uint8_t);
    uint8_t argument;
} event_t;

typedef struct//queue_t
{
    void * buffer;

    size_t element_size;
    uint8_t numb_of_element;

    uint8_t in_index;//if in == out and full is 0, then queue is empty
    uint8_t out_index;

    uint8_t queue_full;
} queue_t;

void manager_start (void);
uint8_t manager_stop (void);
void manager_retrieve (uint8_t sreg);

uint8_t manager_sleep_mode (char mode,uint8_t wkp_src);

void manager_delay_ms(uint16_t delay);//use instead of _delay_ms()
uint8_t manager_delay_until_event_ms(uint16_t delay,event_t event);

uint8_t manager_set_handler (void (*handler)(uint8_t),uint8_t signal);//return 0 if success , 1 if fail
uint8_t manager_reset_handler (void (*handler)(uint8_t),uint8_t signal);//return 0 if success , 1 if fail

/*---------------------------------------------------------------------------*/

uint16_t manager_get_stack_pointer (void);

/*---------------------------------------------------------------------------*/
//queue

uint8_t manager_queue_init(queue_t * queue_ptr,void * data, size_t element_size,uint8_t numb_of_element);
uint8_t manager_queue_send(queue_t * queue_ptr,const void * data,uint16_t timeout);
uint8_t manager_queue_receive(queue_t * queue_ptr,void * data,uint16_t timeout);
uint8_t manager_queue_send_from_isr(queue_t * queue_ptr,const void * data);
uint8_t manager_queue_receive_from_isr(queue_t * queue_ptr,void * data);

/*---------------------------------------------------------------------------*/

#endif // INTERRUPT_MANAGER_H_INCLUDED

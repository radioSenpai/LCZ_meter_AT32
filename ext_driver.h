#ifndef EXT_DRIVER_H_INCLUDED
#define EXT_DRIVER_H_INCLUDED

/*---------------------------------------------------------------------------*/

#include "common_head.h"
#include "board.h"
#include "int_driver.h"
#include "interrupt_manager.h"

/*---------------------------------------------------------------------------*/

#if !defined(EEPROM_CARE_PAGE_WRITE)
#define EEPROM_CARE_PAGE_WRITE 0
#endif

status_t write_eeprom (uint16_t address,const void * data,size_t numb);
status_t read_eeprom (uint16_t address,void * data,size_t numb);

/*---------------------------------------------------------------------------*/

void init_special_io (void);
void wkp_special_io (void);
void stb_special_io (void);

/*---------------------------------------------------------------------------*/

void power_on_analog (void);
void power_off_analog (void);

void curr_src_1m_on (void);
void curr_src_1m_off (void);

void vv_switch(uint8_t src);//0- source voltage , 1-load voltage

status_t cap_ind (uint8_t type);//0- cap ,1- ind

//low battery check
uint8_t lbv_check (void);//0 - battery is normal ,1 - battery is low

/*---------------------------------------------------------------------------*/
//LC72131


//now only LSB first support
#if !defined(LC7_BIT_ORDER)
#define LC7_BIT_ORDER 0//0 for LSB first, 1 for MSB first
#endif

//static memory support only
#if !defined(LC7_DATA_STATIC_MEM)
#define LC7_DATA_STATIC_MEM 1
#endif
//0 - you must provide data memory
//1 - memory will be statically allocate

//LC72131 mode
//please, do not change defines below

#if LC7_BIT_ORDER==0

#define LC7_IN1_MODE 0x28
#define LC7_IN2_MODE 0x29
#define LC7_OUT_MODE 0x2A

#define LC7_BIT_REMAP(x) (x)

#elif LC7_BIT_ORDER==1

#define LC7_IN1_MODE 0x14
#define LC7_IN2_MODE 0x94
#define LC7_OUT_MODE 0x54

#define LC7_BIT_REMAP(x) (7-(x))

#else

#error "invalid bit order selected for LC72131 (LC7_BIT_ORDER)"

#endif // LC7_BIT_ORDER


/*---------------------------------------------------------------------------*/
//IN1 mode ,byte 1
//OSC divider bits (low)

#define LC7_IN1_1_P0 LC7_BIT_REMAP(0)
#define LC7_IN1_1_P1 LC7_BIT_REMAP(1)
#define LC7_IN1_1_P2 LC7_BIT_REMAP(2)
#define LC7_IN1_1_P3 LC7_BIT_REMAP(3)
#define LC7_IN1_1_P4 LC7_BIT_REMAP(4)
#define LC7_IN1_1_P5 LC7_BIT_REMAP(5)
#define LC7_IN1_1_P6 LC7_BIT_REMAP(6)
#define LC7_IN1_1_P7 LC7_BIT_REMAP(7)

/*---------------------------------------------------------------------------*/
//IN1 mode ,byte 2
//OSC divider bits (high)

#define LC7_IN1_2_P8 LC7_BIT_REMAP(0)
#define LC7_IN1_2_P9 LC7_BIT_REMAP(1)
#define LC7_IN1_2_P10 LC7_BIT_REMAP(2)
#define LC7_IN1_2_P11 LC7_BIT_REMAP(3)
#define LC7_IN1_2_P12 LC7_BIT_REMAP(4)
#define LC7_IN1_2_P13 LC7_BIT_REMAP(5)
#define LC7_IN1_2_P14 LC7_BIT_REMAP(6)
#define LC7_IN1_2_P15 LC7_BIT_REMAP(7)

/*---------------------------------------------------------------------------*/
//IN1 mode ,byte 3
// divider control, Fref divider control

#define LC7_IN1_3_SNS LC7_BIT_REMAP(0)//swallow counter set
#define LC7_IN1_3_DVS LC7_BIT_REMAP(1)//swallow counter set
#define LC7_IN1_3_CTE LC7_BIT_REMAP(2)//IF counter start
#define LC7_IN1_3_XS LC7_BIT_REMAP(3)//if 0- then Xtal 4,5MHz ,if 1- then Xtal 7,2 MHz
#define LC7_IN1_3_R0 LC7_BIT_REMAP(4)
#define LC7_IN1_3_R1 LC7_BIT_REMAP(5)
#define LC7_IN1_3_R2 LC7_BIT_REMAP(6)
#define LC7_IN1_3_R3 LC7_BIT_REMAP(7)

/*---------------------------------------------------------------------------*/
//IN2 mode ,byte 1
// I/O control

#define LC7_IN2_1_IOC1 LC7_BIT_REMAP(0)
#define LC7_IN2_1_IOC2 LC7_BIT_REMAP(1)
#define LC7_IN2_1_IO1 LC7_BIT_REMAP(2)
#define LC7_IN2_1_IO2 LC7_BIT_REMAP(3)
#define LC7_IN2_1_BO1 LC7_BIT_REMAP(4)
#define LC7_IN2_1_BO2 LC7_BIT_REMAP(5)
#define LC7_IN2_1_BO3 LC7_BIT_REMAP(6)
#define LC7_IN2_1_BO4 LC7_BIT_REMAP(7)

/*---------------------------------------------------------------------------*/
//IN2 mode ,byte 2
// out data pin control , dead zone control , unlock control

#define LC7_IN2_2_DNC LC7_BIT_REMAP(0)//don`t care
#define LC7_IN2_2_DOC0 LC7_BIT_REMAP(1)
#define LC7_IN2_2_DOC1 LC7_BIT_REMAP(2)
#define LC7_IN2_2_DOC2 LC7_BIT_REMAP(3)
#define LC7_IN2_2_UL0 LC7_BIT_REMAP(4)
#define LC7_IN2_2_UL1 LC7_BIT_REMAP(5)
#define LC7_IN2_2_DZ0 LC7_BIT_REMAP(6)//dead zone
#define LC7_IN2_2_DZ1 LC7_BIT_REMAP(7)//dead zone

/*---------------------------------------------------------------------------*/
//IN2 mode ,byte 3
// if counter control, clock time base, charge pump control, test bits

#define LC7_IN2_3_GT0 LC7_BIT_REMAP(0)
#define LC7_IN2_3_GT1 LC7_BIT_REMAP(1)
#define LC7_IN2_3_TBC LC7_BIT_REMAP(2)
#define LC7_IN2_3_DLC LC7_BIT_REMAP(3)
#define LC7_IN2_3_IFS LC7_BIT_REMAP(4)
#define LC7_IN2_3_TEST0 LC7_BIT_REMAP(5)
#define LC7_IN2_3_TEST1 LC7_BIT_REMAP(6)
#define LC7_IN2_3_TEST2 LC7_BIT_REMAP(7)

/*---------------------------------------------------------------------------*/
//OUT mode ,byte 1
//input port, unlock detector, counter output

#define LC7_OUT_1_I2 LC7_BIT_REMAP(0)//input port
#define LC7_OUT_1_I1 LC7_BIT_REMAP(1)//input port
#define LC7_OUT_1_MBZ LC7_BIT_REMAP(2)//this bit Must Be Zero
#define LC7_OUT_1_UL LC7_BIT_REMAP(3)//unlock detector
#define LC7_OUT_1_C19 LC7_BIT_REMAP(4)
#define LC7_OUT_1_C18 LC7_BIT_REMAP(5)
#define LC7_OUT_1_C17 LC7_BIT_REMAP(6)
#define LC7_OUT_1_C16 LC7_BIT_REMAP(7)

/*---------------------------------------------------------------------------*/
//OUT mode ,byte 2
//counter output

#define LC7_OUT_2_C15 LC7_BIT_REMAP(0)
#define LC7_OUT_2_C14 LC7_BIT_REMAP(1)
#define LC7_OUT_2_C13 LC7_BIT_REMAP(2)
#define LC7_OUT_2_C12 LC7_BIT_REMAP(3)
#define LC7_OUT_2_C11 LC7_BIT_REMAP(4)
#define LC7_OUT_2_C10 LC7_BIT_REMAP(5)
#define LC7_OUT_2_C9 LC7_BIT_REMAP(6)
#define LC7_OUT_2_C8 LC7_BIT_REMAP(7)

/*---------------------------------------------------------------------------*/
//OUT mode ,byte 3
//counter output

#define LC7_OUT_3_C7 LC7_BIT_REMAP(0)
#define LC7_OUT_3_C6 LC7_BIT_REMAP(1)
#define LC7_OUT_3_C5 LC7_BIT_REMAP(2)
#define LC7_OUT_3_C4 LC7_BIT_REMAP(3)
#define LC7_OUT_3_C3 LC7_BIT_REMAP(4)
#define LC7_OUT_3_C2 LC7_BIT_REMAP(5)
#define LC7_OUT_3_C1 LC7_BIT_REMAP(6)
#define LC7_OUT_3_C0 LC7_BIT_REMAP(7)


/*---------------------------------------------------------------------------*/

#if !defined(LC7_XTAL)
#error "you must provide LC7_XTAL definition"
#else
#if LC7_XTAL==4000000UL
    #define LC7_FREF 5000
    #define REF_DIV (1<<LC7_IN1_3_XS)|(1<<LC7_IN1_3_R0)|(1<<LC7_IN1_3_R3)
    #define FREQ_COEF 625/36
#elif LC7_XTAL==4500000UL
    #define LC7_FREF 1000
    #define REF_DIV (1<<LC7_IN1_3_R0)|(1<<LC7_IN1_3_R1)|(1<<LC7_IN1_3_R3)
    #define FREQ_COEF 125/4
#elif LC7_XTAL==6000000UL
    #define LC7_FREF 2500
    #define REF_DIV (1<<LC7_IN1_3_XS)|(1<<LC7_IN1_3_R2)|(1<<LC7_IN1_3_R3)
    #define FREQ_COEF 625/24
#elif LC7_XTAL==7200000UL
    #define LC7_FREF 1000
    #define REF_DIV (1<<LC7_IN1_3_XS)|(1<<LC7_IN1_3_R0)|(1<<LC7_IN1_3_R1)|(1<<LC7_IN1_3_R3)
    #define FREQ_COEF 125/4
#elif LC7_XTAL==8000000UL
    #define LC7_FREF 10000
    #define REF_DIV (1<<LC7_IN1_3_XS)|(1<<LC7_IN1_3_R0)|(1<<LC7_IN1_3_R3)
    #define FREQ_COEF 625/18
#elif
#error "incorrect LC7_XTAL value"
#endif
#endif

/*---------------------------------------------------------------------------*/

typedef struct
{
    uint8_t in_1;
    uint8_t in_2;
    uint8_t unlock;
} lc72131_stat_t;

void lc72131_init (void);
void lc72131_set_frequency(uint32_t frequency,uint8_t band);
//band:
//0 FMIN
//1 AMIN through swallow counter
//2 AMIN direct

uint32_t lc72131_measure_freq(void);
void lc72131_gpio(uint8_t state);
lc72131_stat_t lc72131_status(void);//not tested
void lc72131_start(void);
void lc72131_stop(void);


/*---------------------------------------------------------------------------*/

#endif // EXT_DRIVER_H_INCLUDED

#ifndef COMMON_HEAD_H_INCLUDED
#define COMMON_HEAD_H_INCLUDED


//also see board.h file for setting

/*---------------------------------------------------------------------------*/
//project settings

//#define DEBUG 1

#define DISABLE_GPIO 0//0- normal working,
//1- all GPIO keep as high-Z, special function GPIO is working normally

/*---------------------------------------------------------------------------*/
//entire device settings

#define BAUD_RATE 38400UL

#define SET_LIMIT_I2C_BUS_FRQ 100000UL

#define USE_EEPROM_CORRECTION 1


/*---------------------------------------------------------------------------*/
//device specific

#define F_CPU 12000000UL

#define __AVR_ATmega32A__

#include <util/delay.h>
#include <avr/pgmspace.h>

/*---------------------------------------------------------------------------*/
//standard library

#include <stdint.h>
#include <stddef.h>

/*---------------------------------------------------------------------------*/
//useful macros

#define SETB(register,bit) (register|=(1<<bit))           /*set bit*/
#define CLRB(register,bit) (register&=(~(1<<bit)))        /*clear bit*/
#define INVB(register,bit) (register^=(1<<bit))           /*invert bit*/
#define IFSB(register,bit) (register&(1<<bit))?1:0        /*if set bit*/
#define IFRB(register,bit) (register&(1<<bit))?0:1        /*if reset bit*/

//be careful , mask must have same length as register
#define SETMASK(register,mask) (register|=mask)
#define CLRMASK(register,mask) (register&=~(mask))
#define INVMASK(register,mask) (register^=mask)

#define LOW_PART_NUMB(N) ((N)&0xFF)
#define HIGH_PART_NUMB(N) (((N)&0xFF00)>>8)

/*---------------------------------------------------------------------------*/
//measure module settings

//measure specific constants
#define CAP_COEF 1.61
#define IND_COEF 0.693

#define PHASE_EXCEED 6//possible values [0 ... 6]

#define NUMBER_OF_CELL 2

/*---------------------------------------------------------------------------*/
//interrupt manager settings

#define MAN_MAX_HANDLERS 10

/*---------------------------------------------------------------------------*/
//extern driver settings

#define EEPROM_CARE_PAGE_WRITE 0

#define LC7_BIT_ORDER 0//0 for LSB first, 1 for MSB first

#define LC7_DATA_STATIC_MEM 1//static memory support only
//0 - you must provide data memory (not realized)
//1 - memory will be statically allocate

#define LC7_XTAL 4000000UL
//possible values :
// 4MHz (5kHz fref),
// 4.5MHz (1kHz fref),
// 6MHz (2.5kHz fref),
// 7.2MHz (1kHz fref),
// 8MHz (10kHz fref),

/*---------------------------------------------------------------------------*/
//internal driver settings

#define TICKS_PER_SECOND 200

/*---------------------------------------------------------------------------*/
//DEBUG

#if defined(DEBUG)

extern void writeSTR(const __flash char * const);
extern void writeNumber (int32_t number);
extern void writeHex (uint32_t numberHex);

#define DEBUG_PRINTF(x) writeSTR(x)
#define DEBUG_NUMB_PRINT(x) writeNumber(x)
#define DEBUG_HEX(x) writeHex(x)

#else

#define DEBUG_PRINTF(x) (void)(x)
#define DEBUG_NUMB_PRINT(x) (void)(x)
#define DEBUG_HEX(x) (void)(x)

#endif


/*---------------------------------------------------------------------------*/

#endif // COMMON_HEAD_H_INCLUDED

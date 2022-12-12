#ifndef MEASURE_H_INCLUDED
#define MEASURE_H_INCLUDED

#include "common_head.h"
#include "ext_driver.h"
#include "int_driver.h"

/*---------------------------------------------------------------------------*/

#if !defined(PHASE_EXCEED)
#define PHASE_EXCEED 6//for DDS generator (also for oxide,
                            //inductor and impedance measure )
                            //possible values [0 ... 6]
#endif
//this parameter define size of phase (8+PHASE_EXCEED bits)

#if !defined(CAP_COEF)
#define CAP_COEF 1.61
#endif

#if !defined(IND_COEF)
#define IND_COEF 0.693
#endif

#if !defined(NEGATIVE_FEEDBACK_GAIN)
#define NEGATIVE_FEEDBACK_GAIN 16
#endif

#if !defined(NUMBER_OF_CELL)
#define NUMBER_OF_CELL 2
#endif

#if !defined(MEASURE_CAPACITY_OF_BATTERY)
#define MEASURE_CAPACITY_OF_BATTERY 1
#endif

#if !defined(MEASURE_ENERGY_OF_BATTERY)
#define MEASURE_ENERGY_OF_BATTERY 0//not tested ,may cause problems
#endif


/*---------------------------------------------------------------------------*/
//common for all initialize functions in this module

#define MEASURE_INIT 1
#define MEASURE_DEINIT 0

/*---------------------------------------------------------------------------*/

typedef float math_type_t;

typedef struct//hard_calib_t
{
    math_type_t curr_src_1m;// real_current

    math_type_t pin_resistance[3];//for impedance measure
    math_type_t pin_reactance[3];//for impedance measure
    //0- at 100Hz ,1-at 1000 and 2- 10000

    math_type_t parasitic_cap;//for low cap measure

    math_type_t cap_coef;
    math_type_t ind_coef;

} hard_calib_t;

#define HARD_CALIB_DEFAULT {0.001,{0,0,0},{0,0,0},0,CAP_COEF,IND_COEF}


typedef hard_calib_t calibration_t;//deprecated

static const calibration_t no_calib=HARD_CALIB_DEFAULT;//deprecated

/*---------------------------------------------------------------------------*/
//oxide capacitor measure

typedef struct//oxide_value_t
{
    math_type_t capacitance;
    math_type_t esr;
    uint8_t over_cap;
} oxide_value_t;

void init_oxide_cap_measure(uint8_t type);
oxide_value_t oxide_cap_measure (uint16_t frequency,const hard_calib_t * calib);

/*---------------------------------------------------------------------------*/
//inductance measure

typedef struct//inductance_value_t
{
    math_type_t inductance;
    math_type_t esr;
    uint8_t over_ind;
} inductance_value_t;

void init_inductance_measure (uint8_t type);
inductance_value_t inductance_measure (uint16_t frequency,const hard_calib_t * calib);

/*---------------------------------------------------------------------------*/
//impedance measure

typedef struct//impedance_value_t
{
    math_type_t real;
    math_type_t imag;
    uint8_t over;
} impedance_value_t;

void init_impedance_measure (uint8_t type);
impedance_value_t impedance_measure (uint16_t freq,const hard_calib_t * calib);

/*---------------------------------------------------------------------------*/
//DDS generator

void init_dds_generator(uint8_t type);
void dds_generator (uint16_t freq);

/*---------------------------------------------------------------------------*/
//frequency measure

void init_frequency_measure (uint8_t type);
uint32_t frequency_measure (void);

/*---------------------------------------------------------------------------*/
//low capacitance measure

typedef struct//low_cap_value_t
{
    math_type_t capacitance;
    uint8_t over_cap;
} low_cap_value_t;

void init_low_cap_measure(uint8_t type);
low_cap_value_t low_cap_measure (const hard_calib_t * calib);

/*---------------------------------------------------------------------------*/
//inductance

typedef struct//inductor_value_t
{
    math_type_t inductance;
    uint8_t over_ind;
} inductor_value_t;

void init_inductor_measure (uint8_t type);
inductor_value_t inductor_measure(const hard_calib_t * calib);


/*---------------------------------------------------------------------------*/
//voltage measure


typedef struct//voltage_type_t
{
    math_type_t voltage;
    uint8_t over_volt;
} voltage_type_t;

void init_voltage_measure(uint8_t type);
voltage_type_t voltage_measure (const hard_calib_t * calib);

/*---------------------------------------------------------------------------*/
//battery measure

typedef struct //battery_type_t
{
    //math_type_t voltage;
    //math_type_t current;

    #if MEASURE_ENERGY_OF_BATTERY==1
    math_type_t energy;
    #endif // MEASURE_ENERGY_OF_BATTERY

    #if MEASURE_CAPACITY_OF_BATTERY==1
    math_type_t capacity;
    #endif // MEASURE_CAPACITY_OF_BATTERY

    uint8_t test_in_progress;
    //uint8_t over;
} battery_type_t;

typedef struct //battery_test_condition_t
{
    math_type_t bound_voltage;
    math_type_t max_current;
    uint8_t cutoff_current_ratio;
} battery_test_condition_t;

typedef enum {battery_keep_test_e=0,battery_test_stop_e,battery_test_start_e} battery_test_control_t;

void init_battery_measure (uint8_t type);
void battery_set_test_condition (battery_test_condition_t * condition,uint8_t cell);
battery_type_t battery_measure (battery_test_control_t control,uint8_t cell);

/*---------------------------------------------------------------------------*/

#endif // MEASURE_H_INCLUDED

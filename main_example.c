#include "head_example.h"

void init(void)
{

    init_gpio();

    init_special_io();
    stb_special_io();

    if (init_uart()!=status_all_right)
    {
        while (1)
        {
            ;
        }
    }

    init_but ();

    //I2C need for EEPROM
    if (init_i2c()!=status_all_right)
    {
        ;
    }

    if (init_adc(INIT_ADC_NONE)!=status_all_right)
    {
        ;
    }

    //SPI need for LC72131
    if (init_spi(INIT_SPI_CCB_MODE)!=status_all_right)
    {
        ;
    }

    lc72131_init();
    lc72131_stop();


    return ;
}

control_t get_but (void)
{
    uint8_t but;
    control_t retVal=nothing_cnt;

    manager_delay_ms(5);

    but=read_this_but(BUT_CHECK_SWITCH|BUT_RESET);

    switch (but)
    {
    case 1<<0:
            {
                retVal=exit_cnt;

                break;
            }
        case 1<<1:
            {
                retVal=next_cnt;

                break;
            }
        case 1<<2:
            {
                retVal=previous_cnt;

                break;
            }
        case 1<<3:
            {
                retVal=enter_cnt;

                break;
            }
    default :
        {
            break;
        }
    }

    return retVal;
}

void catch_value (math_type_t value,const __flash char * const  str)
{
    (void)value;
    (void)str;

    //now you may displaying or send over UART

    return ;
}

int main (void)
{

    init();

    manager_start ();

    hard_calib_t default_calib=HARD_CALIB_DEFAULT;


    control_t control=nothing_cnt;

/*---------------------------------------------------------------------------*/
    //impedance measure example

    init_impedance_measure(MEASURE_INIT);

    do
    {
        control=get_but();

        uint16_t frequency=1000;
        impedance_value_t impedance;

        impedance=impedance_measure (frequency,&default_calib);

        if (impedance.over)
        {
            ;
        }
        else
        {
            catch_value(impedance.real,PSTR("resistance"));
            catch_value(impedance.imag,PSTR("reactance"));
        }

        manager_delay_ms(300);
    } while (control!=exit_cnt);

    init_impedance_measure(MEASURE_DEINIT);

/*---------------------------------------------------------------------------*/
    //capacitance measure example

    init_low_cap_measure(MEASURE_INIT);

    do
    {
        control=get_but();

        low_cap_value_t capacitance;

        capacitance=low_cap_measure (&default_calib);

        if (capacitance.over_cap)
        {//over capacitance
            ;
        }
        else
        {
            catch_value(capacitance.capacitance,PSTR("capacitance"));
        }

        manager_delay_ms(300);
    } while (control!=exit_cnt);

    init_low_cap_measure(MEASURE_DEINIT);

/*---------------------------------------------------------------------------*/
    //DDS generator example

    init_dds_generator(MEASURE_INIT);

    uint16_t frequency=1000;

    do
    {
        control=get_but();

        switch (control)
        {
        case next_cnt:
            {//increase frequency
                if (frequency<=(10000-100))
                {
                    frequency+=100;
                }

                dds_generator(frequency);
                break ;
            }
        case previous_cnt:
            {//decrease frequency

                if (frequency>=(100+100))
                {
                    frequency-=100;
                }

                dds_generator(frequency);
                break ;
            }
        default :
            {
                break ;
            }
        }



        manager_delay_ms(300);
    } while (control!=exit_cnt);

    init_dds_generator(MEASURE_DEINIT);

/*---------------------------------------------------------------------------*/

    while (1)
    {
        ;
    }

    return 0;
}

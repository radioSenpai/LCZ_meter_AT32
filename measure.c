#include "measure.h"

/*---------------------------------------------------------------------------*/
//constants

const math_type_t pi=3.14159;

/*---------------------------------------------------------------------------*/
//tables

typedef struct //quadrature8u_t
{
    uint8_t real;
    uint8_t imag;
} quadrature8u_t;

const __flash quadrature8u_t cosine[]=
//0-0 ,511 - 2*PI
{
    {255 ,128},
{254 ,129},{254 ,131},{254 ,132},{254 ,134},{254 ,135},{254 ,137},
{254 ,138},{254 ,140},{254 ,141},{254 ,143},{253 ,145},{253 ,146},
{253 ,148},{253 ,149},{252 ,151},{252 ,152},{252 ,154},{251 ,155},
{251 ,157},{251 ,158},{250 ,160},{250 ,161},{249 ,163},{249 ,164},
{249 ,166},{248 ,167},{248 ,169},{247 ,170},{247 ,172},{246 ,173},
{245 ,175},{245 ,176},{244 ,178},{244 ,179},{243 ,180},{242 ,182},
{242 ,183},{241 ,185},{240 ,186},{240 ,187},{239 ,189},{238 ,190},
{237 ,191},{236 ,193},{236 ,194},{235 ,195},{234 ,197},{233 ,198},
{232 ,199},{231 ,201},{230 ,202},{230 ,203},{229 ,204},{228 ,206},
{227 ,207},{226 ,208},{225 ,209},{224 ,210},{223 ,212},{222 ,213},
{221 ,214},{219 ,215},{218 ,216},{217 ,217},{216 ,218},{215 ,219},
{214 ,221},{213 ,222},{212 ,223},{210 ,224},{209 ,225},{208 ,226},
{207 ,227},{206 ,228},{204 ,229},{203 ,230},{202 ,230},{201 ,231},
{199 ,232},{198 ,233},{197 ,234},{195 ,235},{194 ,236},{193 ,236},
{191 ,237},{190 ,238},{189 ,239},{187 ,240},{186 ,240},{185 ,241},
{183 ,242},{182 ,242},{180 ,243},{179 ,244},{178 ,244},{176 ,245},
{175 ,245},{173 ,246},{172 ,247},{170 ,247},{169 ,248},{167 ,248},
{166 ,249},{164 ,249},{163 ,249},{161 ,250},{160 ,250},{158 ,251},
{157 ,251},{155 ,251},{154 ,252},{152 ,252},{151 ,252},{149 ,253},
{148 ,253},{146 ,253},{145 ,253},{143 ,254},{141 ,254},{140 ,254},
{138 ,254},{137 ,254},{135 ,254},{134 ,254},{132 ,254},{131 ,254},
{129 ,254},{127 ,254},{126 ,254},{124 ,254},{123 ,254},{121 ,254},
{120 ,254},{118 ,254},{117 ,254},{115 ,254},{114 ,254},{112 ,254},
{110 ,253},{109 ,253},{107 ,253},{106 ,253},{104 ,252},{103 ,252},
{101 ,252},{100 ,251},{98 ,251},{97 ,251},{95 ,250},{94 ,250},
{92 ,249},{91 ,249},{89 ,249},{88 ,248},{86 ,248},{85 ,247},
{83 ,247},{82 ,246},{80 ,245},{79 ,245},{77 ,244},{76 ,244},
{75 ,243},{73 ,242},{72 ,242},{70 ,241},{69 ,240},{68 ,240},
{66 ,239},{65 ,238},{64 ,237},{62 ,236},{61 ,236},{60 ,235},
{58 ,234},{57 ,233},{56 ,232},{54 ,231},{53 ,230},{52 ,230},
{51 ,229},{49 ,228},{48 ,227},{47 ,226},{46 ,225},{45 ,224},
{43 ,223},{42 ,222},{41 ,221},{40 ,219},{39 ,218},{38 ,217},
{37 ,216},{36 ,215},{34 ,214},{33 ,213},{32 ,212},{31 ,210},
{30 ,209},{29 ,208},{28 ,207},{27 ,206},{26 ,204},{25 ,203},
{25 ,202},{24 ,201},{23 ,199},{22 ,198},{21 ,197},{20 ,195},
{19 ,194},{19 ,193},{18 ,191},{17 ,190},{16 ,189},{15 ,187},
{15 ,186},{14 ,185},{13 ,183},{13 ,182},{12 ,180},{11 ,179},
{11 ,178},{10 ,176},{10 ,175},{9 ,173},{8 ,172},{8 ,170},
{7 ,169},{7 ,167},{6 ,166},{6 ,164},{6 ,163},{5 ,161},
{5 ,160},{4 ,158},{4 ,157},{4 ,155},{3 ,154},{3 ,152},
{3 ,151},{2 ,149},{2 ,148},{2 ,146},{2 ,145},{1 ,143},
{1 ,141},{1 ,140},{1 ,138},{1 ,137},{1 ,135},{1 ,134},
{1 ,132},{1 ,131},{1 ,129},{1 ,127},{1 ,126},{1 ,124},
{1 ,123},{1 ,121},{1 ,120},{1 ,118},{1 ,117},{1 ,115},
{1 ,114},{1 ,112},{2 ,110},{2 ,109},{2 ,107},{2 ,106},
{3 ,104},{3 ,103},{3 ,101},{4 ,100},{4 ,98},{4 ,97},
{5 ,95},{5 ,94},{6 ,92},{6 ,91},{6 ,89},{7 ,88},
{7 ,86},{8 ,85},{8 ,83},{9 ,82},{10 ,80},{10 ,79},
{11 ,77},{11 ,76},{12 ,75},{13 ,73},{13 ,72},{14 ,70},
{15 ,69},{15 ,68},{16 ,66},{17 ,65},{18 ,64},{19 ,62},
{19 ,61},{20 ,60},{21 ,58},{22 ,57},{23 ,56},{24 ,54},
{25 ,53},{25 ,52},{26 ,51},{27 ,49},{28 ,48},{29 ,47},
{30 ,46},{31 ,45},{32 ,43},{33 ,42},{34 ,41},{36 ,40},
{37 ,39},{38 ,38},{39 ,37},{40 ,36},{41 ,34},{42 ,33},
{43 ,32},{45 ,31},{46 ,30},{47 ,29},{48 ,28},{49 ,27},
{51 ,26},{52 ,25},{53 ,25},{54 ,24},{56 ,23},{57 ,22},
{58 ,21},{60 ,20},{61 ,19},{62 ,19},{64 ,18},{65 ,17},
{66 ,16},{68 ,15},{69 ,15},{70 ,14},{72 ,13},{73 ,13},
{75 ,12},{76 ,11},{77 ,11},{79 ,10},{80 ,10},{82 ,9},
{83 ,8},{85 ,8},{86 ,7},{88 ,7},{89 ,6},{91 ,6},
{92 ,6},{94 ,5},{95 ,5},{97 ,4},{98 ,4},{100 ,4},
{101 ,3},{103 ,3},{104 ,3},{106 ,2},{107 ,2},{109 ,2},
{110 ,2},{112 ,1},{114 ,1},{115 ,1},{117 ,1},{118 ,1},
{120 ,1},{121 ,1},{123 ,1},{124 ,1},{126 ,1},{128 ,1},
{129 ,1},{131 ,1},{132 ,1},{134 ,1},{135 ,1},{137 ,1},
{138 ,1},{140 ,1},{141 ,1},{143 ,1},{145 ,2},{146 ,2},
{148 ,2},{149 ,2},{151 ,3},{152 ,3},{154 ,3},{155 ,4},
{157 ,4},{158 ,4},{160 ,5},{161 ,5},{163 ,6},{164 ,6},
{166 ,6},{167 ,7},{169 ,7},{170 ,8},{172 ,8},{173 ,9},
{175 ,10},{176 ,10},{178 ,11},{179 ,11},{180 ,12},{182 ,13},
{183 ,13},{185 ,14},{186 ,15},{187 ,15},{189 ,16},{190 ,17},
{191 ,18},{193 ,19},{194 ,19},{195 ,20},{197 ,21},{198 ,22},
{199 ,23},{201 ,24},{202 ,25},{203 ,25},{204 ,26},{206 ,27},
{207 ,28},{208 ,29},{209 ,30},{210 ,31},{212 ,32},{213 ,33},
{214 ,34},{215 ,36},{216 ,37},{217 ,38},{218 ,39},{219 ,40},
{221 ,41},{222 ,42},{223 ,43},{224 ,45},{225 ,46},{226 ,47},
{227 ,48},{228 ,49},{229 ,51},{230 ,52},{230 ,53},{231 ,54},
{232 ,56},{233 ,57},{234 ,58},{235 ,60},{236 ,61},{236 ,62},
{237 ,64},{238 ,65},{239 ,66},{240 ,68},{240 ,69},{241 ,70},
{242 ,72},{242 ,73},{243 ,75},{244 ,76},{244 ,77},{245 ,79},
{245 ,80},{246 ,82},{247 ,83},{247 ,85},{248 ,86},{248 ,88},
{249 ,89},{249 ,91},{249 ,92},{250 ,94},{250 ,95},{251 ,97},
{251 ,98},{251 ,100},{252 ,101},{252 ,103},{252 ,104},{253 ,106},
{253 ,107},{253 ,109},{253 ,110},{254 ,112},{254 ,114},{254 ,115},
{254 ,117},{254 ,118},{254 ,120},{254 ,121},{254 ,123},{254 ,124},
{254 ,126}};

/*---------------------------------------------------------------------------*/
//math operations

typedef struct//complex_t
{
    math_type_t real;
    math_type_t imag;
} complex_t ;

complex_t addQuad(complex_t first,complex_t second)
{
    complex_t retVal;

    retVal.real=first.real+second.real;
    retVal.imag=first.imag+second.imag;

    return retVal;
}

complex_t subQuad (complex_t first,complex_t second)
{
    complex_t retVal;

    retVal.real=first.real-second.real;
    retVal.imag=first.imag-second.imag;

    return retVal;
}

complex_t mulQuad (complex_t first,complex_t second)
{
    complex_t retVal;

    retVal.real=first.real*second.real-first.imag*second.imag;
    retVal.imag=first.imag*second.real+first.real*second.imag;

    return retVal;
}

complex_t divQuad (complex_t first,complex_t second)
{
    complex_t retVal={0,0};

    math_type_t sqrAbs;

    sqrAbs=second.real*second.real+second.imag*second.imag;

    if ((sqrAbs==(math_type_t)0.0)||(sqrAbs==(math_type_t)-0.0))
    {
        return retVal;
    }

    retVal.real=(first.real*second.real+first.imag*second.imag)/sqrAbs;
    retVal.imag=(first.imag*second.real-first.real*second.imag)/sqrAbs;

    return retVal;
}

/*---------------------------------------------------------------------------*/
//oxide capacitance

void init_oxide_cap_measure(uint8_t type)
{
    init_impedance_measure(type);

    return;
}

oxide_value_t oxide_cap_measure (uint16_t frequency,const hard_calib_t * calib)
{

    impedance_value_t impedance;
    impedance=impedance_measure(frequency,calib);

    //for capacitance imagine part of Z is negative
    impedance.imag=-impedance.imag;

    oxide_value_t retVal;

    retVal.esr=impedance.real;
    retVal.over_cap=impedance.over;

    if (impedance.imag!=0)
    {
        retVal.capacitance=1/(2*pi*frequency*impedance.imag);
    }
    else
    {
        retVal.over_cap=1;
    }

    return retVal;
}

/*---------------------------------------------------------------------------*/
//inductance

void init_inductance_measure (uint8_t type)
{
    init_impedance_measure(type);

    return;
}

inductance_value_t inductance_measure (uint16_t frequency,const hard_calib_t * calib)
{

    impedance_value_t impedance;
    impedance=impedance_measure(frequency,calib);


    inductance_value_t retVal;


    retVal.esr=impedance.real;
    retVal.over_ind=impedance.over;

    retVal.inductance=impedance.imag/(2*pi*frequency);


    return retVal;
}

/*---------------------------------------------------------------------------*/

static volatile uint16_t frequency=0;
static volatile uint16_t phase_static=0;


static void reset_phase(void)
{
    uint8_t sreg=manager_stop();
    phase_static=0;
    manager_retrieve(sreg);

    return ;
}

static void set_frequency (uint16_t real_freq)
{
    uint16_t norm_freq;
    norm_freq=(uint64_t)256*(sizeof(cosine)/sizeof(cosine[0]))*real_freq*((uint16_t)1<<PHASE_EXCEED)/F_CPU;

    uint8_t sreg=manager_stop();
    frequency=norm_freq;
    manager_retrieve(sreg);

    return;
}

static void tim_isr (uint8_t signal)
{

    (void)signal;


    uint16_t phase=phase_static;

    uint8_t compA,compB;

    compA=cosine[phase>>PHASE_EXCEED].real;
    compB=cosine[phase>>PHASE_EXCEED].imag;

    set_tim1 (compA,compB);


    phase+=frequency;

    if (phase>=(uint32_t)sizeof(cosine)/sizeof(cosine[0])*((uint16_t)1<<PHASE_EXCEED))
    {
        phase-=(uint32_t)sizeof(cosine)/sizeof(cosine[0])*((uint16_t)1<<PHASE_EXCEED);
    }

    phase_static=phase;


    return;
}

typedef struct//diff_voltage_t
{
    complex_t voltage;
    uint8_t sat;
} diff_voltage_t;

typedef struct//mux_gain_t
{
    uint8_t mux;
    uint8_t mux_bias;
    math_type_t gain;
} mux_gain_t;

static diff_voltage_t get_diff_voltage (const uint8_t src,const uint16_t average)
{// source: 0 - src, 1 - load

    const mux_gain_t mux_gain_src_ld_real [3]=
    {
        {
            MUX_ADC_VCC|MUX_ADC_DIFF_200_32,MUX_ADC_VCC|MUX_ADC_DIFF_200_22,200
        },
        {
            MUX_ADC_VCC|MUX_ADC_DIFF_10_32,MUX_ADC_VCC|MUX_ADC_DIFF_10_22,10
        },
        {
            MUX_ADC_VCC|MUX_ADC_DIFF_1_32,MUX_ADC_VCC|MUX_ADC_DIFF_1_22,1
        }
    };

    const mux_gain_t mux_gain_src_ld_imag [3]=
    {
        {
            MUX_ADC_VCC|MUX_ADC_DIFF_200_10,MUX_ADC_VCC|MUX_ADC_DIFF_200_00,200
        },
        {
            MUX_ADC_VCC|MUX_ADC_DIFF_10_10,MUX_ADC_VCC|MUX_ADC_DIFF_10_00,10
        },
        {
            MUX_ADC_VCC|MUX_ADC_DIFF_1_01,MUX_ADC_VCC|MUX_ADC_DIFF_1_11,-1
        }
    };


    const uint8_t max_index=2;//sizeof(mux_gain_src_ld_real)/sizeof(mux_gain_t)-1;

    set_mux_adc(mux_gain_src_ld_real[max_index].mux|MUX_LEFT_ADJ);

    vv_switch(src);//voltage source
    manager_delay_ms(200);//depend on RC time constant

    diff_voltage_t retVal;

    retVal.sat=0;
    retVal.voltage.real=0;
    retVal.voltage.imag=0;


    uint8_t saturation_detect (int32_t voltage,int16_t sat_level)
    {
        if (voltage>=sat_level)
        {//saturation
            return 1;
        }
        else
        {
            if (voltage<=-sat_level)
            {//saturation
                return 1;
            }
            else
            {
                return 0;
            }
        }

        return 0;
    }

    uint8_t cnt=0;

    for (cnt=0;cnt<2;cnt++)
    {


        uint8_t sat=0;


        const mux_gain_t * mux_gain ;

        if (cnt==0)
        {
            mux_gain=mux_gain_src_ld_real;
        }
        else
        {
            mux_gain=mux_gain_src_ld_imag;
        }

        ///////////////////////////////////

        uint8_t next_band=0;

        uint8_t index=0;

        do
        {
            next_band=0;

            set_mux_adc(mux_gain[max_index-index].mux|MUX_LEFT_ADJ);

            manager_delay_ms(1);

            start_adc();
            wait_adc();

            int16_t voltage=get_adc();

            #if defined(DEBUG)

            DEBUG_PRINTF(PSTR("voltage :"));
            DEBUG_NUMB_PRINT(voltage);
            DEBUG_PRINTF(PSTR("\n"));

            #endif // defined

            sat=saturation_detect(voltage,28000);

            if (sat)
            {
                if (index==0)
                {//over
                    sat=1;
                }
                else
                {
                    index--;

                    set_mux_adc(mux_gain[max_index-index].mux|MUX_LEFT_ADJ);

                    manager_delay_ms(10);

                    sat=0;
                }
            }
            else
            {
                if (index<max_index)
                {
                    if (saturation_detect(voltage*(mux_gain[max_index-index-1].gain/mux_gain[max_index-index].gain),20000))
                    {
                        ;
                    }
                    else
                    {
                        next_band=1;

                        index++;
                    }
                }
                else
                {
                    ;
                }
            }

        } while (next_band);

        math_type_t result;

        int32_t bias=0;

        if (sat)
        {
            result=0;
        }
        else
        {
            int32_t averageVoltage=0;

            uint16_t average_counter=0;


            for (average_counter=0;average_counter<average;average_counter++)
            {
                set_mux_adc(mux_gain[max_index-index].mux|MUX_LEFT_ADJ);

                start_adc();
                wait_adc();

                averageVoltage+=get_adc();


//                set_mux_adc(mux_gain[max_index-index].mux_bias|MUX_LEFT_ADJ);
//
//                start_adc ();
//                wait_adc();
//
//                bias+=get_adc();

            }

            result=(averageVoltage-bias)/
                (mux_gain[max_index-index].gain);


//            for (average_counter=0;average_counter<average;average_counter++)
//            {
//                start_adc();
//                wait_adc();
//
//                int16_t voltage=get_adc();
//
//                averageVoltage+=voltage;
//            }
//
//
//
//            set_mux_adc(mux_gain[max_index-index].mux_bias|MUX_LEFT_ADJ);
//
//            manager_delay_ms(1);
//
//            uint8_t bias_avr_cnt;
//
//            for (bias_avr_cnt=0;bias_avr_cnt<10;bias_avr_cnt++)
//            {
//                start_adc ();
//                wait_adc();
//                bias+=get_adc();
//            }
//
//
//
//            result=(averageVoltage-bias/10*average)/
//                (mux_gain[max_index-index].gain);
//

        }

        #if defined(DEBUG)

            DEBUG_PRINTF(PSTR("voltage bias:"));
            DEBUG_NUMB_PRINT(bias/average);
            //DEBUG_NUMB_PRINT(bias/10);
            DEBUG_PRINTF(PSTR("\n"));

            DEBUG_PRINTF(PSTR("index:"));
            DEBUG_NUMB_PRINT(index);
            DEBUG_PRINTF(PSTR("\n"));
        #endif // defined


        ///////////////////////////////////


        if (cnt==0)
        {
            retVal.voltage.real=result;
        }
        else
        {
            retVal.voltage.imag=result;
        }

        retVal.sat|=sat;

    }


    return retVal;
}

/*---------------------------------------------------------------------------*/
//impedance

void init_impedance_measure (uint8_t type)
{
    if (type==MEASURE_INIT)
    {
        power_on_analog();

        wkp_special_io();

        init_adc(INIT_ADC_SIMPLE);

        manager_set_handler (tim_isr,MAN_TIM1_OVF_SIGNAL);

    }

    if (type==MEASURE_DEINIT)
    {

        manager_reset_handler (tim_isr,MAN_TIM1_OVF_SIGNAL);

        init_adc(INIT_ADC_NONE);

        stb_special_io ();

        power_off_analog ();
    }


    return ;
}

impedance_value_t impedance_measure (uint16_t freq,const hard_calib_t * calib)
{

    reset_phase();


    impedance_value_t retVal={0};

    uint16_t real_freq=freq;

    set_frequency (real_freq);

    ///////////////////////////////////////////////////////////////////

    const uint16_t numbOfSamp=1000;

    complex_t ldAcc={0,0},srcAcc={0,0};


    init_tim1 (INIT_TIM1_PWM|INIT_TIM1_WITH_OVF_INT);


    //manager_delay_ms(100);


    diff_voltage_t voltage;

    voltage=get_diff_voltage(0,numbOfSamp);

    if (voltage.sat)
    {
        retVal.over=1;
    }

    srcAcc=voltage.voltage;


    voltage=get_diff_voltage(1,numbOfSamp);

    if (voltage.sat)
    {
        retVal.over=1;
    }

    ldAcc=voltage.voltage;


    init_tim1 (INIT_TIM1_NONE);



    #if defined(DEBUG)

    DEBUG_PRINTF(PSTR("voltage src:"));
    DEBUG_NUMB_PRINT(srcAcc.real/numbOfSamp);
    //DEBUG_PRINTF(PSTR("\n"));
    DEBUG_NUMB_PRINT(srcAcc.imag/numbOfSamp);
    DEBUG_PRINTF(PSTR("\n"));

    DEBUG_PRINTF(PSTR("voltage ld:"));
    DEBUG_NUMB_PRINT(ldAcc.real/(int32_t)numbOfSamp);
    //DEBUG_PRINTF(PSTR("\n"));
    DEBUG_NUMB_PRINT(ldAcc.imag/(int32_t)numbOfSamp);
    DEBUG_PRINTF(PSTR("\n"));
    #endif // defined


    ///////////////////////////////////////////////////////////////////

    uint8_t index=1;

    switch (freq)
    {
    case 100:
        {
            index=0;
            break;
        }
    case 1000:
        {
            index=1;
            break;
        }
    case 10000:
        {
            index=2;
            break;
        }
    default :
        {
            break;
        }
    }

    complex_t srcImp={50,0};

    complex_t Z;

    Z=divQuad(mulQuad(srcImp,ldAcc),subQuad(srcAcc,ldAcc));

    Z.real-=calib->pin_resistance[index];
    Z.imag-=calib->pin_reactance[index];

    retVal.real=Z.real;
    retVal.imag=Z.imag;


    return retVal;
}

/*---------------------------------------------------------------------------*/
//DDS

void init_dds_generator(uint8_t type)
{
    if (type==MEASURE_INIT)
    {
        power_on_analog();

        phase_static=0;
        frequency=0;

        manager_set_handler (tim_isr,MAN_TIM1_OVF_SIGNAL);

        init_tim1 (INIT_TIM1_PWM|INIT_TIM1_WITH_OVF_INT);

    }

    if (type==MEASURE_DEINIT)
    {
        init_tim1 (INIT_TIM1_NONE);

        manager_reset_handler (tim_isr,MAN_TIM1_OVF_SIGNAL);

        frequency=0;
        phase_static=0;

        power_off_analog ();
    }

    return;
}

void dds_generator (uint16_t freq)
{

    if (frequency>10000)
    {
        ;
    }
    else
    {
        set_frequency (freq);
    }


    return ;
}

/*---------------------------------------------------------------------------*/
//frequency measure

void init_frequency_measure (uint8_t type)
{
    if (type==MEASURE_INIT)
    {
        lc72131_start();
    }
    else
    {
        lc72131_stop();
    }

    return ;
}

uint32_t frequency_measure (void)
{
    uint32_t frequency;

    frequency=lc72131_measure_freq();

    return frequency;
}

/*---------------------------------------------------------------------------*/
//low capacitance

void init_low_cap_measure(uint8_t type)
{
    if (type==MEASURE_INIT)
    {
        power_on_analog();

        if (cap_ind(0)!=all_right)
        {
            DEBUG_PRINTF(PSTR("problem cap_ind\n"));
        }

        lc72131_start();
    }

    if (type==MEASURE_DEINIT)
    {
        lc72131_stop();

        power_off_analog ();
    }

    return;
}

low_cap_value_t low_cap_measure (const hard_calib_t * calib)
{

    low_cap_value_t retVal;

    ///////////////////////////////////////////////////

    const math_type_t coef=CAP_COEF,resistance=1100;

    uint32_t frequency=lc72131_measure_freq();

    //coef is T'/T or F/F'

    if (frequency==0)
    {
        DEBUG_PRINTF(PSTR("over cap!\n"));
        retVal.over_cap=1;
    }
    else
    {
        retVal.capacitance=(math_type_t)1/(2*coef*frequency*resistance)-calib->parasitic_cap;
        retVal.over_cap=0;
    }

    return retVal;
}

/*---------------------------------------------------------------------------*/
//inductance

void init_inductor_measure (uint8_t type)
{
    if (type==MEASURE_INIT)
    {
        power_on_analog();

        if (cap_ind(1)!=all_right)
        {
            DEBUG_PRINTF(PSTR("problem cap_ind\n"));
        }

        lc72131_start();
    }

    if (type==MEASURE_DEINIT)
    {
        lc72131_stop();

        power_off_analog ();
    }

    return;
}

inductor_value_t inductor_measure(const hard_calib_t * calib)
{
    (void)calib;

    inductor_value_t retVal;


    ////////////////////////////////////

    uint32_t frequency=lc72131_measure_freq();

    const math_type_t coef=IND_COEF,resistance=100;

    //coef is T'/T or F/F'

    if (frequency==0)
    {
        DEBUG_PRINTF(PSTR("over inductance!\n"));
        retVal.over_ind=1;
    }
    else
    {
        retVal.inductance=(math_type_t)1*resistance/(2*coef*frequency);
        retVal.over_ind=0;
    }

    return retVal;
}

/*---------------------------------------------------------------------------*/
//voltage measure

void init_voltage_measure(uint8_t type)
{
    if (type==MEASURE_INIT)
    {
        if (init_adc(INIT_ADC_SIMPLE)!=status_all_right)
        {
            #if defined(DEBUG)
            DEBUG_PRINTF(PSTR("cannot init adc\n"));
            #endif // defined
        }

        set_mux_adc(MUX_ADC_VREF|MUX_ADC_SING_6);
    }

    if (type==MEASURE_DEINIT)
    {
        if (init_adc(INIT_ADC_NONE)!=status_all_right)
        {
            #if defined(DEBUG)
            DEBUG_PRINTF(PSTR("cannot init adc\n"));
            #endif // defined
        }
    }

    return ;
}

voltage_type_t voltage_measure (const hard_calib_t * calib)
{

    (void)calib;

    voltage_type_t retVal;

    const math_type_t refVolt=2.56;

    uint32_t average_voltage=0;

    uint8_t sat=0;

    const uint16_t average=1000;
    uint16_t average_cnt;



    for (average_cnt=0;(average_cnt<average)&&(!sat);average_cnt++)
    {
        int16_t tmp_voltage;

        start_adc();
        wait_adc();
        tmp_voltage=get_adc();

        if (tmp_voltage>=(MAX_ADC_VAL-MAX_ADC_VAL/10))
        {
            sat=1;
        }
        else
        {
            sat=0;
        }

        average_voltage+=tmp_voltage;
    }


    retVal.voltage=((math_type_t)average_voltage/((math_type_t)average*(MAX_ADC_VAL+1)))*refVolt;
    retVal.over_volt=sat;


    return retVal;
}

/*---------------------------------------------------------------------------*/

static int16_t abs_val_int (int16_t value)
{
    return value<0?-value:value;
}

static uint32_t abs_val_int_32 (int32_t value)
{
    return value<0?-value:value;
}

typedef struct //battery_parameters_t
{
    int16_t bound_voltage ;
    int8_t max_current ;//also used as max discharge current

    uint8_t test_in_progress;

    uint8_t cutoff_current_ratio;

} battery_parameters_t;


static const volatile uint8_t battery_channel[NUMBER_OF_CELL]={MUX_ADC_VREF|MUX_ADC_SING_7,MUX_ADC_VREF|MUX_ADC_SING_6};

static volatile int8_t compare_values[NUMBER_OF_CELL]={0,0};
static volatile battery_parameters_t battery[NUMBER_OF_CELL]={0};

#if MEASURE_ENERGY_OF_BATTERY==1
static volatile int16_t voltage_of_cell[NUMBER_OF_CELL]={0,0};
#endif // MEASURE_ENERGY_OF_BATTERY

#if MEASURE_ENERGY_OF_BATTERY==1
static volatile uint64_t energy[NUMBER_OF_CELL]={0,0};
#endif // MEASURE_ENERGY_OF_BATTERY

#if MEASURE_CAPACITY_OF_BATTERY==1
static volatile uint32_t capacity[NUMBER_OF_CELL]={0,0};
#endif // MEASURE_CAPACITY_OF_BATTERY

static volatile uint8_t select_cell=0;

static void adc_handler (uint8_t signal)
{
    (void)signal;

    manager_start();//this handler must not be reentrant

    int16_t voltage=0;

    voltage=get_adc();

    #if MEASURE_ENERGY_OF_BATTERY==1
    voltage_of_cell[select_cell]=voltage;
    #endif // MEASURE_ENERGY_OF_BATTERY

    int8_t pwm_value=0;

    if (battery[select_cell].test_in_progress==0)
    {//channel off
        pwm_value=0;
    }
    else
    {

        int16_t current_for_const_voltage=0;
        int8_t max_current=battery[select_cell].max_current;

        current_for_const_voltage=(battery[select_cell].bound_voltage-voltage)*NEGATIVE_FEEDBACK_GAIN;

        static uint8_t timeout_counter[NUMBER_OF_CELL]={0};

        if (abs_val_int(current_for_const_voltage)>(int16_t)max_current)
        {//current limit
            if (current_for_const_voltage>0)
            {
                pwm_value=max_current;
            }
            else
            {
                pwm_value=-max_current;
            }


            timeout_counter[select_cell]=0;
        }
        else
        {//voltage limit

            pwm_value=current_for_const_voltage;


            if (abs_val_int((int16_t)battery[select_cell].cutoff_current_ratio*pwm_value)<(int16_t)max_current)
            {
                if (timeout_counter[select_cell]>=255)
                {
                    battery[select_cell].test_in_progress=0;
                    timeout_counter[select_cell]=0;

                    pwm_value=0;
                }
                else
                {
                    timeout_counter[select_cell]++;
                }
            }
        }
    }

    uint8_t sreg=manager_stop();
    compare_values[select_cell]=pwm_value;
    manager_retrieve(sreg);

    select_cell++;

    if (select_cell>=NUMBER_OF_CELL)
    {
        select_cell=0;
    }

    set_mux_adc(battery_channel[select_cell]);

    manager_stop();
    start_adc();

    return ;
}

static void tim1_handler (uint8_t signal)
{
    (void)signal;

    uint8_t cell=0;

    for (cell=0;cell<NUMBER_OF_CELL;cell++)
    {
        #if MEASURE_ENERGY_OF_BATTERY==1
        energy[cell]+=(int32_t)voltage_of_cell[cell]*compare_values[cell];
        #endif // MEASURE_ENERGY_OF_BATTERY

        #if MEASURE_CAPACITY_OF_BATTERY==1
        capacity[cell]+=abs_val_int_32(compare_values[cell]);
        #endif // MEASURE_CAPACITY_OF_BATTERY
    }

    set_tim1 ((int16_t)compare_values[0]+SET_TIM1_MAX_PWM_VALUE/2,
              (int16_t)compare_values[1]+SET_TIM1_MAX_PWM_VALUE/2);

    return ;
}

void init_battery_measure (uint8_t type)
{
    if (type==MEASURE_INIT)
    {
        power_on_analog();

        uint8_t cell=0;

        for (cell=0;cell<NUMBER_OF_CELL;cell++)
        {
            #if MEASURE_ENERGY_OF_BATTERY==1
            energy[cell]=0;
            #endif // MEASURE_ENERGY_OF_BATTERY

            #if MEASURE_CAPACITY_OF_BATTERY==1
            capacity[cell]=0;
            #endif // MEASURE_CAPACITY_OF_BATTERY

            compare_values[cell]=0;

            battery[cell].test_in_progress=0;
        }

        manager_set_handler(tim1_handler,MAN_TIM1_OVF_SIGNAL);
        manager_set_handler(adc_handler,MAN_ADC_SIGNAL);

        init_adc(INIT_ADC_SIMPLE|INIT_ADC_WITH_INT);
        init_tim1(INIT_TIM1_PWM|INIT_TIM1_WITH_OVF_INT);

        select_cell=0;
        set_mux_adc(battery_channel[0]);

        start_adc();
    }

    if (type==MEASURE_DEINIT)
    {
        init_tim1(INIT_TIM1_NONE);
        init_adc(INIT_ADC_NONE);

        manager_reset_handler(tim1_handler,MAN_TIM1_OVF_SIGNAL);
        manager_reset_handler(adc_handler,MAN_ADC_SIGNAL);

        power_off_analog();
    }

    return ;
}

static const math_type_t delta_t=((math_type_t)SET_TIM1_MAX_PWM_VALUE+1)/F_CPU;
static const math_type_t current_coeff=2*0.224*1/(SET_TIM1_MAX_PWM_VALUE+1)*4.7;//the last constant is power supply voltage
static const math_type_t voltage_coeff=2*2.56/(MAX_ADC_VAL+1);

/*---------------------------------------------------------------------------*/
//battery measure

void battery_set_test_condition (battery_test_condition_t * condition,uint8_t cell)
{

    if (cell>=NUMBER_OF_CELL)
    {
        return ;
    }

    int16_t bound_voltage=condition->bound_voltage/voltage_coeff;
    int16_t max_current=condition->max_current/current_coeff;

    #if defined(DEBUG)

    DEBUG_PRINTF(PSTR("bound voltage :"));
    DEBUG_NUMB_PRINT(bound_voltage);
    DEBUG_PRINTF(PSTR("\n"));

    DEBUG_PRINTF(PSTR("max current:"));
    DEBUG_NUMB_PRINT(max_current);
    DEBUG_PRINTF(PSTR("\n"));

    #endif // defined

    if (max_current>127)
    {
        max_current=127;
    }

    if (max_current<0)
    {
        max_current=0;
    }

    uint8_t sreg=manager_stop();
    battery[cell].bound_voltage=bound_voltage;
    battery[cell].max_current=max_current;
    battery[cell].cutoff_current_ratio=condition->cutoff_current_ratio;
    manager_retrieve(sreg);

    return ;
}

battery_type_t battery_measure (battery_test_control_t control,uint8_t cell)
{

    battery_type_t retVal={0};

    if (cell>=NUMBER_OF_CELL)
    {
        ;
    }
    else
    {
        uint8_t sreg=manager_stop();

        switch (control)
        {
        case battery_keep_test_e:
            {
                break;
            }
        case battery_test_stop_e:
            {

                #if defined(DEBUG)
                DEBUG_PRINTF(PSTR("stop test: "));
                DEBUG_NUMB_PRINT(cell);
                DEBUG_PRINTF(PSTR("cell\n"));
                #endif // defined


                battery[cell].test_in_progress=0;
                break;
            }
        case battery_test_start_e:
            {
                #if defined(DEBUG)
                DEBUG_PRINTF(PSTR("start test: "));
                DEBUG_NUMB_PRINT(cell);
                DEBUG_PRINTF(PSTR("cell\n"));
                #endif // defined

                battery[cell].test_in_progress=1;
                break;
            }
        default :
            {
                break;
            }
        }

        retVal.test_in_progress=battery[cell].test_in_progress;

        #if MEASURE_ENERGY_OF_BATTERY==1
        uint64_t energy_tmp=energy[cell];
        energy[cell]=0;
        #endif // MEASURE_ENERGY_OF_BATTERY

        #if MEASURE_CAPACITY_OF_BATTERY==1
        uint32_t capacity_tmp=capacity[cell];
        capacity[cell]=0;
        #endif // MEASURE_CAPACITY_OF_BATTERY

        (volatile void)manager_retrieve(sreg);


        #if defined(DEBUG)

        #if MEASURE_CAPACITY_OF_BATTERY==1
        DEBUG_PRINTF(PSTR("capacity :"));
        DEBUG_NUMB_PRINT(capacity_tmp);
        DEBUG_PRINTF(PSTR("\n"));
        #endif // MEASURE_CAPACITY_OF_BATTERY

        #endif // defined


        #if MEASURE_CAPACITY_OF_BATTERY==1
        math_type_t real_capacity=(math_type_t)capacity_tmp*current_coeff*delta_t/3600;
        #endif // MEASURE_CAPACITY_OF_BATTERY

        #if MEASURE_ENERGY_OF_BATTERY==1
        math_type_t real_energy=(math_type_t)energy_tmp*current_coeff*voltage_coeff*delta_t/3600;
        #endif // MEASURE_ENERGY_OF_BATTERY


        #if MEASURE_CAPACITY_OF_BATTERY==1
        retVal.capacity=real_capacity;
        #endif // MEASURE_CAPACITY_OF_BATTERY

        #if MEASURE_ENERGY_OF_BATTERY==1
        retVal.energy=real_energy;
        #endif // MEASURE_ENERGY_OF_BATTERY

    }

    return retVal;
}

/*---------------------------------------------------------------------------*/


#include "ext_driver.h"
#include <avr/io.h>

/*---------------------------------------------------------------------------*/

status_t write_eeprom (uint16_t address,const void * data,size_t numb)
{

    size_t index_of_input=0;
    uint8_t buf_index=0;

    const uint8_t dataBlockLength=16;//page write mode for EEPROM

    uint8_t dataBuf[dataBlockLength+1];//data plus low byte of address


    for (index_of_input=0;index_of_input<numb;)
    {

        dataBuf[0]=address&0xff;

        #if EEPROM_CARE_PAGE_WRITE==1

        if (((address%dataBlockLength)!=0)||(numb-index_of_input<dataBlockLength))
        {//if not all page need to be write
            if (read_eeprom (address,&dataBuf[1],dataBlockLength)!=status_all_right)
            {
                return status_no_receive;
            }
        }

        buf_index=(address%dataBlockLength);

        while ((index_of_input<numb)&&(buf_index<(dataBlockLength)))
        {
            dataBuf[buf_index+1]=((uint8_t *)data)[index_of_input];

            index_of_input++;
            buf_index++;
        }

        if (send_data_i2c((address>>7)&0xff,dataBuf,dataBlockLength+1))
        {//send address of byte (address and dataBuf[0]) and data
            return status_interface_fail;
        }

        #else

        buf_index=0;

        while ((index_of_input<numb)&&(buf_index<(16-(address%16))))
        {
            dataBuf[buf_index+1]=((uint8_t *)data)[index_of_input];

            index_of_input++;
            buf_index++;
        }

        if (send_data_i2c((address>>7)&0xff,dataBuf,buf_index+1))
        {//send address of byte (address and dataBuf[0]) and data
            return status_interface_fail;
        }

        #endif // EEPROM_CARE_PAGE_WRITE


        if (send_stop_i2c())
        {
            return status_interface_fail;
        }




        status_t status;

        do
        {//dummy write to poll status
            status=send_data_i2c((address>>7)&0xff,dataBuf,1);

            if (send_stop_i2c())
            {
                return status_interface_fail;
            }

        } while(status==status_no_send);

        if (status==status_interface_fail)
        {
            return status_interface_fail;
        }

        address+=buf_index;
    }


    return status_all_right;
}

status_t read_eeprom (uint16_t address,void * data,size_t numb)
{

    status_t status=status_all_right;

    ((uint8_t *)data)[0]=address&0xff;

    if (send_data_i2c((address>>7)&0xff,data,1)!=status_all_right)
    {//dummy write to set internal counter
        status=status_no_receive;
    }
    else
    {
        if (receive_data_i2c(((address>>7)&0xff),data,numb)!=status_all_right)
        {//sequential read
            status=status_no_receive;
        }
    }

    if (send_stop_i2c()!=status_all_right)
    {//stop
        status=status_no_receive;
    }


    return status;
}

/*---------------------------------------------------------------------------*/

void init_special_io (void)
{

    #if DISABLE_GPIO==1
    return;
    #endif // DISABLE_GPIO

    VV_SWITCH_DIR|=1<<VV_SWITCH_PIN;

    LBV_PORT&=~(1<<LBV_PIN);
    LBV_DIR|=1<<LBV_PIN;

    return;
}

void wkp_special_io (void)
{
    #if DISABLE_GPIO==1
    return;
    #endif // DISABLE_GPIO


    return;
}

void stb_special_io (void)
{
    #if DISABLE_GPIO==1
    return;
    #endif // DISABLE_GPIO

    VV_SWITCH_PORT&=~(1<<VV_SWITCH_PIN);

    LBV_PORT&=~(1<<LBV_PIN);
    LBV_DIR|=1<<LBV_PIN;

    return;
}

/*---------------------------------------------------------------------------*/

static uint8_t lc7_gpio_state=0;

void power_on_analog (void)
{

    lc7_gpio_state|=(1<<LC7_ANALOG_FRONT_POWER);

    lc72131_gpio(lc7_gpio_state);

    return ;
}

void power_off_analog (void)
{

    lc7_gpio_state&=~(1<<LC7_ANALOG_FRONT_POWER);

    lc72131_gpio(lc7_gpio_state);

    return ;
}

/*---------------------------------------------------------------------------*/

void curr_src_1m_on (void)
{
    lc7_gpio_state|=(1<<LC7_CURRENT_SRC);
    lc72131_gpio(lc7_gpio_state);

    return ;
}

void curr_src_1m_off (void)
{

    lc7_gpio_state&=~(1<<LC7_CURRENT_SRC);
    lc72131_gpio(lc7_gpio_state);

    return ;
}

/*---------------------------------------------------------------------------*/

void vv_switch(uint8_t src)
{
    #if DISABLE_GPIO==1
    return;
    #endif // DISABLE_GPIO


    if (src==0)
    {
        VV_SWITCH_PORT&=~(1<<VV_SWITCH_PIN);
    }

    if (src==1)
    {
        VV_SWITCH_PORT|=(1<<VV_SWITCH_PIN);
    }

    return ;
}

status_t cap_ind (uint8_t type)
{//0- cap ,1- ind

    if (type==0)
    {
        lc7_gpio_state&=~(1<<LC7_CAP_IND);
    }
    else
    {
        if (type==1)
        {
            lc7_gpio_state|=(1<<LC7_CAP_IND);
        }
        else
        {
            return status_inv_param;
        }
    }

    lc72131_gpio(lc7_gpio_state);

    return status_all_right;
}

/*---------------------------------------------------------------------------*/
//low battery voltage

uint8_t lbv_check (void)
{
    uint8_t status=0;

    LBV_DIR&=~(1<<LBV_PIN);
    LBV_PORT|=(1<<LBV_PIN);

    _delay_us(10);

    if (LBV_IN&(1<<LBV_PIN))
    {//battery is normal
        status=0;
    }
    else
    {//battery is low
        status=1;
    }

    LBV_PORT&=~(1<<LBV_PIN);
    LBV_DIR|=(1<<LBV_PIN);

    return status;
}

/*---------------------------------------------------------------------------*/
//LC72131

#if LC7_DATA_STATIC_MEM==1
static uint8_t lc72131_data_cache[6]=
{
    1<<LC7_IN1_1_P6,
    0,
    REF_DIV,
    0,
    0,
    0
};
#endif // LC7_DATA_STATIC_MEM

static uint8_t lc72131_bit_swap(uint8_t bitmap)
{
    uint8_t retVal=0;
    uint8_t counter;

    for (counter=0;counter<8;counter++)
    {
        if (bitmap&1<<counter)
        {
            retVal|=1<<(7-counter);
        }
    }

    return retVal;
}

void lc72131_init (void)
{

    uint8_t data[4];

    data[0]=LC7_IN1_MODE;

    uint8_t counter;

    for (counter=0;counter<3;counter++)
    {
        data[counter+1]=lc72131_data_cache[counter];
    }

    exchange_data_spi(data,4,4);


    data[0]=LC7_IN2_MODE;

    for (counter=0;counter<3;counter++)
    {
        data[counter+1]=lc72131_data_cache[counter+3];
    }

    exchange_data_spi(data,4,4);

    return ;
}

void lc72131_set_frequency(uint32_t frequency,uint8_t band)
{

    uint8_t data[4];

    uint32_t fref=LC7_FREF;


    data[3]=REF_DIV;

    switch (band)
    {
    case 0:
        {//FMIN
            data[3]|=(1<<LC7_IN1_3_DVS);
            fref*=2;
            break;
        }
    case 1:
        {//AMIN through swallow counter
            data[3]|=(1<<LC7_IN1_3_SNS);
            break;
        }
    case 2:
        {//AMIN direct
            break;
        }
    default :
        {
            break;
        }
    }

    uint16_t divValue;

    divValue=frequency/fref;

    data[0]=LC7_IN1_MODE;
    #if LC7_BIT_ORDER==0
    data[1]=divValue&0xff;//LSB first
    data[2]=divValue>>8;
    #else
    data[1]=lc72131_bit_swap(divValue&0xff);
    data[2]=lc72131_bit_swap(divValue>>8);
    #endif // LC7_BIT_ORDER


    uint8_t counter;

    for (counter=0;counter<3;counter++)
    {
        lc72131_data_cache[counter]=data[counter+1];
    }

    exchange_data_spi(data,4,4);

    return ;
}

uint32_t lc72131_measure_freq (void)
{
    uint32_t frequency;
    uint8_t data[4];

    //set time base
    //set DO as end of measure
    data[0]=LC7_IN2_MODE;

    uint8_t counter;
    for (counter=0;counter<3;counter++)
    {
        data[counter+1]=lc72131_data_cache[counter+3];
    }

    data[2]|=1<<LC7_IN2_2_DOC1;
    data[3]|=(1<<LC7_IN2_3_GT1);//32 ms time base
    lc72131_data_cache[4]=data[2];
    lc72131_data_cache[5]=data[3];

    exchange_data_spi(data,4,4);


    //reset counter
    data[0]=LC7_IN1_MODE;

    for (counter=0;counter<3;counter++)
    {
        data[counter+1]=lc72131_data_cache[counter];
    }

    data[3]&=~(1<<LC7_IN1_3_CTE);
    lc72131_data_cache[2]=data[3];
    exchange_data_spi(data,4,4);


    //trig counter
    data[0]=LC7_IN1_MODE;

    for (counter=0;counter<3;counter++)
    {
        data[counter+1]=lc72131_data_cache[counter];
    }

    data[3]|=(1<<LC7_IN1_3_CTE);
    exchange_data_spi(data,4,4);


    _delay_us(100);


    //wait
    while (read_gpio (LC7_PORT_NUMB,LC7_DO_PIN))
    {
        ;
    }

    //read
    data[0]=LC7_OUT_MODE;

    exchange_data_spi(data,4,4);

    uint32_t tmpFreq;

    #if LC7_BIT_ORDER==0

    tmpFreq=((uint32_t)lc72131_bit_swap(data[1]&0xf0))<<16;
    tmpFreq+=(uint32_t)lc72131_bit_swap(data[2])<<8;
    tmpFreq+=(uint32_t)lc72131_bit_swap(data[3]);

    #else

    tmpFreq=((uint32_t)data[1]&0x0f)<<16;
    tmpFreq+=(uint32_t)data[2]<<8;
    tmpFreq+=(uint32_t)data[3];

    #endif

    //time base 32 ms
    frequency=tmpFreq*FREQ_COEF;

    return frequency;
}

void lc72131_gpio(uint8_t state)
{
    uint8_t data[4];

    data[0]=LC7_IN2_MODE;

    uint8_t counter;
    for (counter=0;counter<3;counter++)
    {
        data[counter+1]=lc72131_data_cache[counter+3];
    }

    data[1]=(1<<LC7_IN2_1_IOC1)|(1<<LC7_IN2_1_IOC2);



    if (state&(1<<0))
        {
            data[1]|=(1<<LC7_IN2_1_BO1);
        }
    if (state&(1<<1))
        {
            data[1]|=(1<<LC7_IN2_1_BO2);
        }
    if (state&(1<<2))
        {
            data[1]|=(1<<LC7_IN2_1_BO3);
        }
    if (state&(1<<3))
        {
            data[1]|=(1<<LC7_IN2_1_BO4);
        }
    if (state&(1<<4))
        {
            data[1]|=(1<<LC7_IN2_1_IO1);
        }
    if (state&(1<<5))
        {
            data[1]|=(1<<LC7_IN2_1_IO2);
        }


    lc72131_data_cache[3]=data[1];

    exchange_data_spi(data,4,4);

    return ;
}

lc72131_stat_t lc72131_status (void)
{
    lc72131_stat_t retVal={0};

    uint8_t data[4];

    //read
    data[0]=LC7_OUT_MODE;

    exchange_data_spi(data,4,4);


    if (!(data[1]&(1<<LC7_OUT_1_I1)))
    {
        retVal.in_1=1;
    }

    if (!(data[1]&(1<<LC7_OUT_1_I2)))
    {
        retVal.in_2=1;
    }

    if (!(data[1]&(1<<LC7_OUT_1_UL)))
    {
        retVal.unlock=1;
    }

    return retVal;
}

void lc72131_start(void)
{

    uint8_t data[4];
    uint8_t counter;

    //normal operation of charge pump
    //for normal operation
    data[0]=LC7_IN2_MODE;

    for (counter=0;counter<3;counter++)
    {
        data[counter+1]=lc72131_data_cache[counter+3];
    }

    data[3]&=~(1<<LC7_IN2_3_DLC);

    lc72131_data_cache[5]=data[3];

    exchange_data_spi(data,4,4);


    //start oscillators
    data[0]=LC7_IN1_MODE;



    for (counter=0;counter<3;counter++)
    {
        data[counter+1]=lc72131_data_cache[counter];
    }

    data[3]&=~(1<<LC7_IN1_3_R1|1<<LC7_IN1_3_R2);
    data[3]|=(1<<LC7_IN1_3_R3|1<<LC7_IN1_3_R0);

    lc72131_data_cache[2]=data[3];

    exchange_data_spi(data,4,4);

    return;
}

void lc72131_stop(void)
{

    uint8_t data[4];
    uint8_t counter;


    //forced low charge pump
    //for low  power consumption
    data[0]=LC7_IN2_MODE;

    for (counter=0;counter<3;counter++)
    {
        data[counter+1]=lc72131_data_cache[counter+3];
    }

    data[3]|=(1<<LC7_IN2_3_DLC);

    lc72131_data_cache[5]=data[3];

    exchange_data_spi(data,4,4);

    //stop all oscillators
    data[0]=LC7_IN1_MODE;



    for (counter=0;counter<3;counter++)
    {
        data[counter+1]=lc72131_data_cache[counter];
    }

    data[3]&=~(1<<LC7_IN1_3_R0);
    data[3]|=(1<<LC7_IN1_3_R3|1<<LC7_IN1_3_R2|1<<LC7_IN1_3_R1);

    lc72131_data_cache[2]=data[3];

    exchange_data_spi(data,4,4);



    return;
}

/*---------------------------------------------------------------------------*/

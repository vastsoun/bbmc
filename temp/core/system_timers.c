/** Module Header **/
#include "system_timers.h"

/** Std C Headers **/
#include <string.h>

/** BBMC Headers **/
#include "uartStdio.h"
#include "cmdline.h"


/** Internal Data Types
 *  
 */
//TODO: add a timer state data type


/** Internal Data
 *  
 */
//TODO: add an internal static state for each timer


/** System Timer Functions
 * 
 */

int
timer_enable (unsigned int timer)
{
    int ret;
    
    if (timer == TIMER_RUN)
    {
        dev_timer_1_enable();
        ret = 0;
    }
    
    else if (timer == TIMER_GOTO)
    {
        dev_timer_2_enable();
        ret = 1;
    }
    
    else if (timer == TIMER_RMPI)
    {
        dev_timer_3_enable();
        ret = 2;
    }
    
    else if (timer == TIMER_STOP)
    {
        dev_timer_4_enable();
        ret = 3;
    }
    
    else
    {
        ret = -1;
    }
    
    return ret;
}

int
timer_disable (unsigned int timer)
{
    int ret;
    
    if (timer == TIMER_RUN)
    {
        dev_timer_1_disable();
        ret = 0;
    }
    
    else if (timer == TIMER_GOTO)
    {
        dev_timer_2_disable();
        ret = 1;
    }
    
    else if (timer == TIMER_RMPI)
    {
        dev_timer_3_disable();
        ret = 2;
    }
    
    else if (timer == TIMER_STOP)
    {
        dev_timer_4_disable();
        ret = 3;
    }
    
    else
    {
        ret = -1;
    }
    
    return ret;
}

int 
timer_frequency_get (unsigned int timer, unsigned int *frequency)
{
    return dev_timer_frequency_get (timer, frequency);
}

int 
timer_frequency_set (unsigned int timer, double frequency)
{
    unsigned int count;
    
    if (frequency < 0)
    {
        return -1;
    }
    
    frequency = DMTIMER_SYSTEM_CLK_DEFAULT / frequency;
    count = DMTIMER_COUNT_MAX - (unsigned int)frequency;
    
    return dev_timer_frequency_set (timer, count);
}

int 
timer_print (const char *format)
{
    UARTprintf("\r\n%sControl Timer Configurations: \r\n", format);
    
    unsigned int freq;
    
    timer_frequency_get(TIMER_RUN, &freq);
    UARTprintf("\r\n%stimer::run  := %d Hz", format, freq);
    
    timer_frequency_get(TIMER_GOTO, &freq);
    UARTprintf("\r\n%stimer::goto := %d Hz", format, freq);
    
    timer_frequency_get(TIMER_RMPI, &freq);
    UARTprintf("\r\n%stimer::rmpi := %d Hz", format, freq);
    
    timer_frequency_get(TIMER_STOP, &freq);
    UARTprintf("\r\n%stimer::stop := %d Hz", format, freq);
    
    CmdLineNewline(2);
    
    return 0;
}




/**
 * 
 */

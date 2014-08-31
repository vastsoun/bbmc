/** Module Header **/
#include "isr_manager.h"

/** Std C Headers **/
#include <stdlib.h>
#include <errno.h>
#include <ctype.h>


/** Firmware Headers **/
#include "uartStdio.h"

/** BBMC Headers **/
#include "device_layer.h"
#include "util.h"




/** External functions
 * 
 */
//extern void isr_experiment (void);
//extern void isr_goto (void);
//extern void isr_rmpi (void);
//TODO
void isr_experiment (void)
{
    ;
}

void isr_goto (void)
{
    ;
}

void isr_rmpi (void)
{
    ;
}


extern void isr_gpio_killswitch (void);
extern void isr_gpio_poslim (void);
extern void isr_systick (void);



/** Internal Data Types 
 * 
 */

typedef struct
{
    unsigned int iteration_counter;
    unsigned int termination_flag;
    unsigned int termination_counter;
    unsigned int termination_return;
}
isr_state_t;



/** Internal Data
 * 
 */
static isr_state_t volatile   g_isr_state;

static isr_fp_t    const      g_isr_funcs[6] = 
                                                {
                                                    isr_experiment,
                                                    isr_goto,
                                                    isr_rmpi,
                                                    isr_systick,
                                                    isr_gpio_killswitch,
                                                    isr_gpio_poslim,
                                                };



/** ISR state managemenent funtions 
 * 
 */

int 
isr_setup (void)
{
    UARTPuts("\r\nConfiguring Interrupt Controller: ", -1);
    dev_intc_setup((isr_fp_t *)g_isr_funcs);
    UARTPuts("\tDONE", -1);
    
    return 0;
}

int 
isr_state_init (void)
{
    g_isr_state.iteration_counter = 0;
    g_isr_state.termination_flag = 0;
    g_isr_state.termination_counter = 0;
    g_isr_state.termination_return = 0;
    
    return 0;
}

int 
isr_state_set (isr_state set_mode, int set_val)
{
    if (set_mode == ITER_COUNT)
    {
        g_isr_state.iteration_counter = set_val;
    }
    
    else if (set_mode == TERM_COUNT)
    {
        g_isr_state.termination_counter = set_val;
    }
    
    else if (set_mode == TERM_FLAG)
    {
        g_isr_state.termination_flag = set_val;
    }
    
    else if (set_mode == TERM_RET)
    {
        g_isr_state.termination_return = set_val;
    }
    
    else
    {
        UARTPuts("error: contrl_state_set: set_mode argument is invalid\r\n", -1);
        return -1;
    }
    
    return 0;
}

int 
isr_state_get (isr_state get_mode)
{
    unsigned int ret_val = 0;
    
    if (get_mode == ITER_COUNT)
    {
        ret_val = g_isr_state.iteration_counter;
    }
    
    else if (get_mode == TERM_COUNT)
    {
        ret_val = g_isr_state.termination_counter;
    }
    
    else if (get_mode == TERM_FLAG)
    {
        ret_val = g_isr_state.termination_flag;
    }
    
    else if (get_mode == TERM_RET)
    {
        ret_val = g_isr_state.termination_return;
    }
    
    else
    {
        UARTPuts("error: contrl_state_get: get_mode argument is invalid\r\n", -1);
        return -1;
    }
    
    return (int)ret_val;
}

int 
isr_state_print (const char* format)
{
    UARTprintf("\r\n%sISR State is:\r\n", format);
    
    UARTprintf("\r\n%siteration counter is   :  %d", format, g_isr_state.iteration_counter);
    UARTprintf("\r\n%stermination counter is :  %d", format, g_isr_state.termination_counter);
    UARTprintf("\r\n%stermination flag is    :  %d", format, g_isr_state.termination_flag);
    UARTprintf("\r\n%stermination return is  :  %d", format, g_isr_state.termination_return);
    
    return 0;
}


/** ISR management functions
 * 
 */

int
isr_master_enable (void)
{
    return dev_intc_master_enable();
}

int
isr_master_disable (void)
{
    return dev_intc_master_disable();
}






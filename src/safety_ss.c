/** Module Header **/
#include "safety_ss.h"

/** Std C Headers **/
#include <stdlib.h>
#include <errno.h>
#include <ctype.h>
#include <math.h>
#include <float.h>

/** Firmware Headers **/
#include "uartStdio.h"

/** BBMC Headers **/
#include "device_layer.h"
#include "system_timers.h"
#include "isr_manager.h"
#include "safety_ss.h"

#include "global_state.h"
#include "global_flags.h"

#include "util.h"


/** Internal Data Types
 *  
 */

typedef struct
{
    unsigned int ticks;
    unsigned int ticking_flag;
}
systick_t;



/** Internal Data
 *  
 */

static bbmc_actuator_state_t volatile stop_state[BBMC_DOF_NUM];

static bbmc_contrl_motor_t volatile stop_controller[BBMC_DOF_NUM];

static systick_t volatile systick_state;



/** Internal Functions
 * 
 */

static void _gpio_hall_check (void)
{
    /** Check to see if Y axis Hall Sensor triggered the interrupt
     * 
     */
    if ((GPIOPinIntStatus(HALL_Y_GPIO_ADDRESS, HALL_Y_GPIO_INT_LINE, HALL_Y_GPIO_PIN) 
        >> HALL_Y_GPIO_PIN))
    {
        if (((GPIOPinRead(HALL_Y_GPIO_ADDRESS, HALL_Y_GPIO_PIN) >> HALL_Y_GPIO_PIN)) && 
            (global_flags_gpreset_get(1) == MIN))
        {
            global_flags_gpreset_set(1, MAX);
            global_flag_set(FLG_STOP_EMR);
            
            if (global_flag_get(FLG_DEBUG) == 1)
            {
                UARTPuts("\r\nDEBUG_MSG: GPIO INTERRUPT: minpos y\r\n", -1);
            }
        }
        
        if ((!(GPIOPinRead(HALL_Y_GPIO_ADDRESS, HALL_Y_GPIO_PIN) >> HALL_Y_GPIO_PIN)) &&
            (global_flags_gpreset_get(1) == MAX))
        {
            global_flags_gpreset_set(1, MIN);
            global_flag_set(FLG_STOP_EMR);
            
            if (global_flag_get(FLG_DEBUG) == 1)
            {
                UARTPuts("\r\nDEBUG_MSG: GPIO INTERRUPT: maxpos y\r\n", -1);
            }
        }
        
        /* clear status(masked) of interrupts */
        GPIOPinIntClear(HALL_Y_GPIO_ADDRESS, HALL_Y_GPIO_INT_LINE, HALL_Y_GPIO_PIN);
    }
    
    /** Check to see if X axis Hall Sensor triggered the interrupt
     * 
     */
    if ((GPIOPinIntStatus(HALL_X_GPIO_ADDRESS, HALL_X_GPIO_INT_LINE, HALL_X_GPIO_PIN)
            >> HALL_X_GPIO_PIN))
    {
        if (((GPIOPinRead(HALL_X_GPIO_ADDRESS, HALL_X_GPIO_PIN) >> HALL_X_GPIO_PIN)) && 
            (global_flags_gpreset_get(2) == MIN))
        {
            global_flags_gpreset_set(2, MAX);
            global_flag_set(FLG_STOP_EMR);
            
            if (global_flag_get(FLG_DEBUG) == 1)
            {
                UARTPuts("\r\nDEBUG_MSG: GPIO INTERRUPT: minpos x\r\n", -1);
            }
        }
        
        if ((!(GPIOPinRead(HALL_X_GPIO_ADDRESS, HALL_X_GPIO_PIN) >> HALL_X_GPIO_PIN)) && 
            (global_flags_gpreset_get(2) == MAX))
        {
            global_flags_gpreset_set(2, MIN);
            global_flag_set(FLG_STOP_EMR);
            
            if (global_flag_get(FLG_DEBUG) == 1)
            {
                UARTPuts("\r\nDEBUG_MSG: GPIO INTERRUPT: maxpos x\r\n", -1);
            }
        }
        
        /* clear status(masked) of interrupts */
        GPIOPinIntClear(HALL_X_GPIO_ADDRESS, HALL_X_GPIO_INT_LINE, HALL_X_GPIO_PIN);
    }
}





/** Fault/Safety Subsystem Management (global position presets)
 * 
 */

int 
poslim_enable (unsigned int axis)
{
    return dev_poslim_enable(axis);
}

int 
poslim_disable (unsigned int axis)
{
    return dev_poslim_disable(axis);
}

int 
poslim_gpio_get (unsigned int  dev_id, unsigned int *pin_value)
{
    *pin_value = dev_gpio_poslim_get(dev_id);
    
    return 0;
}



int 
killswitch_enable (void)
{
    return dev_killswitch_enable();
}

int 
killswitch_disable (void)
{
    return dev_killswitch_disable();
}

int 
killswitch_gpio_get (unsigned int *pin_value)
{
    *pin_value = dev_gpio_killswitch_get();
    
    return 0;
}




/** Safety ISR Handlers
 * 
 */

void 
isr_gpio_poslim(void)
{
    static int counter = 0;
    
    
    /** Test to see if and which HALL Sensor triggered the poslim event **/
    _gpio_hall_check();
    
    
    if (global_flag_get(FLG_STOP_EMR) == 1)
    {
        global_flag_isr_set(RET_POSLIM);
        
        counter= 0;
        systick_state.ticks = 0;
        
        global_state_read (&stop_state[0]);
        global_state_read (&stop_state[1]);
        
        DMTimerEnable(TIMER_STOP);
        
        for (;;)
        {
            
            input_encoder_2D (stop_state);
            
            stop_controller[0].control.output = 0;
            stop_controller[1].control.output = 0;
            
            if ((fabs(stop_state[0].input.speed) <= STOP_SPEED_X) && 
                (fabs(stop_state[1].input.speed) <= STOP_SPEED_X))
            {
                counter = systick_state.ticks;
                
                if (counter >= MAX_STOP_COUNT)
                {
                    stop_controller[0].control.output = 0;
                    stop_controller[1].control.output = 0;
                    
                    isr_state_set (TERM_FLAG, 1);
                    break;
                }
            }
            
            output_motor_2D (stop_controller);
            
            /* start tick timer */
            DMTimerIntEnable(TIMER_STOP, DMTIMER_INT_OVF_EN_FLAG);
            
            while(systick_state.ticking_flag == 0)
            {
                ;
            }
            
            /* stop tick timer */
            DMTimerIntDisable(TIMER_STOP, DMTIMER_INT_OVF_EN_FLAG);
            DMTimerIntStatusClear(TIMER_STOP, DMTIMER_INT_OVF_IT_FLAG);
            
        }
        
        /* stop tick timer */
        DMTimerIntDisable(TIMER_STOP, DMTIMER_INT_OVF_EN_FLAG);
        DMTimerIntStatusClear(TIMER_STOP, DMTIMER_INT_OVF_IT_FLAG);
        
        stop_controller[0].control.output = 0;
        stop_controller[1].control.output = 0;
        
        output_motor_2D (stop_controller);
        
        pwm_disable(1);
        pwm_disable(2);
        
        /* stop tick timer */
        DMTimerDisable(TIMER_STOP);
    
        UARTPuts("\r\n\r\n\tGPIO pos-lim system has enacted Emergency Stop\r\n", -1);
    }
}

void 
isr_gpio_killswitch(void)
{
    static int counter = 0;
    
    if ((GPIOPinIntStatus(KILLSWITCH_GPIO_ADDRESS, KILLSWITCH_GPIO_INT_LINE, 
            KILLSWITCH_GPIO_PIN) >> KILLSWITCH_GPIO_PIN))
    {
        global_flag_set(FLG_STOP_EMR);
        global_flag_isr_set(RET_KILLSW);
        
        counter= 0;
        systick_state.ticks = 0;
        
        global_state_read (&stop_state[0]);
        global_state_read (&stop_state[1]);
        
        DMTimerEnable(TIMER_STOP);
        
        for (;;)
        {
            input_encoder_2D (stop_state);
            
            stop_controller[0].control.output = 0;
            stop_controller[1].control.output = 0;
            
            if ((fabs(stop_state[0].input.speed) <= STOP_SPEED_X) && 
                (fabs(stop_state[1].input.speed) <= STOP_SPEED_X))
            {
                counter = systick_state.ticks;
                
                if (counter >= MAX_STOP_COUNT)
                {
                    stop_controller[0].control.output = 0;
                    stop_controller[1].control.output = 0;
                    
                    isr_state_set (TERM_FLAG, 1);
                    break;
                }
            }
            
            output_motor_2D (stop_controller);
            
            /* start tick timer */
            DMTimerIntEnable(TIMER_STOP, DMTIMER_INT_OVF_EN_FLAG);
            
            while(systick_state.ticking_flag == 0)
            {
                ;
            }
            
            /* stop tick timer */
            DMTimerIntDisable(TIMER_STOP, DMTIMER_INT_OVF_EN_FLAG);
            DMTimerIntStatusClear(TIMER_STOP, DMTIMER_INT_OVF_IT_FLAG);
        }
        
        /* stop tick timer */
        DMTimerIntDisable(TIMER_STOP, DMTIMER_INT_OVF_EN_FLAG);
        DMTimerIntStatusClear(TIMER_STOP, DMTIMER_INT_OVF_IT_FLAG);
        
        stop_controller[0].control.output = 0;
        stop_controller[1].control.output = 0;
        
        output_motor_2D (stop_controller);
        
        pwm_disable(1);
        pwm_disable(2);
        
        /* stop tick timer */
        DMTimerDisable(TIMER_STOP);
        
        UARTPuts("\r\nWARNING: Killswitch has enacted Emergency Stop\r\n", -1);
        
        if (global_flag_get(FLG_DEBUG) == 1)
        {
            UARTPuts("DEBUG_MSG: Killswitch Engage!!\r\n", -1);
        }
    }
    
    /* clear status (masked) of interrupts */
    GPIOPinIntClear(KILLSWITCH_GPIO_ADDRESS, KILLSWITCH_GPIO_INT_LINE, KILLSWITCH_GPIO_PIN);
}

void 
isr_systick (void)
{
    //! this can remain like this for now, but must be updated
    //! new features will be added to systick.
    
    DMTimerIntDisable(TIMER_STOP, DMTIMER_INT_OVF_EN_FLAG);
    DMTimerIntStatusClear(TIMER_STOP, DMTIMER_INT_OVF_IT_FLAG);
    
    systick_state.ticks++;
    systick_state.ticking_flag = 1;
    
    DMTimerIntEnable(TIMER_STOP, DMTIMER_INT_OVF_EN_FLAG);
}




/** Safety Stop Subystem Managemt
 * 
 */

int 
safety_stop_setup (void)
{
    /* argument map: STOP_ARGS::contrl_stop_immediate
     * 
     *  i0: -                         , d0: P gain
     *  i1: -                         , d1: I gain
     *  i2: -                         , d2: Integral Sum
     *  i3: -                         , d3: -
     *  i4: -                         , d4: -
     *  i5: -                         , d5: -
     *  i6: -                         , d6: -
     *  i7: -                         , d7: -
     */
    
    stop_controller[0].arg_double[0] = STOP_SPEED_GAIN_P_1;
    stop_controller[1].arg_double[0] = STOP_SPEED_GAIN_P_2;
    
    stop_controller[0].arg_double[1] = STOP_SPEED_GAIN_I_1;
    stop_controller[1].arg_double[1] = STOP_SPEED_GAIN_I_2;
    
    stop_controller[0].arg_double[2] = 0;
    stop_controller[1].arg_double[2] = 0;
    
    return 0;
}

int 
safety_stop_reset (void)
{
    /* reset integral sum */
    stop_controller[0].arg_double[2] = 0;
    stop_controller[1].arg_double[2] = 0;
    
    return 0;
}

void
contrl_stop_immediate (bbmc_actuator_state_t volatile *state, 
                       bbmc_contrl_motor_t   volatile *controller)
{
    #ifdef DRIVER_MODE_VOLTAGE
    
    controller->control.output = 0;
    
    #endif
    
    #ifdef DRIVER_MODE_CURRENT
    
    int dof_id = controller->control.dev_id;
    
    controller->arg_double[2] += (- state->input.speed);
    
    controller->control.output = stop_controller[dof_id].arg_double[0] * (- state->input.speed);
    
    controller->control.output += stop_controller[dof_id].arg_double[1] * stop_controller[dof_id].arg_double[2];
    
    #endif
}


/** EOF
 *  
 */

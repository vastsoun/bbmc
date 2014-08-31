/** Device Layer Library Header **/
#include "device_layer.h"


/** Internal Data
 * 
 * 
 */
static dmtimer_handle_t   bbmc_timers[4];
static ehrpwm_handle_t    bbmc_pwm[2];
static eqep_handle_t      bbmc_qei[2];






/** Core setup configuration functions 
 *  
 */

int 
dev_setup(void)
{
    
    dev_stdio_setup();
    UARTprintf("\033[2J\033[1;1HBBMC-v0.5 is now initializing...\r\n");
    UARTPuts("\r\nUART_0 stdio channel has opened.", -1);
    
    UARTPuts("\r\nInitializing Peripheral Drivers: ", -1);
    dev_peripheral_setup();
    UARTPuts("\tDONE", -1);
    
    UARTPuts("\r\nConfiguring MPU cache: ", -1);
    dev_mpucache_setup(CACHE_ALL);
    UARTPuts("\tDONE", -1);
    
    UARTPuts("\r\nConfiguring System Timers: ", -1);
    dev_timer_setup();
    UARTPuts("\t\tDONE", -1);
    
    UARTPuts("\r\nConfiguring Performance Timer: ", -1);
    PerfTimerSetup();
    UARTPuts("\t\tDONE", -1);
    
    UARTPuts("\r\nConfiguring GPIO: ", -1);
    dev_gpio_setup();
    UARTPuts("\t\t\tDONE", -1);
    
    UARTPuts("\r\nConfiguring PWM and QEI: ", -1);
    dev_pwm_setup();
    dev_qei_setup();
    UARTPuts("\t\tDONE", -1);
    
    return 0;
}

int 
dev_peripheral_setup (void)
{
    L3L4_driver_init();
    pwmss_driver_init();
    
    return 0;
}

int 
dev_stdio_setup(void)
{
    UARTStdioInit();
    
    return 0;
}

int 
dev_mpucache_setup (unsigned int cache_mode)
{
    if (cache_mode == CACHE_ALL)
    {
        CacheEnable(CACHE_ALL);
        UARTPuts("\r\ndevconfig: mpu cache: all enabled", -1);
    }
    
    else if (cache_mode == CACHE_ICACHE)
    {
        CacheEnable(CACHE_ICACHE);
        UARTPuts("\r\ndevconfig: mpu cache: data cache enabled", -1);
    }
    
    else if (cache_mode == CACHE_DCACHE)
    {
        CacheEnable(CACHE_DCACHE);
        UARTPuts("\r\ndevconfig: mpu cache: instruction cache enabled", -1);
    }
    
    else
    {
        UARTPuts("\r\nerror: devconfig: mpu cache: invalid cache_mode argumemnt", -1);
    }
    
    return 0;
}



/** PWM Output Subsystem 
 *  
 */

int 
dev_pwm_setup (void)
{
    ehrpwm_handle_init(1, &bbmc_pwm[0]);
    ehrpwm_handle_init(2, &bbmc_pwm[1]);
    
    ehrpwm_open(1);
    ehrpwm_open(2);
    
    ehrpwm_init(1);
    ehrpwm_init(2);
    
    ehrpwm_1_pinmux_setup();
    ehrpwm_2_pinmux_setup();
    
    return 0;
}

int 
dev_pwm_frequency_set (unsigned int dev_id, 
                       double frequency, 
                       unsigned int resolution)
{
    return ehrpwm_config_frequency_set(dev_id, frequency, resolution);
}

int
dev_pwm_frequency_get (unsigned int dev_id, 
                       double *ret_frequency)
{
    return ehrpwm_config_frequency_get(dev_id, ret_frequency);
}

int 
dev_pwm_enable (unsigned int dev_id)
{
    return ehrpwm_enable(dev_id);
}

int 
dev_pwm_disable (unsigned int dev_id)
{
    return ehrpwm_disable(dev_id);
}



/** Encoder Input Subsystem 
 *  
 */

//TODO
int 
dev_qei_setup (void)
{
    eqep_handle_init(1, &bbmc_qei[0]);
    eqep_handle_init(2, &bbmc_qei[1]);
    
    eqep_open(1);
    eqep_open(2);
    
    eqep_init(1);
    eqep_init(2);
    
    eqep_1_pinmux_setup();
    eqep_2_pinmux_setup();
    
    return 0;
}

int 
dev_qei_data_init (dev_input_qei_t volatile *state)
{
    return eqep_data_init (&(state->input));
}

int dev_qei_data_cpy  (dev_input_qei_t volatile *src,
                       dev_input_qei_t volatile *dest)
{
    return eqep_data_copy (&(src->input), &(src->input));
}

int 
dev_qei_cap_config (unsigned int dev_id,
                    unsigned int unit_position,
                    unsigned int clk_prescaler)
{
    return eqep_caputure_config (dev_id, unit_position, clk_prescaler);
}

int dev_qei_count_set  (dev_input_qei_t volatile *state, unsigned int count)
{
    return eqep_write (state->dev_id, count);
}

int 
dev_qei_frequency_set (dev_input_qei_t volatile *data, unsigned int frequency)
{
    data->input.sampling_freq = frequency;
    
    return 0;
}


/** System and Controller Timers 
 *  
 */

int 
dev_timer_setup (void)
{
    dmtimer_handle_init(2, &bbmc_timers[0]);
    dmtimer_handle_init(3, &bbmc_timers[1]);
    dmtimer_handle_init(4, &bbmc_timers[2]);
    dmtimer_handle_init(5, &bbmc_timers[3]);
    
    dmtimer_open(2);
    dmtimer_open(3);
    dmtimer_open(4);
    dmtimer_open(5);
    
    dmtimer_init(2);
    dmtimer_init(3);
    dmtimer_init(4);
    dmtimer_init(5);
    
    return 0;
}

int 
dev_timer_frequency_get (unsigned int timer, unsigned int *frequency)
{
    if ((timer != TIMER_1) || 
        (timer != TIMER_2) ||
        (timer != TIMER_3) ||
        (timer != TIMER_4) )
    {
        return -1;
    }
    
    unsigned int temp;
    int freq;
    
    temp = dmtimer_tldr_config_get(timer);
    temp = DMTIMER_COUNT_MAX - temp;
    
    freq = DMTIMER_SYSTEM_CLK_DEFAULT / temp;
    
    *frequency = freq;
    
    return 0;
}

int 
dev_timer_frequency_set (unsigned int timer, unsigned int count)
{
    if ((timer != TIMER_1) || 
        (timer != TIMER_2) ||
        (timer != TIMER_3) ||
        (timer != TIMER_4) )
    {
        return -1;
    }
    
    return dmtimer_tldr_config_set(timer, count);
}

int
dev_timer_1_enable (void)
{
    DMTimerIntEnable(TIMER_1, DMTIMER_INT_OVF_EN_FLAG);
    DMTimerEnable(TIMER_1);
    
    IntMasterIRQEnable();
    
    return 0;
}

int
dev_timer_1_disable (void)
{
    IntMasterIRQDisable();
    
    DMTimerIntDisable(TIMER_1, DMTIMER_INT_OVF_EN_FLAG);
    DMTimerIntStatusClear(TIMER_1, DMTIMER_INT_OVF_IT_FLAG);
    DMTimerDisable(TIMER_1);
    
    return 0;
}

int
dev_timer_2_enable (void)
{
    DMTimerIntEnable(TIMER_2, DMTIMER_INT_OVF_EN_FLAG);
    DMTimerEnable(TIMER_2);
    
    IntMasterIRQEnable();
    
    return 0;
}

int
dev_timer_2_disable (void)
{
    IntMasterIRQDisable();
    
    DMTimerIntDisable(TIMER_2, DMTIMER_INT_OVF_EN_FLAG);
    DMTimerIntStatusClear(TIMER_2, DMTIMER_INT_OVF_IT_FLAG);
    DMTimerDisable(TIMER_2);
    
    return 0;
}

int
dev_timer_3_enable (void)
{
    DMTimerIntEnable(TIMER_3, DMTIMER_INT_OVF_EN_FLAG);
    DMTimerEnable(TIMER_3);
    
    IntMasterIRQEnable();
    
    return 0;
}

int
dev_timer_3_disable (void)
{
    IntMasterIRQDisable();
    
    DMTimerIntDisable(TIMER_3, DMTIMER_INT_OVF_EN_FLAG);
    DMTimerIntStatusClear(TIMER_3, DMTIMER_INT_OVF_IT_FLAG);
    DMTimerDisable(TIMER_3);
    
    return 0;
}

int
dev_timer_4_enable (void)
{
    DMTimerIntEnable(TIMER_4, DMTIMER_INT_OVF_EN_FLAG);
    DMTimerEnable(TIMER_4);
    
    IntMasterIRQEnable();
    
    return 0;
}

int
dev_timer_4_disable (void)
{
    IntMasterIRQDisable();
    
    DMTimerIntDisable(TIMER_4, DMTIMER_INT_OVF_EN_FLAG);
    DMTimerIntStatusClear(TIMER_4, DMTIMER_INT_OVF_IT_FLAG);
    DMTimerDisable(TIMER_4);
    
    return 0;
}


/** Safety Subsystem 
 *  
 */

int 
dev_gpio_setup (void)
{
    //! this is temporary until the gpio driver is updated to new prog. model.
    GPIO1ModuleClkConfig();
    GpioModuleEnable(SOC_GPIO_1_REGS);
    
    gpio_dir_1_setup();
    gpio_dir_2_setup();
    
    gpio_hall_1_setup();
    gpio_hall_2_setup();
    
    gpio_killswitch_setup (KILLSWITCH_GPIO_DEBOUCE_TIME);
    
    return 0;
}

int 
dev_poslim_enable (unsigned int axis)
{
    
    if (axis == 0)
    {
        GPIOPinIntClear(HALL_Y_GPIO_ADDRESS, HALL_Y_GPIO_INT_LINE, HALL_Y_GPIO_PIN);
        GPIOPinIntEnable(HALL_Y_GPIO_ADDRESS, HALL_Y_GPIO_INT_LINE, HALL_Y_GPIO_PIN);
    }
    
    else if (axis == 1)
    {
        GPIOPinIntClear(HALL_X_GPIO_ADDRESS, HALL_X_GPIO_INT_LINE, HALL_X_GPIO_PIN);
        GPIOPinIntEnable(HALL_X_GPIO_ADDRESS, HALL_X_GPIO_INT_LINE, HALL_X_GPIO_PIN);
    }
    
    else if (axis == BBMC_DOF_NUM)
    {
        GPIOPinIntClear(HALL_Y_GPIO_ADDRESS, HALL_Y_GPIO_INT_LINE, HALL_Y_GPIO_PIN);
        GPIOPinIntClear(HALL_X_GPIO_ADDRESS, HALL_X_GPIO_INT_LINE, HALL_X_GPIO_PIN);
        
        GPIOPinIntEnable(HALL_Y_GPIO_ADDRESS, HALL_Y_GPIO_INT_LINE, HALL_Y_GPIO_PIN);
        GPIOPinIntEnable(HALL_X_GPIO_ADDRESS, HALL_X_GPIO_INT_LINE, HALL_X_GPIO_PIN);
    }
    
    else
    {
        return -1;
    }
    
    return 0;
}

int 
dev_poslim_disable (unsigned int axis)
{
    if (axis == 0)
    {
        GPIOPinIntDisable(HALL_Y_GPIO_ADDRESS, HALL_Y_GPIO_INT_LINE, HALL_Y_GPIO_PIN);
        GPIOPinIntClear(HALL_Y_GPIO_ADDRESS, HALL_Y_GPIO_INT_LINE, HALL_Y_GPIO_PIN);
    }
    
    else if (axis == 1)
    {
        GPIOPinIntClear(HALL_X_GPIO_ADDRESS, HALL_X_GPIO_INT_LINE, HALL_X_GPIO_PIN);
        GPIOPinIntDisable(HALL_X_GPIO_ADDRESS, HALL_X_GPIO_INT_LINE, HALL_X_GPIO_PIN);
    }
    
    else if (axis == 2)
    {
        GPIOPinIntDisable(HALL_Y_GPIO_ADDRESS, HALL_Y_GPIO_INT_LINE, HALL_Y_GPIO_PIN);
        GPIOPinIntDisable(HALL_X_GPIO_ADDRESS, HALL_X_GPIO_INT_LINE, HALL_X_GPIO_PIN);
        
        GPIOPinIntClear(HALL_Y_GPIO_ADDRESS, HALL_Y_GPIO_INT_LINE, HALL_Y_GPIO_PIN);
        GPIOPinIntClear(HALL_X_GPIO_ADDRESS, HALL_X_GPIO_INT_LINE, HALL_X_GPIO_PIN);
    }
    
    else
    {
        return -1;
    }
    
    return 0;
}

int
dev_killswitch_enable (void)
{
    GPIOPinIntClear(KILLSWITCH_GPIO_ADDRESS, KILLSWITCH_GPIO_INT_LINE, KILLSWITCH_GPIO_PIN);
    GPIOPinIntEnable(KILLSWITCH_GPIO_ADDRESS, KILLSWITCH_GPIO_INT_LINE, KILLSWITCH_GPIO_PIN);
    
    return 0;
}

int
dev_killswitch_disable (void)
{
    GPIOPinIntDisable(KILLSWITCH_GPIO_ADDRESS, KILLSWITCH_GPIO_INT_LINE, KILLSWITCH_GPIO_PIN);
    GPIOPinIntClear(KILLSWITCH_GPIO_ADDRESS, KILLSWITCH_GPIO_INT_LINE, KILLSWITCH_GPIO_PIN);
    
    return 0;
}

unsigned int 
dev_gpio_poslim_get (unsigned int  dev_id)
{
    unsigned int ret;
    
    if (dev_id == 1)
    {
        ret = GPIOPinRead(HALL_Y_GPIO_ADDRESS, HALL_Y_GPIO_PIN) >> HALL_Y_GPIO_PIN;
    }
    
    else if (dev_id == 2)
    {
        ret = GPIOPinRead(HALL_X_GPIO_ADDRESS, HALL_X_GPIO_PIN) >> HALL_X_GPIO_PIN;
    }
    
    else
    {
        ret = 0;
    }
    
    return ret;
}

unsigned int
dev_gpio_killswitch_get (void)
{
    return (GPIOPinRead(KILLSWITCH_GPIO_ADDRESS, KILLSWITCH_GPIO_PIN) >> KILLSWITCH_GPIO_PIN);
}



/** Interrupt Controller configuratinos 
 *  
 */

int 
dev_intc_setup (isr_fp_t *isr_funcs)
{
    IntAINTCInit();
    
    /* Register the func pntr for every ISR */
    IntRegister(SYS_INT_TINT2, isr_funcs[0]);
    IntRegister(SYS_INT_TINT3, isr_funcs[1]);
    IntRegister(SYS_INT_TINT4, isr_funcs[2]);
    IntRegister(SYS_INT_TINT5, isr_funcs[3]);//isr_systick
    IntRegister(SYS_INT_GPIOINT1A , isr_funcs[4]);//isr_gpio_poslim
    IntRegister(SYS_INT_GPIOINT1B , isr_funcs[5]);//isr_gpio_killswitch
    
    /* Set priority for each ISR */
    IntPrioritySet(SYS_INT_TINT2, 3, AINTC_HOSTINT_ROUTE_IRQ);
    IntPrioritySet(SYS_INT_TINT3, 3, AINTC_HOSTINT_ROUTE_IRQ);
    IntPrioritySet(SYS_INT_TINT4, 3, AINTC_HOSTINT_ROUTE_IRQ);
    IntPrioritySet(SYS_INT_TINT5, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntPrioritySet(SYS_INT_GPIOINT1A, 1, AINTC_HOSTINT_ROUTE_IRQ);
    IntPrioritySet(SYS_INT_GPIOINT1B, 2, AINTC_HOSTINT_ROUTE_IRQ);
    
    /* Enable Interrupt recognition in AINTC */
    IntSystemEnable(SYS_INT_TINT2);
    IntSystemEnable(SYS_INT_TINT3);
    IntSystemEnable(SYS_INT_TINT4);
    IntSystemEnable(SYS_INT_TINT5);
    IntSystemEnable(SYS_INT_GPIOINT1A);
    IntSystemEnable(SYS_INT_GPIOINT1B);
    
    return 0;
}

int 
dev_intc_master_disable (void)
{
    IntMasterIRQDisable();
    
    return 0;
}

int 
dev_intc_master_enable (void)
{
    IntMasterIRQEnable();
    
    return 0;
}







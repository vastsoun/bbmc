#ifndef _BBMC_DEVICE_LAYER_H_
#define _BBMC_DEVICE_LAYER_H_


#ifdef __cplusplus
    extern "C" {
#endif

/** Primary Hardware headers **/
#include "soc_AM335x.h"
#include "beaglebone.h"
#include "pin_mux.h"
#include "interrupt.h"
#include "cache.h"

/** Device headers **/
#include "hw_ehrpwm.h"
#include "hw_eqep.h"

/** Drivers **/
#include "uartStdio.h"
#include "dmtimer.h"
#include "pwmss.h"
#include "ehrpwm.h"
#include "eqep.h"
#include "gpio_v2.h"

/** Utilities **/
#include "perf.h"


/** BBMC HARDWARE MACROS
 * 
 */

#define BBMC_DOF_NUM                    2
#define GLOBAL_DATA_MAX                 (131072)
#define CONTROLLER_ITERATIONS_MAX       (GLOBAL_DATA_MAX)


#define TIMER_1_ID                      2
#define TIMER_2_ID                      3
#define TIMER_3_ID                      4
#define TIMER_4_ID                      5

#define TIMER_1                         (SOC_DMTIMER_2_REGS)
#define TIMER_2                         (SOC_DMTIMER_3_REGS)
#define TIMER_3                         (SOC_DMTIMER_4_REGS)
#define TIMER_4                         (SOC_DMTIMER_5_REGS)

#define DIR_1_GPIO_ADDRESS              (SOC_GPIO_1_REGS)
#define DIR_1_GPIO_PIN_ADDRESS          (GPIO_1_7)     
#define DIR_1_GPIO_PIN_MODE             (CONTROL_CONF_MUXMODE(7))
#define DIR_1_GPIO_PIN                  (7)
#define DIR_1_GPIO_PIN_DIR              (GPIO_DIR_OUTPUT)

#define DIR_2_GPIO_ADDRESS              (SOC_GPIO_1_REGS)
#define DIR_2_GPIO_PIN_ADDRESS          (GPIO_1_3)     
#define DIR_2_GPIO_PIN_MODE             (CONTROL_CONF_MUXMODE(7))
#define DIR_2_GPIO_PIN                  (3)
#define DIR_2_GPIO_PIN_DIR              (GPIO_DIR_OUTPUT)

#define HALL_Y_GPIO_ADDRESS             (SOC_GPIO_1_REGS)
#define HALL_Y_GPIO_PIN                 (6)
#define HALL_Y_GPIO_INT_LINE            (GPIO_INT_LINE_1)

#define HALL_X_GPIO_ADDRESS             (SOC_GPIO_1_REGS)
#define HALL_X_GPIO_PIN                 (2)
#define HALL_X_GPIO_INT_LINE            (GPIO_INT_LINE_1)

#define KILLSWITCH_GPIO_ADDRESS         (SOC_GPIO_1_REGS)
#define KILLSWITCH_GPIO_PIN             (14)
#define KILLSWITCH_GPIO_INT_LINE        (GPIO_INT_LINE_2)
#define KILLSWITCH_GPIO_DEBOUCE_TIME    (1000)


/** Input/Output configurations.
 * 
 */
#define OUTPUT_PWM_DIFF
//#define OUTPUT_PWM_DIR
//#define OUTPUT_SPI_DAC

#define INPUT_QEP_DUAL
//#define INPUT_QEP_STD
//#define INPUT_QEP_CAP
//#define EQEP_ISR



/** Wrap the specific peripheral drivers with BBMC i/o type wrapper.
 *  
 */
typedef dmtimer_handle_t       dev_timer_t;
typedef eqep_handle_t          dev_encoder_t;
typedef ehrpwm_handle_t        dev_pwm_t;
typedef gpio_device_pin_t      dev_gpio_pin_t;


/** Define the i/o data types.
 *  
 */
typedef struct
{
    double         output;    
    
    unsigned int   dev_id;
    
}
dev_output_pwm_t;


typedef struct
{
    eqep_data_t    input;
    
    unsigned int   dev_id;
}
dev_input_qei_t;


/** Type definition for ISR functions pointers **/
typedef void (*isr_fp_t) (void);


/** Core setup configuration functions 
 *  
 */
int dev_setup(void);

int dev_peripheral_setup (void);

int dev_stdio_setup(void);

int dev_mpucache_setup (unsigned int cache_mode);


/** PWM Output Subsystem 
 *  
 */
int dev_pwm_setup (void);

int dev_pwm_frequency_set (unsigned int dev_id, 
                           double frequency, 
                           unsigned int resolution);

int dev_pwm_frequency_get (unsigned int dev_id, double *ret_frequency);

int dev_pwm_enable  (unsigned int dev_id);

int dev_pwm_disable (unsigned int dev_id);


/** Encoder Input Subsystem 
 *  
 */
int dev_qei_setup (void);

int dev_qei_data_init  (dev_input_qei_t volatile *state);

int dev_qei_data_cpy   (dev_input_qei_t volatile *src,
                        dev_input_qei_t volatile *dest);

int dev_qei_cap_config (unsigned int dev_id,
                        unsigned int unit_position,
                        unsigned int clk_prescaler);

int dev_qei_count_set  (dev_input_qei_t volatile *state, unsigned int count);

int dev_qei_frequency_set (dev_input_qei_t volatile *data, 
                           unsigned int frequency);


/** System and Controller Timers 
 *  
 */
int dev_timer_setup (void);

int dev_timer_frequency_get (unsigned int timer_id, unsigned int *frequency);

int dev_timer_frequency_set (unsigned int timer_id, unsigned int count);

int dev_timer_1_enable  (void);

int dev_timer_1_disable (void);

int dev_timer_2_enable  (void);

int dev_timer_2_disable (void);

int dev_timer_3_enable  (void);

int dev_timer_3_disable (void);

int dev_timer_4_enable  (void);

int dev_timer_4_disable (void);


/** Safety Subsystem 
 *  
 */
int dev_gpio_setup (void);

int dev_poslim_disable (unsigned int axis);

int dev_poslim_enable (unsigned int axis);

int dev_killswitch_enable (void);

int dev_killswitch_disable (void);

unsigned int dev_gpio_poslim_get (unsigned int  dev_id);

unsigned int dev_gpio_killswitch_get (void);


/** Interrupt Controller configuratinos 
 *  
 */
int dev_intc_setup (isr_fp_t *isr_funcs);

int dev_intc_master_disable (void);

int dev_intc_master_enable (void);


/** 
 *  
 *  
 */




#ifdef __cplusplus
}
#endif


#endif /* _BBMC_DEVICE_LAYER_H_ */

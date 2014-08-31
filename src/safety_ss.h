#ifndef _SAFETY_SUBSYSTEM_H_
#define _SAFETY_SUBSYSTEM_H_


#ifdef __cplusplus
    extern "C" {
#endif



/** Module Dependencies **/
#include "motion_control.h"



/** 
 *  STOP system macros
 */

#define MAX_STOP_COUNT                  (1000)
#define MAX_STOP_DELAY_COUNT            (0xFFFu)

#define STOP_SPEED_GAIN_P_1               (0.01)
#define STOP_SPEED_GAIN_P_2               (0.01)
#define STOP_SPEED_GAIN_I_1               (0.01)
#define STOP_SPEED_GAIN_I_2               (0.01)

#define STOP_SPEED_Y                      (500)
#define STOP_SPEED_X                      (500)


/** Fault/Safety Subsystem Management
 * 
 */

int poslim_enable  (unsigned int axis);

int poslim_disable (unsigned int axis);

int poslim_gpio_get (unsigned int  dev_id, unsigned int *pin_value);


int killswitch_enable  (void);

int killswitch_disable (void);

int killswitch_gpio_get (unsigned int *pin_value);



/** Safety Stop Subystem Managemt
 * 
 */

int safety_stop_setup (void);

int safety_stop_reset (void);

void contrl_stop_immediate (bbmc_input_encoder_t volatile *state, 
                            bbmc_contrl_motor_t  volatile *controller);



/** Safety ISR Handlers
 * 
 */

void isr_gpio_poslim (void);

void isr_gpio_killswitch (void);

void isr_systick (void);




#ifdef __cplusplus
}
#endif


#endif /* _SAFETY_SUBSYSTEM_H_ */

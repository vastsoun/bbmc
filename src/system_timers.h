#ifndef _SYSTEM_TIMERS_H_
#define _SYSTEM_TIMERS_H_


#ifdef __cplusplus
    extern "C" {
#endif



/** Library Dependencies **/
#include "device_layer.h"



/** Definition of the System Timers
 *  
 */

#define TIMER_EXP                       (TIMER_1)
#define TIMER_GOTO                      (TIMER_2)
#define TIMER_RMPI                      (TIMER_3)
#define TIMER_STOP                      (TIMER_4)

#define DMTIMER_EXP                     (DMTIMER_1)
#define DMTIMER_GOTO                    (DMTIMER_2)
#define DMTIMER_RMPI                    (DMTIMER_3)
#define DMTIMER_STOP                    (DMTIMER_4)



/** System Timer Functions
 * 
 */

int timer_enable  (unsigned int timer);

int timer_disable (unsigned int timer);

int timer_frequency_get (unsigned int timer, unsigned int *frequency);

int timer_frequency_set (unsigned int timer, double frequency);

int timer_print (const char *format);




#ifdef __cplusplus
}
#endif


#endif /* _SYSTEM_TIMERS_H_ */

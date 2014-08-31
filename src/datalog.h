#ifndef _BBMC_DATALOG_H_
#define _BBMC_DATALOG_H_


#ifdef __cplusplus
    extern "C" {
#endif


/** BBMC Headers **/
#include "motor_control.h"



/** DataLog Macros
 * 
 */
#define DATALOG_STATIC_DATALEN          (GLOBAL_DATA_MAX)
#define DATALOG_STATIC_DATASIZE         (6)



/** Definition of Datalog Data
 * 
 */
typedef double data_t;




/** Datalog API Functions
 * 
 */

int datalog_s_setup (void);

int datalog_s_init  (data_t init_val);

int datalog_s_single_write (unsigned int dev_id,
                            unsigned int log_index,
                            unsigned int datum_index,
                            data_t value);

void datalog_s_write (unsigned int dev_id,
                      unsigned int index,
                      bbmc_input_encoder_t volatile *state,
                      bbmc_contrl_motor_t  volatile *contrl);

int datalog_s_print (unsigned int dev_id, int range_indeces[4]);



/** Performance Log API Functions
 * 
 */

int performance_log_init (void);

int performance_log_write (unsigned int index, unsigned int value);

int performance_log_print (unsigned int iterations);





#ifdef __cplusplus
}
#endif


#endif /* _BBMC_DATALOG_H_ */

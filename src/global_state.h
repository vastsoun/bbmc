#ifndef _BBMC_GLOBAL_STATE_H_
#define _BBMC_GLOBAL_STATE_H_

#ifdef __cplusplus
extern "C" {
#endif



/** BBMC Headers **/
#include "device_layer.h"
#include "motor_control.h"
#include "global_flags.h"


/** Internal Data Types to hide the implementation.
 *  
 */

typedef  bbmc_input_encoder_t   bbmc_actuator_state_t;

typedef  bbmc_motor_range_t     bbmc_actuator_range_t;




/** State Information API
 * 
 */

int global_state_setup (void);

int global_state_init  (void);

int global_state_read  (bbmc_actuator_state_t volatile *local_state);

int global_state_write (bbmc_actuator_state_t volatile *local_state);

int global_position_init (unsigned int dev_id, pos_reset value);

int global_position_get (unsigned int dev_id);

int global_position_set (unsigned int dev_id, unsigned int position);

int global_sampling_frequency_set (unsigned int dev_id, unsigned int frequency);

int global_state_print (unsigned int dev_id, const char *format);


/** Global Position Limit and Pre-set Configurations
 * 
 */

int global_presets_setup (void);


int global_limits_setup (void);

int global_limits_get (unsigned int dev_id, bbmc_motor_range_t *local_limits);

int global_limits_set (unsigned int dev_id, bbmc_motor_range_t *local_limits);


int global_inits_setup (void);

int global_inits_get (unsigned int dev_id, bbmc_motor_range_t *local_limits);

int global_inits_set (unsigned int dev_id, bbmc_motor_range_t *local_limits);


int global_home_setup (void);

int global_home_get (unsigned int dev_id, bbmc_motor_range_t *local_home);

int global_home_set (unsigned int dev_id, bbmc_motor_range_t *local_home);


int global_positions_print (const char *format);




#ifdef __cplusplus
}
#endif
#endif  /* _BBMC_GLOBAL_STATE_H_ */

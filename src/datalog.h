#ifndef _BBMC_DATALOG__H_
#define _BBMC_DATALOG_H_


#ifdef __cplusplus
    extern "C" {
#endif


/** BBMC Headers **/
//TODO


/** Datalog functions
 * 
 */

int datalog_s_setup (void);

int datalog_s_init  (data_t init_val);

int datalog_s_single_write (unsigned int log_index,
                            unsigned int datum_index,
                            data_t number);

void datalog_s_write (unsigned int index,
                      bbmc_input_encoder_t volatile *state,
                      bbmc_output_motor_t  volatile *contrl);

int datalog_s_print (int range_indeces[4]);





#ifdef __cplusplus
}
#endif


#endif /* _BBMC_DATALOG_H_ */

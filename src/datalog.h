#ifndef _BBMC_DATALOG__H_
#define _BBMC_DATALOG_H_


#ifdef __cplusplus
    extern "C" {
#endif



/**
 * 
 */
#define DATALOG_STATIC_DATALEN          (GLOBAL_DATA_MAX)
#define DATALOG_STATIC_DATASIZE         (6)



/**
 * 
 */
typedef double data_t;


/**
 * 
 */
typedef struct
{
    data_t data[DATALOG_STATIC_DATASIZE];
}
dataset_s_t;


/**
 * 
 */
typedef struct
{
    dataset_s_t log[DATALOG_STATIC_DATALEN];
    
    int d_size;
    int l_size;
    
    int d_index;
    int l_index;
}
datalog_s_t;


/**
 * 
 */

int datalog_s_setup (datalog_s_t volatile *datalog_ptr);

int datalog_s_init (datalog_s_t volatile *datalog_ptr, data_t init_val);

int datalog_s_print (datalog_s_t volatile *datalog_ptr, int range_indeces[4]);

void datalog_write (unsigned int index,
                           datalog_s_t volatile *datalog, 
                           bbmc_dof_state_t volatile *state,
                           bbmc_dof_contrl_t volatile *contrl);





#ifdef __cplusplus
}
#endif


#endif /* _BBMC_DATALOG_H_ */

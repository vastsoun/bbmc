#ifndef _GLOBAL_SYSTEM_FLAGS_H_
#define _GLOBAL_SYSTEM_FLAGS_H_


#ifdef __cplusplus
    extern "C" {
#endif


/** Usage Datatypes 
 * 
 */

typedef enum
{
    CMD_LINE    = 0,
    DEBUG       = 1,
    PERFLOG     = 2,
    DATALOG     = 3,
    EXEC_CHK    = 4,
    CONTRL_RUN  = 5,
    ISR_RET     = 6,
    STOP_EMR    = 7,
    GPOS_RESET  = 8,
    
    ALL         = 9,
    ISR         = 10,
    POS_Y_RESET = 11,
    POS_X_RESET = 12,
    LOGS        = 13,
    CLI         = 14,
    CMD         = 15
}
system_flag;


typedef enum
{
    MIN = 0,
    MAX = 1  
}
pos_reset;


typedef enum
{
    CLEAN  = ISR_RETURN_CLEAN,
    KILLSW = ISR_RETURN_KILLSW,
    POSLIM = ISR_RETURN_POSLIM,
    ERROR  = ISR_RETURN_ERROR,
    DEBUG  = ISR_RETURN_DEBUG
}
isr_ret_t;




/** System Flag Handler Functions
 *  
 */

int global_flag_get   (system_flag flag);

int global_flag_set   (system_flag flag);

int global_flag_clear (system_flag flag);


int global_flags_set   (system_flag flag);

int global_flags_clear (system_flag flag);


int global_flags_gpreset_set (int dev_id, pos_reset value);
                                     
pos_reset global_flags_gpreset_get (int dev_id);


int global_flag_isr_set (isr_ret_t value);

isr_ret_t global_flag_isr_get (void);


int global_flags_print (const char *format);




#ifdef __cplusplus
}
#endif


#endif /* _GLOBAL_SYSTEM_FLAGS_H_ */

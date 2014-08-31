#ifndef _GLOBAL_SYSTEM_FLAGS_H_
#define _GLOBAL_SYSTEM_FLAGS_H_


#ifdef __cplusplus
    extern "C" {
#endif



/** Module Dependencies **/
#include "device_layer.h"
#include "isr_manager.h"



/** Usage Datatypes 
 * 
 */

typedef enum
{
    FLG_CMDLINE     = 0,
    FLG_DEBUG       = 1,
    FLG_PERFLOG     = 2,
    FLG_DATALOG     = 3,
    FLG_EXEC_CHK    = 4,
    FLG_CONTRL_RUN  = 5,
    FLG_ISR_RET     = 6,
    FLG_STOP_EMR    = 7,
    FLG_GPOS_RESET  = 8,
    
    FLG_ALL         = 9,
    FLG_ISR         = 10,
    FLG_POS_Y_RESET = 11,
    FLG_POS_X_RESET = 12,
    FLG_LOGS        = 13,
    FLG_CLI         = 14,
    FLG_CMD         = 15
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
    RET_CLEAN  = ISR_RETURN_CLEAN,
    RET_KILLSW = ISR_RETURN_KILLSW,
    RET_POSLIM = ISR_RETURN_POSLIM,
    RET_ERROR  = ISR_RETURN_ERROR,
    RET_DEBUG  = ISR_RETURN_DEBUG
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

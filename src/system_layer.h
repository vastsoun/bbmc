#ifndef _SYSTEM_LAYER_H_
#define _SYSTEM_LAYER_H_


#ifdef __cplusplus
    extern "C" {
#endif



/** 
 *  
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











/** System Flag handler functions
 *  
 */
int sys_flags_clear  (const char *init_mode);

int sys_flags_print (const char *format);

int sys_flags_set (const char *init_mode);
                             
int sys_flags_get (const char *flag);

int sys_flags_gpreset_set (int dof_id, int set_val);
                                     
int sys_flags_gpreset_get (int dof_id);






#ifdef __cplusplus
}
#endif


#endif /* _SYSTEM_LAYER_H_ */

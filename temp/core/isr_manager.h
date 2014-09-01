#ifndef _ISR_MANAGER_H_
#define _ISR_MANAGER_H_


#ifdef __cplusplus
    extern "C" {
#endif



/** Module Macros to define ISR return values
 * 
 */
#define ISR_RETURN_CLEAN                (0)
#define ISR_RETURN_KILLSW               (-100)
#define ISR_RETURN_POSLIM               (-200)
#define ISR_RETURN_ERROR                (-300)
#define ISR_RETURN_DEBUG                (-400)



/** Module Data Types
 *  
 */
typedef enum
{
    ITER_COUNT = 0,
    TERM_FLAG  = 1,
    TERM_COUNT = 2,
    TERM_RET   = 3
}
isr_state;



/** ISR state funtions 
 * 
 */

int isr_state_init (void);

int isr_state_set (isr_state set_mode, int set_val);

int isr_state_get (isr_state get_mode);

int isr_state_print (const char* format);



/** ISR management functions
 * 
 */
 
int isr_master_enable  (void);

int isr_master_disable (void);

int isr_function_setup (void);



/** ISR return value configurer
 * 
 */
int isr_return_value(unsigned int cmnd_ret, int ret);


#ifdef __cplusplus
}
#endif


#endif /* _ISR_MANAGER_H_ */

#ifndef _ISR_MANAGER_H_
#define _ISR_MANAGER_H_


#ifdef __cplusplus
    extern "C" {
#endif



/** Module Data Types
 *  
 */
typedef enum
{
    TERM_FLAG  = 0,
    ITER_COUNT = 1,
    TERM_COUNT = 2
    
}
isr_state_field;


typedef struct
{
    unsigned int ticks;
    unsigned int flag;
}
systick_t;



/** ISR state funtions 
 * 
 */

int isr_state_init (void);

int isr_state_set (isr_state_field set_mode, int set_val);

int isr_state_get (isr_state_field get_mode);

int isr_state_print (const char* format);



/** ISR management functions
 * 
 */
 
int isr_master_enable (void);

int isr_master_disable (void);

int isr_function_setup(void);



/** ISR return value configurer
 * 
 */
int isr_return_value(unsigned int cmnd_ret, int ret);


#ifdef __cplusplus
}
#endif


#endif /* _ISR_MANAGER_H_ */

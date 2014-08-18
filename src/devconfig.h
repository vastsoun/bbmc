#ifndef _DEVICE_CONFIG__H_
#define _DEVICE_CONFIG_H_


#ifdef __cplusplus
    extern "C" {
#endif



/** BBMC HARDWARE MACROS
 * 
 */
#define DEVICE_INPUT_FUNC_NUM           (3)
#define DEVICE_OUTPUT_FUNC_NUM          (2)


#define TIMER_RUN                       2
#define TIMER_GOTO                      3
#define TIMER_RMPI                      4
#define TIMER_STOP                      5

#define DMTIMER_CONTROLLER              (SOC_DMTIMER_2_REGS)
#define DMTIMER_GOTO                    (SOC_DMTIMER_3_REGS)
#define DMTIMER_RMPI                    (SOC_DMTIMER_4_REGS)
#define DMTIMER_STOP                    (SOC_DMTIMER_5_REGS)

#define DIR_1_GPIO_ADDRESS              (SOC_GPIO_1_REGS)
#define DIR_1_GPIO_PIN_ADDRESS          (GPIO_1_7)     
#define DIR_1_GPIO_PIN_MODE             (CONTROL_CONF_MUXMODE(7))
#define DIR_1_GPIO_PIN                  (7)
#define DIR_1_GPIO_PIN_DIR              (GPIO_DIR_OUTPUT)

#define DIR_2_GPIO_ADDRESS              (SOC_GPIO_1_REGS)
#define DIR_2_GPIO_PIN_ADDRESS          (GPIO_1_3)     
#define DIR_2_GPIO_PIN_MODE             (CONTROL_CONF_MUXMODE(7))
#define DIR_2_GPIO_PIN                  (3)
#define DIR_2_GPIO_PIN_DIR              (GPIO_DIR_OUTPUT)

#define HALL_Y_GPIO_ADDRESS             (SOC_GPIO_1_REGS)
#define HALL_Y_GPIO_PIN                 (6)
#define HALL_Y_GPIO_INT_LINE            (GPIO_INT_LINE_1)

#define HALL_X_GPIO_ADDRESS             (SOC_GPIO_1_REGS)
#define HALL_X_GPIO_PIN                 (2)
#define HALL_X_GPIO_INT_LINE            (GPIO_INT_LINE_1)

#define KILLSWITCH_GPIO_ADDRESS         (SOC_GPIO_1_REGS)
#define KILLSWITCH_GPIO_PIN             (14)
#define KILLSWITCH_GPIO_INT_LINE        (GPIO_INT_LINE_2)
#define KILLSWITCH_GPIO_DEBOUCE_TIME    (1000)



/** 
 *  
 *  
 */
typedef dmtimer_handle_t       bbmc_dev_timer_t;
typedef eqep_handle_t           bbmc_dev_encoder_t;
typedef ehrpwm_handle_t         bbmc_dev_pwm_t;
typedef gpio_device_pin_t      bbmc_dev_gpio_pin_t;


/** 
 *  
 *  
 */


/** 
 *  
 *  
 */




/** 
 *  
 *  
 */




/** 
 *  
 *  
 */








#ifdef __cplusplus
}
#endif


#endif /* _DEVICE_CONFIG_H_ */

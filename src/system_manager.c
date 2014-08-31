/** Module Header **/
#include "system_manager.h"

/** Std C Headers **/
#include <stdlib.h>
#include <errno.h>
#include <ctype.h>
#include <math.h>

/** Firmware Headers **/
#include "uartStdio.h"

/** BBMC Headers **/
#include "device_layer.h"
#include "isr_manager.h"
#include "system_timers.h"
#include "safety_ss.h"

#include "motor_control.h"
#include "global_flags.h"
#include "global_state.h"

//#include "uretts_model.h"
//#include "uretts_control.h"

#include "datalog.h"
//#include "signal_generator.h"

#include "bbmc_facilities.h"
//#include "goto.h"
//#include "rmpi.h"
//#include "experiement.h"

#include "util.h"




/** Internal Data Types
 *  
 */



/** Internal Data
 *  
 */



/** Device Configurations
 * 
 */

int 
sys_devices_setup (void)
{
    return dev_setup();
}


/** ISR Management
 * 
 */

int 
sys_isr_setup (void)
{
    isr_setup ();
    isr_state_init ();
    
    return 0;
}


/** System Timers Managment
* 
*/
//TODO: handle the timers' states


/** Fault/Safety Subsystem Management
 * 
 */
//TODO ?


/** Motion Control & System I/O Management
 * 
 */

int 
sys_motor_control_setup (void)
{
    io_func_setup ();
    
    return 0;
}


/** Global Flag Management
 *  
 */
int 
sys_gflag_setup (void)
{
    global_flags_clear (FLG_ALL);
    
    return 0;
}


/** Global State Data Management
 *  
 */

int 
sys_gstate_setup (void)
{
    global_presets_setup ();
    
    global_state_init ();
    
    return 0;
}


/** Datalog Management
 *  
 */

int 
sys_logs_init (void)
{
    UARTPuts("\r\n\r\nConfiguring and Initializing <datalog> and <perf> handlers: ", -1);
    
    datalog_s_setup ();
    datalog_s_init (0);
    performance_log_init ();
    
    UARTPuts("\tDONE\r\n", -1);
    
    return 0;
}



/** Signal Generator Configurations
 * 
 */
//TODO
int 
sys_signal_gen_setup (void)
{
    
    return 0;
}


/** URETTS Carriage Model & Paramters
 * 
 */
//TODO
int 
sys_uretts_setup (void)
{
    
    return 0;
}


/** BBMC Facility Management
 * 
 */
int 
sys_facilities_setup (void)
{
    
    return 0;
}


/** BBMC GOTO Setup
 * 
 */
int 
sys_goto_setup (void)
{
    
    return 0;
}


/** BBMC RMPI Setup
 * 
 */
int 
sys_rmpi_setup (void)
{
    
    return 0;
}


/** BBMC Experiment Setup
 * 
 */
int 
sys_experiment_setup (void)
{
    
    return 0;
}




/** BBMC Master Init
 * 
 */

int 
bbmc_setup (void)
{
    sys_devices_setup ();
    sys_isr_setup ();
    sys_motor_control_setup ();
    
    sys_gflag_setup ();
    sys_gstate_setup ();
    
    sys_logs_init ();
    sys_signal_gen_setup ();
    
    sys_uretts_setup ();
    
    sys_facilities_setup ();
    sys_goto_setup ();
    sys_rmpi_setup ();
    sys_experiment_setup ();
    
    return 0;
}



/** System Halt & Power-down.
 * 
 */

void 
bbmc_halt (void)
{
    //TODO: Add power down and cleanup functions
    
    UARTPuts("\r\nSystem Halt: System is shuting down. Goodbye!\r\n", -1);
    
    for (;;)
    {
        ;
    }
}




/**
 * 
 */

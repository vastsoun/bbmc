#ifndef _SYSTEM_MANAGEMENT_H_
#define _SYSTEM_MANAGEMENT_H_


#ifdef __cplusplus
    extern "C" {
#endif


/** Devices Configuration
 * 
 */

int sys_devices_setup (void);


/** ISR Management
 * 
 */

int sys_isr_setup (void);


/** System Timers Managment
* 
*/



/** Fault/Safety Subsystem Management
 * 
 */



/** Motion Control & System I/O Management
 * 
 */

int sys_motor_control_setup (void);


/** Global Flag Management
 *  
 */
int sys_gflag_setup (void);


/** Global State Data Management
 *  
 */

int sys_gstate_setup (void);


/** Datalog Management
 *  
 */

int sys_logs_init (void);


/** Signal Generator Configurations
 * 
 */

int sys_signal_gen_setup (void);


/** URETTS Carriage Model & Paramters
 * 
 */

int sys_uretts_setup (void);


/** URETTS Controller
 * 
 */

int sys_uretts_setup (void);


/** BBMC Facility Management
 * 
 */

int sys_facilities_setup (void);


/** BBMC GOTO Setup
 * 
 */

int sys_goto_setup (void);


/** BBMC RMPI Setup
 * 
 */
int sys_rmpi_setup (void);


/** BBMC Experiment Setup
 * 
 */
int sys_experiment_setup (void);


/** BBMC Master Init
 * 
 */

int bbmc_setup (void);


/** System Halt & Power-down.
 * 
 */

void bbmc_halt (void);





#ifdef __cplusplus
}
#endif


#endif /* _SYSTEM_MANAGEMENT_H_ */

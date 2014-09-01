#ifndef _TEMPLATE__H_
#define _TEMPLATE_H_


#ifdef __cplusplus
    extern "C" {
#endif


#include <math.h>


/** System Macros
 *  
 */

#define MAX_DURATION_COUNT              (131072)
#define MAX_DEBUG_COUNT                 (2000)


/** BBMC TRAJECTORY MACROS
 *  
 */
#define TRAJECTORY_POINTS_NUM         (DATA_SIGLEN_MAX)
#define ATAN2_ZERO_ERROR              100


/** 
 *  
 *  
 */
typedef struct 
{
    double  x;
    double  x_dot;
    double  x_ddot;
}
taskspace_state_t;




/* trajectory generation functions */
void traject_null (bbmc_dof_state_t volatile *state, 
                   bbmc_dof_contrl_t volatile *controller);



#ifdef __cplusplus
}
#endif


#endif /* _TEMPLATE_H_ */

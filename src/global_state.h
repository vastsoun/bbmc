


/* machine state data */
static bbmc_input_encoder_t volatile  g_dof_state[BBMC_DOF_NUM];


static bbmc_limits_motor_t            g_position_limits[BBMC_DOF_NUM];
static bbmc_limits_motor_t            g_position_init[BBMC_DOF_NUM];
static bbmc_limits_motor_t            g_position_home[BBMC_DOF_NUM];


/* machine controller data */
static bbmc_contrl_motor_t volatile  g_args_experiment[BBMC_DOF_NUM];
static bbmc_contrl_motor_t volatile  g_args_goto[BBMC_DOF_NUM];
static bbmc_contrl_motor_t volatile  g_args_stop[BBMC_DOF_NUM];

#include "../mobilebot/mobilebot.h"

/*******************************************************************************
* int mb_initialize()
*
* this initializes all the PID controllers from the configuration file
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/

int mb_initialize_controller(){
    //mb_load_controller_config();
    left_wheel_velocity_pid = rc_filter_empty();
    right_wheel_velocity_pid = rc_filter_empty();
   
    if(rc_filter_first_order_lowpass(&right_wheel_velocity_pid, DT, 20*DT)) return -1;
    if(rc_filter_first_order_lowpass(&left_wheel_velocity_pid, DT, 20*DT)) return -1;
    
    rc_filter_pid(&left_wheel_velocity_pid, 1.992, 41.08, 0.024, 0.1, DT); 
    rc_filter_pid(&right_wheel_velocity_pid,1.992, 41.08, 0.024,0.1,  DT); 
    rc_filter_enable_saturation(&left_wheel_velocity_pid, -1.0, 1.0);
    rc_filter_enable_saturation(&right_wheel_velocity_pid, -1.0, 1.0);

     //1.5
    fwd_output = rc_filter_empty();
    turn_output = rc_filter_empty();
   
    if(rc_filter_first_order_lowpass(&fwd_output, DT, 20*DT)) return -1;
    if(rc_filter_first_order_lowpass(&turn_output, DT, 20*DT)) return -1;
    
    rc_filter_pid(&fwd_output, 1,0,0, 0.1, DT); 
    rc_filter_pid(&turn_output,1,0,0,0.1, DT); 
    rc_filter_enable_saturation(&fwd_output, -1.0, 1.0);
    rc_filter_enable_saturation(&turn_output, -1.0, 1.0);

    return 0;
}

/*******************************************************************************
* int mb_load_controller_config()
*
* this provides a basic configuration load routine
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/
int mb_load_controller_config(){
    FILE* file = fopen(CFG_PATH, "r");
    if (file == NULL){
        printf("Error opening pid.cfg\n");
    }

/******
*
*   Example of loading a line from .cfg file:
*
*    fscanf(file, "%f %f %f %f", 
*        &pid_params.kp,
*        &pid_params.ki,
*        &pid_params.kd,
*        &pid_params.dFilterHz
*        );
*
******/
    fclose(file);
    return 0;
}

/*******************************************************************************
* int mb_controller_update()
* 
* TODO: Write your PID controller here
* take inputs from the global mb_state
* write outputs to the global mb_state
*
* return 0 on success
*
*******************************************************************************/

int mb_controller_update(mb_state_t* mb_state, mb_setpoints_t* mb_setpoints){
//rc_filter_prefill_inputs(&setpoint_filter, mb_setpoints   
// if(rc_filter_first_order_lowpass(&mb_setpoints->fwd_velocity,DT, 20*DT)) return -1; 
   
    //1.5
    mb_state->fwd_velocity = (mb_state->right_velocity + mb_state->left_velocity)/2;
    mb_state->turn_velocity = (mb_state->right_velocity - mb_state->left_velocity)/WHEEL_BASE;
    float fwd_error = mb_setpoints->fwd_velocity - mb_state->fwd_velocity;
    float turn_error = mb_setpoints->turn_velocity - mb_state->turn_velocity;

    float left_calibration = (1.912590 +  mb_setpoints->fwd_velocity) / 51.455893;
    float right_calibration = (2.386846 +  mb_setpoints->fwd_velocity) / 44.075490;
    //right = rc_filter_march(&fwd_output,fwd_error) + WHEEL_BASE*rc_filter_march(&turn_output,turn_error)/2;
    //left = rc_filter_march(&fwd_output,fwd_error) - WHEEL_BASE*rc_filter_march(&turn_output,turn_error)/2;

    //1.2
    float left_error = mb_setpoints->fwd_velocity - mb_setpoints->turn_velocity * WHEEL_BASE /2 - mb_state->left_velocity;
    float right_error = mb_setpoints->fwd_velocity + mb_setpoints->turn_velocity * WHEEL_BASE/2  - mb_state->right_velocity;

    //float left_error = rc_filter_march(&fwd_output,fwd_error) - WHEEL_BASE*rc_filter_march(&turn_output,turn_error)/2 - mb_state->left_velocity;
    //float right_error = rc_filter_march(&fwd_output,fwd_error) + WHEEL_BASE*rc_filter_march(&turn_output,turn_error)/2 - mb_state->right_velocity;

   // pwm to motors
    mb_state->left_cmd = rc_filter_march(&left_wheel_velocity_pid,left_error) + left_calibration;
    mb_state->right_cmd = rc_filter_march(&right_wheel_velocity_pid,right_error) + right_calibration;
   
   
    return 0;
}


/*******************************************************************************
* int mb_destroy_controller()
* 
* TODO: Free all resources associated with your controller
*
* return 0 on success
*
*******************************************************************************/

int mb_destroy_controller(){
    rc_filter_free(&left_wheel_velocity_pid);
    rc_filter_free(&right_wheel_velocity_pid);
    
    //1.5
    rc_filter_free(&fwd_output);
    rc_filter_free(&turn_output);
    return 0;

}
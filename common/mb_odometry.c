/*******************************************************************************
* mb_odometry.c
*
* TODO: Implement these functions to add odometry and dead rekoning 
*
*******************************************************************************/

#include "../mobilebot/mobilebot.h"
#include "mb_defs.h"
#include <math.h>

#define PI 3.14159265358979323846

/*******************************************************************************
* mb_initialize_odometry() 
*
* TODO: initialize odometry
* NOTE: you should initialize from Optitrack data if available
*
*******************************************************************************/
void mb_initialize_odometry(mb_odometry_t* mb_odometry, float x, float y, float theta){
    mb_odometry->x = x;
    mb_odometry->y = y;
    mb_odometry->theta = theta;
}


/*******************************************************************************
* mb_update_odometry() 
*
* TODO: calculate odometry from internal variables
*       publish new odometry to lcm ODOMETRY_CHANNEL
*
*******************************************************************************/
void mb_update_odometry(mb_odometry_t* mb_odometry, mb_state_t* mb_state){
    float left_delta = (WHEEL_DIAMETER / 2) * (2 * PI * ( mb_state->left_encoder_delta) / (ENCODER_RES * GEAR_RATIO));
    float right_delta = (WHEEL_DIAMETER / 2) *( 2 * PI *  (mb_state->right_encoder_delta) / (ENCODER_RES * GEAR_RATIO));
    float total_delta = (right_delta + left_delta) / 2;
    float theta_delta = (right_delta - left_delta) / WHEEL_BASE;

    // update positions 
    mb_odometry->x = mb_odometry->x + total_delta * cos(mb_odometry->theta + theta_delta/2);
    mb_odometry->y = mb_odometry->y + total_delta * sin(mb_odometry->theta + theta_delta/2);

    // gyro-odometry
    float gyro_delta = mb_state->tb_angles[2] - mb_state->last_yaw;
    float GO_delta = gyro_delta - theta_delta;
    float theta_thres = 0.04363;
    if (fabs(GO_delta) > theta_thres) {
        mb_odometry->theta = mb_odometry->theta + theta_delta;
    } else {
        mb_odometry->theta = mb_odometry->theta + gyro_delta;
    }

}


/*******************************************************************************
* mb_clamp_radians() 
* clamp an angle from -PI to PI
*******************************************************************************/
float mb_clamp_radians(float angle){

    if(angle < -PI)
    {
        for(; angle < -PI; angle += 2.0*PI);
    }
    else if(angle > PI)
    {
        for(; angle > PI; angle -= 2.0*PI);
    }

    return angle;
}


/*******************************************************************************
* mb_angle_diff_radians() 
* computes difference between 2 angles and wraps from -PI to PI
*******************************************************************************/
float mb_angle_diff_radians(float angle1, float angle2){
    float diff = angle2 - angle1;
    while(diff < -PI) diff+=2.0*PI;
    while(diff > PI) diff-=2.0*PI;
    return diff;
}

/*******************************************************************************
* measure_motor_params.c
*   Template code 
*   Complete this code to automatically measure motor parameters
*   or print out data to be namalyzed in numpy
* 
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <rc/start_stop.h>
#include <rc/encoder_eqep.h>
#include <rc/encoder.h>
#include <rc/adc.h>
#include <rc/time.h>
#include <rc/motor.h>
#include "../common/mb_motor.h"
#include "../common/mb_defs.h"


float enc2meters = (WHEEL_DIAMETER * M_PI) / (GEAR_RATIO * ENCODER_RES);

float test_speed(int ticks);

/*******************************************************************************
* int main() 
*
*******************************************************************************/
int main(){

	// make sure another instance isn't running
    // if return value is -3 then a background process is running with
    // higher privaledges and we couldn't kill it, in which case we should
    // not continue or there may be hardware conflicts. If it returned -4
    // then there was an invalid argument that needs to be fixed.
    if(rc_kill_existing_process(2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
    if(rc_enable_signal_handler()==-1){
                fprintf(stderr,"ERROR: failed to start signal handler\n");
                return -1;
    }

#if defined(MRC_VERSION_1v3) || defined(MRC_VERSION_2v1)
    if(mb_motor_init()<0){
        fprintf(stderr,"ERROR: failed to initialze mb_motors\n");
        return -1;
    }
#endif

#if defined(BEAGLEBONE_BLUE)
    if(rc_motor_init()<0){
        fprintf(stderr,"ERROR: failed to initialze motors\n");
        return -1;
    }
#endif

    if(rc_encoder_eqep_init()<0){
        fprintf(stderr,"ERROR: failed to initialze encoders\n");
        return -1;
    }
    
    // make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();

	// done initializing so set state to RUNNING
	rc_set_state(RUNNING);

	if(rc_get_state()==RUNNING){
		for(float pwm = 0.0; pwm <=1.0;pwm+=0.05){
			rc_motor_set(1,pwm);
			rc_motor_set(2,pwm);
			rc_nanosleep(2E8);
			rc_encoder_eqep_write(1,0.0);
			rc_encoder_eqep_write(2,0.0);
			rc_nanosleep(1E9);
			int tick1 = rc_encoder_eqep_read(1);
			int tick2 = rc_encoder_eqep_read(2);
			float speed1 = test_speed(tick1);
			float speed2 = test_speed(tick2);
			printf("%f, %f, %f\n",pwm,speed1,speed2);
		}
	}
	
	rc_motor_set(1,0.0);
	rc_motor_set(2,0.0);
	rc_encoder_eqep_cleanup();
	rc_motor_cleanup();
    // remove pid file LAST
	rc_remove_pid_file();   
	return 0;
}

float test_speed(int  ticks){
    float speed = enc2meters * ticks / 1;
    return speed;   
}

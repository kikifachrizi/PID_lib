a library used for easyly when we want control omnidirectional robot
there three params that you must change to matching with your system, the parameter is kp ki kd
the derivative parameter has been filtered with low pas filter butterworth orde 1 with cut-off frequency 25Khz

example: 
#include "PID_lib.h"

//            dir1   dir2   pwm
PID_lib pid1(PE_11, PF_14, PE_13);
Timer t;

int main(){
    t.reset();
    t.start();

    while(1){
        float rpm = rpm.readRPM();
        pid1.pid_pwm(250 , 1.0 , 0.001 , 0.0003, rpm , t.read_high_resolution());
        wait(5);
        pid1.stop();
    }
}
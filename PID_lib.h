/*
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

*/

#ifndef PID_lib_H
#define PID_lib_H

#include "mbed.h"

class PID_lib {

public:

    PID_lib(PinName direksi1, PinName direksi2, PinName pulseWidth);
  
    void pid_pwm(float target_, float kp_, float ki_, float kd_, float rpm, float t_);
    void pwm_read(float target_, float kp_, float ki_, float kd_, float rpm, float t_);
    void stop();
    void manualPwm(float speed_);
    float record_data(int dir_,float speed_,float freq, float t_r);
    void useParams1();
    void printParams();

private:
       Timer tr;
       DigitalOut dir1;
       DigitalOut dir2;
       PwmOut pwm;
       float freq_rec;
       float rpm_rec;
       float rpm;
       float freq;
       float ppr;
       float e;
       float laste;
       float eI;
       float eD;
       float dt;
       float tim;
       float lastime;
       float pidPwm;
       float pwmLebih;
       float lastPid;
       float hP;
       float hI;
       float hD;
       float setI;
       
       float rpmFilt;
       float rpmFiltn1;
       float rpmn1;
};

#endif 

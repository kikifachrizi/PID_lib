#include "PID_lib.h"
#include <cstdio>


PID_lib::PID_lib(PinName direksi1, PinName direksi2, PinName pulseWidth) : dir1(direksi1), dir2(direksi2), pwm(pulseWidth){    
    dir1 = 0;
    dir2 = 0;
    pwm = 0;
    
    ppr = 270.0f;
    double phi = 3.14285714;

    t_pid.reset();
    t_pid.start();
}

void PID_lib::reset_timer(){
    t_pid.reset();
    t_pid.start();

    printf("%llu\n", t_pid.read_high_resolution_us());
}

float PID_lib::compute(double target_, double kp_, double ki_, double kd_ , double sensor_){
    tim = t_pid;
    dt = tim - lastime;
    //error computing start          
    e = target_ - sensor_;
    eI += e;
    eD = e - laste;
    //error computing end
    //storing error value start
    hI = eI*ki_*dt;
    //storing error value end
    //saturasi ki start
    if(hI < 0.5 && hI > 0){
        setI = hI;
    }else if(hI > 0.5){
        setI = 0.5;
    }
    else if(hI < 0 && hI > -0.5){
        setI = hI;
    }else if(hI < -0.5){
        setI = -0.5;
    }
    //saturasi ki end
    
    pid = e*kp_+setI+eD*kd_/dt;//pwm pid
    lastime = tim;//update timer
    laste = e;//update error

    //pidSat saturasi start
    if(pid > 0.5){
        pidSat = 0.5;
    }else if(pid < 0.5 && pid > 0){
        pidSat = fabs(pid);
    }else if(pid < -0.5){
        pidSat = -0.5;
    }else if(pid > -0.5 && pid < 0){
        pidSat = fabs(pid);
    }
    //pidSat saturasi end
    
    // pid cek sample time
    if(dt < 0.001){pid_out = fabs(lastPid);}
    else{pid_out = fabs(pid);}

    lastPid = pid;

    return pid_out;
}

void PID_lib::manualPwm(double speed_){
    if(speed_ < 0){
    //   pwm.write(speed_);
      dir1 = 0;
      dir2 = 1; 
    }else if(speed_ > 0){
    //   pwm.write(speed_);
      dir1 = 1;
      dir2 = 0; 
    }
    pwm.period(0.010);
    pwm.write(abs(speed_));
    // printf("%f;\t",pwm.read());
}

void PID_lib::stop(){
    rps = 0;
    rpsn1 = 0;
    rpsFilt = 0;
    rpsFiltn1 = 0;
    lastPid = 0;
    e = 0;
    eI = 0;
    eD = 0;
    pidPwm = 0;
    pwm = 0;
    dir1 = 0;
    dir2 = 0;
}

void PID_lib::pos(double target_, double kp_ , double ki_ , double kd_ , double angle_ , double t_){
    errAngle = target_ - angle_;
    while(errAngle > 10 || errAngle < -10){
        pwm.period(0.010);
        pwm = 0.5;
        if(errAngle < 0){
            dir1 = 0;
            dir2 = 1;
        }else{
            dir1 = 1;
            dir2 = 0;
        }
    }

    while(errAngle > 5 || errAngle < -5){
        pwm.period(0.010);
        pwm = 0.2;
        if(errAngle < 0){
            dir1 = 0;
            dir2 = 1;
        }else{
            dir1 = 1;
            dir2 = 0;
        }
    }
    while(errAngle > 1 || errAngle < -1){

    }
    stop();
}

void PID_lib::pid_speed(double target_, double kp_,double ki_,double kd_, double rpm, double t_){
  tim = t_;
  dt = tim - lastime;
  
//   rpm = freq/ppr*60;
//   rpsFilt = 0.03046875*rps + 0.03046875*rpsn1 + 0.93906251*rpsFiltn1;//10hz filter 

  //error computing start          
  e = target_ - rpm;
  eI += e;
  eD = e - laste;
  //error computing end
  
  //storing error value start
  hP = e*kp_;
  hI = eI*ki_*dt;
  hD = eD*kd_/dt;
  //storing error value end
  
  //saturasi ki start
  if(hI < 0.5 && hI > 0){
    setI = hI;
  }else if(hI > 0.5){
    setI = 0.5;
  }
  else if(hI < 0 && hI > -0.5){
    setI = hI;
  }else if(hI < -0.5){
    setI = -0.5;
  }
  //saturasi ki end
  
  pidPwm = hP+setI+hD;//pwm pid
  lastime = tim;//update timer
  laste = e;//update error
  
  rpsFiltn1 = rpsFilt;rpsn1 = rps;//update filter
  //motor direksi start
  if(pidPwm < 0){dir1 = 1; dir2 = 0;}
  else{dir1 = 0; dir2 = 1;}
  //motor direksi end
  
  //pwm saturasi start
  if(pidPwm > 0.5){
     pwmLebih = 0.5;
  }else if(pidPwm < 0.5 && pidPwm > 0){
     pwmLebih = fabs(pidPwm);
  }else if(pidPwm < -0.5){
     pwmLebih = -0.5;
  }else if(pidPwm > -0.5 && pidPwm < 0){
     pwmLebih = fabs(pidPwm);
  }
  //pwm saturasi end
  
  //motor start
  pwm = fabs(pwmLebih);
  //motor end
  
  lastPid = pidPwm;
//  printf("%f\n", rpmFilt);
//    printf("%f;%f;%f\n",e,eI,eD);
    printf("%f\n", rpm);
//    printf("%f;%f;%f\n",hP,setI,hD);
//  pc.printf("%f;%1lu\n",pidPwm,t1.read_high_resolution_us());
}


//       rpmFilt = 0.03046875*rpm_rec + 0.03046875*rpmn1 + 0.93906251*rpmFiltn1;//10 hz
//       rpmFilt = 0.04503501*rpm_rec + 0.04503501*rpmn1 + 0.90992999*rpmFiltn1;//15 hz
//       rpmFilt = 0.0591907*rpm_rec + 0.0591907*rpmn1 + 0.88161859*rpmFiltn1;//20 hz
//       rpmFilt = 0.07295966*rpm_rec_ + 0.07295966*rpmn1 + 0.85408069*rpmFiltn1;//25 hz
//       rpmFilt = 0.09942446*rpm_rec + 0.09942446*rpmn1 + 0.80115107*rpmFiltn1;//35 hz
//       rpmFilt = 0.13672874*rpm_rec + 0.13672874*rpmn1 + 0.72654253*rpmFiltn1;//50 hz
      


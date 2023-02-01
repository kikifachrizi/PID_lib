#include "PID_lib.h"


PID_lib::PID_lib(PinName direksi1, PinName direksi2, PinName pulseWidth) : dir1(direksi1), dir2(direksi2), pwm(pulseWidth){    
    dir1 = 0;
    dir2 = 0;
    pwm = 0;
    
    ppr = 3600.0f;
    double phi = 3.14285714;
    
//    t.start();
//    tr.reset();
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

void PID_lib::pid_speed(double target_, double kp_,double ki_,double kd_, double rps, double t_){
  tim = t_;
  dt = tim - lastime;
  
//   rpm = freq/ppr*60;
  rpsFilt = 0.03046875*rps + 0.03046875*rpsn1 + 0.93906251*rpsFiltn1;//10hz filter 

  //error computing start          
  e = target_ - rps;
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
    printf("%f;%f\n", rps, rpsFilt);
//    printf("%f;%f;%f\n",hP,setI,hD);
//  pc.printf("%f;%1lu\n",pidPwm,t1.read_high_resolution_us());
}


//       rpmFilt = 0.03046875*rpm_rec + 0.03046875*rpmn1 + 0.93906251*rpmFiltn1;//10 hz
//       rpmFilt = 0.04503501*rpm_rec + 0.04503501*rpmn1 + 0.90992999*rpmFiltn1;//15 hz
//       rpmFilt = 0.0591907*rpm_rec + 0.0591907*rpmn1 + 0.88161859*rpmFiltn1;//20 hz
//       rpmFilt = 0.07295966*rpm_rec_ + 0.07295966*rpmn1 + 0.85408069*rpmFiltn1;//25 hz
//       rpmFilt = 0.09942446*rpm_rec + 0.09942446*rpmn1 + 0.80115107*rpmFiltn1;//35 hz
//       rpmFilt = 0.13672874*rpm_rec + 0.13672874*rpmn1 + 0.72654253*rpmFiltn1;//50 hz
      


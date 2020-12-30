#include <algorithm>
#include "PIDControler.h"
#include "ProcessorMulti_Processor_Core_Vars.h"

void PIDControler::set_ks(double kp, double ki, double kd) {
    k_p = kp;
    k_i = ki;
    k_d = kd;
}

void PIDControler::reset() {
    err_prev = 0;
    err_integrate = 0;
}

double PIDControler::step(double err) {
    err_integrate += err;
    double u_control = k_p * err + k_i * err_integrate + k_d * (err - err_prev);    // using the PID equation
    err_prev = err;
    return u_control;
}

std::pair<short, short>
calc_steer(double dis, double yaw, int laserSize, short *laserData, double laserUnit,
                 ProcessorMulti_Processor_Core_Params *params, ProcessorMulti_Processor_Core_Vars *vars) {

    // 首先判断是否到达目的地，如果到达就停止
    if(vars->is_arrive) {
        steer=vars->baseSteer;
        speed=0;
        return {speed, steer};
    }
    // 接着判断

    short span = 3; // 180 / 3 == 60
    int leftSum = 0, rightSum = 0;
    int l_tot=0, r_tot=0,my_turning;


//先不替换0，l_tot, r_tot以后也可能有用
// 注意捕捉第一个路口是向右还是向左，双路口特判再说
// 统计如果右侧30度的合计小于small_sum, 认为第一个路口是向右
    for(size_t i=300; i<=laserSize; i+=2){
       r_tot += *std::min_element(laserData + i, laserData + i + 1);
    }
    if(r_tot < vars->small_sum)  my_turning=0;

    for(size_t i=0; i<=60; i+=2){
        l_tot += *std::min_element(laserData + i, laserData + i + 1);
    }
    if(l_tot < vars->small_sum) my_turning=1;

    //如果尚未标记，就开始标记第一个路口的方向
    if((!vars->mark_1) && r_tot<vars->small_sum){
        vars->first_turning=0; //第一个路口是向右
        vars->mark_1 = 1; //标记我已经发现了第一个路口了
    }
    else if((!vars->mark_1) && l_tot<vars->small_sum){
        vars->first_turning=1;
        vars->mark_1 = 1;
    }
    //如果左右都是路口，显然按照程序顺序，会被标记为向右

    //比照我是否是沿规划的方向，is_right=0时才需要判断,知道is_right=1以后照走即可
    if(!(vars->is_right)){
        //我在正确的顺时针  || 我在正确地逆时针
        if((!vars->dir) && (vars->first_turning==0))  //顺时钟
            vars->is_right=1; 
        if(vars->dir && vars->first_turning) //逆时针
            vars->is_right=1;
    }



//    bool leftMono = true, rightMono = true;

    std::replace(laserData, laserData+laserSize, 0, vars->infDistance);
   // laserSize == 361
   // sum 60 degrees
   for(size_t i = 0; i <= 120; i+=2) {
       leftSum += *std::min_element(laserData + i, laserData + i + 1);
   }
   for(size_t i = 240; i <= laserSize; i += 2) {
       rightSum += *std::min_element(laserData + i, laserData + i + 1);
   }

   int sumDelta = leftSum - rightSum;
   double err = (double)sumDelta / (double)(laserSize/2 / span);
//   err = err>=0 ? sqrt(abs(err)) : -sqrt(abs(err)); // try to rm this?
   err /= 9;    // make the plot more reasonable
   err = err >= 0 ? err * err : -err * err;
//   qDebug() << err << endl;

   
   int detectRange = vars->forwardDetectRange;

   if(*std::min_element(laserData - detectRange, laserData + detectRange + 1) != 0 && sumDelta < 0 && dir==) {
       // if here is a right corner and there is a wall ahead  // notice that its all turning right
       err += 5;   // then turn right! 一般以右转为优先级
      
   }
   else if(*std::min_element(laserData - detectRange, laserData + detectRange + 1) != 0 && sumDelta < 0 && dir==) {
       // if here is a left corner and there is a wall ahead
       err -= 5;   // then turn left! 有时只有左转路口、前面也有墙，就左转
   }

   qDebug() << err << " ";

   int steer = vars->pid.step(err);
   int speed = 150; // turning should be a bit slower

   if(abs(steer) < vars->straightThreshold) {
       speed = vars->straightSpeed;
   }

   // cope with overflow
   int steerMax = 600;
   int speedMax = 180;
   steer = std::max(steer, -steerMax);
   steer = std::min(steer, steerMax);
   speed = std::max(speed, -speedMax);
   speed = std::min(speed, speedMax);

//   int filt = vars->filterRange * 2;
   short* laserMid = laserData + (laserSize - 1) / 2;
   int safeRange = vars->safeAngle * 2;

   if(vars->reverse && vars->prev_odom - dis > vars->backwardDistance / laserUnit) {
       vars->reverse = false;
   }

   if((*std::min_element(laserMid - safeRange, laserMid + safeRange + 1) < vars->safeDistance) || vars->reverse) {
       qDebug() << *std::min_element(laserMid - safeRange, laserMid + safeRange + 1) << " ";
       speed = -100;
       steer = vars->baseSteer;
       if(!vars->reverse) {
           vars->reverse = true;
           vars->prev_odom = dis;
       }
   }


   return {speed, steer};
}
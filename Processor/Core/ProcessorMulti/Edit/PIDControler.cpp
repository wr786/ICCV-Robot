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
    //如果遇到的第一个路口是左右双叉，显然按照程序顺序，会被标记为向右

    //对比规划的方向，is_right=0时才需要判断,知道is_right=1以后照走即可
    if(!(vars->is_right)){
        //还没有确定正确路径，我在正确的顺时针  || 我在正确地逆时针
        if((!vars->dir) && (vars->first_turning==0))  //顺时钟
            vars->is_right=1; 
        else if(vars->dir && vars->first_turning) //逆时针
            vars->is_right=1;
        else // 肯定是摸索方向与规划方向不同，出错了
        {
            //需要对照地图计算是沿当前方向还是扭头，重点是能够定位，这点也需要先验知识
            //比方说，我应该逆时针走一点就到，结果顺时针走了背面的路口
            //                          //
            //                          //
            //                          //     
            //                          //     
            //now                       //     
            ///////////st//////aim////////
            if(vars->need_chg[st][ed]){  //如果知识告诉我们直接按照现在的路走，不需要管与规划的冲突
                vars->is_right=1;
            }
            else{
                vars->State=2;  //应该进入第一次调整路径的状态
            }
        }
    }
//  以下是State=2的扭头算法
    if((vars->State==2) && (vars->is_right==0)){  //方向盘打死扭头即可
        speed=150;
        steer=600;
        
        vars->time_for_chg--;
        if(vars->time_for_chg == 0){  //认为转弯结束了，此处判断可以结合实时的激光分布细化重写
            vars->is_right =1;  //终于扭头完毕，找到了正确方向
        }
        return {speed, steer};
    }

//前方有路障，进入State=1状态，应该绕障，先探测前方




    if()

//    以下是State=0,走中线的PID控制算法
    std::replace(laserData, laserData+laserSize, 0, vars->infDistance);
   // laserSize == 361
   // sum 60 degrees
   // bool leftMono = true, rightMono = true;
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

   if(*std::min_element(laserData - detectRange, laserData + detectRange + 1) != 0 && sumDelta < 0 ) {
       // if here is a right corner and there is a wall ahead  // notice that its all turning right
       err += 5;   // then turn right! 一般以右转为优先级
      
   }
   else if(*std::min_element(laserData - detectRange, laserData + detectRange + 1) != 0 && sumDelta < 0 ) {
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

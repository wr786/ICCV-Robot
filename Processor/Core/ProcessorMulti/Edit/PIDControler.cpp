#include <algorithm>
#include <cmath>
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

    int steer, speed;

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
        if(vars->dir == CLOCKWISE && (vars->first_turning==0))  //顺时钟
            vars->is_right=1; 
        else if(vars->dir == ANTI_CLOCKWISE && vars->first_turning) //逆时针
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
            if(vars->need_chg[vars->st][vars->ed]){  //如果知识告诉我们直接按照现在的路走，不需要管与规划的冲突
                vars->is_right=1;
            }
            else{
                vars->State = ADJUST;  //应该进入第一次调整路径的状态
            }
        }
    }
//  以下是State=2的扭头算法
    if((vars->State==ADJUST) && (vars->is_right==0)){  //方向盘打死扭头即可
        speed=150;
        steer=600;
        
        vars->time_for_chg--;
        if(vars->time_for_chg == 0){  //认为转弯结束了，此处判断可以结合实时的激光分布细化重写
            vars->is_right =1;  //终于扭头完毕，找到了正确方向,应该接着走中线
        }
        return {speed, steer};
    }
//以下为避障状态，考虑走中线本身的局限性，用找最远
//把0替换为最远大的数值
    std::replace(laserData, laserData+laserSize, 0, vars->infDistance);
    //前方有路障，进入State=1状态，应该绕障，先探测前方
    int MMM=188,totMax=0;  //MMM是较远的一个距离阈值
    int maxi; //maxi是最远的一条激光线，所谓最远，是统计以它为中线，左右30°范围求和
    for(int i=0;i<laserSize;i++)
    {
        int tot=0; //想统计i这条激光左右各自15度的求和
        for(int j=max(0,i-30);j<min(laserSize,i+30);j++)
        {
            if(laserData[j]>MMM) tot++;
        }
        if(tot>totMax)
        {
            maxi=i; totMax=tot;  
        }
    }
    double far_angle=maxi*0.5/0.985-9; //这个数字？
    double angle_err=90-far_angle;
    steer=angle_err/90.0*400;  // 这一行基本没用
    double P1,P2; //用于计算speed的系数,避障需要减速
    P1=10.0/90;
    P2=10.0/laserData[maxi];
    speed=160-P1*angle_err+P2*laserData[180];  //为什么这么设计
    if(speed<140) speed=140;
    if(speed>180) speed=180; //防止溢出

    //想看车子前方左右各自15°，front_dis小于安全距离就倒车
    int front_dis=1e18;  
    for(int i=-30;i<=30;i++)  
        front_dis=min(laserData[180+i],front_dis);
    static int steer_back=-1; //倒车steer_back
    if(front_dis<vars->safeDistance)
    {
        if(!vars->reverse){
            steer_back=angel_err>0? -400:400; //方向盘打死
            vars->reverse=1;
        }
    }
    if(front_dis>2*vars->safeDistance)  //倒车一段时间后，前方探测到的最短距离大于阈值，改为正向
    {
        vars->reverse=0;
    }
    if(vars->reverse)
    {
        vars->State=1;
        speed=-speed; steer=steer_back;
        return {speed,steer};
    }
    
    //考察是否拐弯：看差错角在不在40°以内
    bool is_turn=(abs(angle_err)>40);
    if(is_turn) 
    {
        vars->State=1;
        speed=angle_err>0? 400:-400;
        return {speed,steer};
    }
  
//    以下是State=0,走中线的PID控制算法
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

   steer = vars->pid.step(err);
   speed = 150; // turning should be a bit slower

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
   return {speed,steer};
//   int filt = vars->filterRange * 2;
/*
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
   */
}

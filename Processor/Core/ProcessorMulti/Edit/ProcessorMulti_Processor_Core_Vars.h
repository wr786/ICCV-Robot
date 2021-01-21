//You need to modify this file.

#ifndef PROCESSORMULTI_PROCESSOR_CORE_VARS_H
#define PROCESSORMULTI_PROCESSOR_CORE_VARS_H

#include<RobotSDK_Global.h>

/*! \defgroup ProcessorMulti_Processor_Core_Vars ProcessorMulti_Processor_Core_Vars
	\ingroup ProcessorMulti_Processor_Core
	\brief ProcessorMulti_Processor_Core_Vars defines the Vars in ProcessorMulti_Processor_Core.
*/

/*! \addtogroup ProcessorMulti_Processor_Core_Vars
	@{
*/

/*! \file ProcessorMulti_Processor_Core_Vars.h
	 Defines the Vars of ProcessorMulti_Processor_Core
*/

//*******************Please add other headers below*******************


#include "ProcessorMulti_Processor_Core_ParamsData.h"
#include "PIDControler.h"
#include "OGM.h"
#include "poscalc.h"

enum PID_State {
    DEFAULT = 0, // State=0: 走过道中线（default）
    BYPASS,      // State=1 特殊情况，不能找中线，要倒车绕障;
    ADJUST       // State=2,第一次调整路线，要扭头
};

enum Direction {
    CLOCKWISE = 0,
    ANTI_CLOCKWISE
};

//The Vars is defined as below
/*! \class ProcessorMulti_Processor_Core_Vars 
	\brief The Vars of ProcessorMulti_Processor_Core.
	\details **Please add details below**

*/
class ROBOTSDK_OUTPUT ProcessorMulti_Processor_Core_Vars 
{
public:
	/*! \fn ProcessorMulti_Processor_Core_Vars()
		\brief The constructor of ProcessorMulti_Processor_Core_Vars. [required]
		\details ****Please add details below****

	*/
	ProcessorMulti_Processor_Core_Vars() 
	{
		if(mapWidth > 0 && mapHeight > 0) {
			map = new double* [mapHeight];
			for(int i=0; i<mapHeight; i++) {
				map[i] = new double[mapWidth];
			}
		} else {
            map = NULL;
		}
	}
	/*! \fn ~ProcessorMulti_Processor_Core_Vars()
		\brief The destructor of ProcessorMulti_Processor_Core_Vars. [required]
		\details *****Please add details below*****

	*/
	~ProcessorMulti_Processor_Core_Vars() 
	{
		if(map != NULL) {
			for(int i=0; i<mapHeight; i++) {
				delete map[i];
			}
			delete [] map;
			map = NULL;
		}
	}
public:
	//*******************Please add variables below*******************
	
	/* PID相关 */
	//整体设计：首先建立地图存储知识
	//预计，输入始末房间号，只需要判断顺时针或逆时针
	//但是，小车的走路，认为它一开始在楼道中无法判断自己面朝什么方向
	// 它获得的只有激光数据，预计它无法基于此来判断自己在超前还是朝后
	// 所以，我们先让小车按照自己检测的路径按照可行的办法走
	// 显然，上电之后，小车首先会让自己走中线
	// 于是，PID控制中，我们现在的修改是，先摸索走路，可以右转就右转，可以左转就左转
	// 但是要捕捉第一个路口的方向，比照预存的知识
	//此时判断究竟是接着走下去还是扭头  (如果是对的，只需要对照地图,预计每一次制造地图都是一样的？) 
    PIDControler pid;
	bool mark_1=0; //第一个路口只需要记录一次
	//mark_1相当于锁， mark_1=1就是已经记录了第一个路口的方向
	bool is_right=0, is_arrive=0, first_turning=0; 
	//is_right=1之后就不需要管路径了
	int small_sum=50,time_for_chg=10; //右侧30度的合计，未替换0，这是下限阈值
	// time_for_chg这么长的时间让它扭头
	// is_right_dir=0表示是按照正确路径在走
	// is_arrive=0 表示还没到
	// first_turning=0 表示遇到的第一个路口是向右，1表示向左

    PID_State State;
    Direction dir;  // dir=0: 顺时针(defaul)；dir=1:逆时针

    // 需要看一下现场的房间号设计知识
// 给定起点、终点后，由于地形相当简单，我们只需要确认应该顺时针还是逆时针，这部分需要现场看
    #define room_num 10
    int st,ed;
    bool dir_know[room_num][room_num];  //dir_know[i][j]=0表示从i到j应该顺时针走
    bool need_chg[room_num][room_num]; //need_chg[i][j]=1表示如果i,j没有按照规划路线,就让他按照现有路走了
	// 
    bool reverse = false;
    double prev_odom;
    int baseSteer = 40;
    int filterRange = 5;
    short k_p = 20;
    short k_i = 0;
    short k_d = 13;
    int straightThreshold = 30;
    int straightSpeed = 150;
    int infDistance = 17867;
    int backwardDistance = 50;
    int safeAngle = 45;
    int safeDistance = 50;
    int forwardDetectRange = 5;
	
	int turningTimestamp = 0;

	/* 地图相关 */
	int mapWidth = 640;
	int mapHeight = 600;
	double** map;

	double aL = 4.2;
	double xL = 0.28;
	double yL = 2.6;
	double mapRes = 0.5;
	int ZeroX = -360;
	int ZeroY = -500;
	double logodd_occu = 1;
	double logodd_free = -0.7;

    int T = 120;
    int TMAX = 120;

	/* poscalc */
	bool positionsInited = false;

    double lastBotX = 0;
    double lastBotY = 0;
	double relOdom = 0;
	const double deltaOdom = 0.5;
};

/*! @}*/ 

#endif

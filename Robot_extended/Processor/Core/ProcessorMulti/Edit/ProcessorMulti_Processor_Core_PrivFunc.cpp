//You need to program this file.

#include "../NoEdit/ProcessorMulti_Processor_Core_PrivFunc.h"
#include <cmath>
using namespace std;

//*******************Please add static libraries in .pro file*******************
//e.g. unix:LIBS += ... or win32:LIBS += ...

bool DECOFUNC(setParamsVarsOpenNode)(QString qstrConfigName, QString qstrNodeType, QString qstrNodeClass, QString qstrNodeName, void * paramsPtr, void * varsPtr)
{
	XMLDomInterface xmlloader(qstrConfigName,qstrNodeType,qstrNodeClass,qstrNodeName);
	ProcessorMulti_Processor_Core_Params * params=(ProcessorMulti_Processor_Core_Params *)paramsPtr;
	ProcessorMulti_Processor_Core_Vars * vars=(ProcessorMulti_Processor_Core_Vars *)varsPtr;
	/*======Please Program below======*/
	/*
	Function: open node.
	Procedure:
	1: load parameters (params). [GetParamValue(xmlloader,params,tag); GetEnumParamValue(xmlloader,params,tag); GetUEnumParamValue(xmlloader,params,tag)]
	2: initialize variables (vars).
	3: If everything is OK, return 1 for successful opening and vice versa.
	*/
	
	return 1;
}

bool DECOFUNC(handleVarsCloseNode)(void * paramsPtr, void * varsPtr)
{
	ProcessorMulti_Processor_Core_Params * params=(ProcessorMulti_Processor_Core_Params *)paramsPtr;
	ProcessorMulti_Processor_Core_Vars * vars=(ProcessorMulti_Processor_Core_Vars *)varsPtr;
	/*======Please Program below======*/
	/*
	Function: close node.
	Procedure:
	1: handle/close variables (vars).
	2: If everything is OK, return 1 for successful closing and vice versa.
	*/
	
	return 1;
}

void DECOFUNC(getInternalTrigger)(void * paramsPtr, void * varsPtr, QObject * & internalTrigger, QString & internalTriggerSignal)
{
	ProcessorMulti_Processor_Core_Params * params=(ProcessorMulti_Processor_Core_Params *)paramsPtr;
	ProcessorMulti_Processor_Core_Vars * vars=(ProcessorMulti_Processor_Core_Vars *)varsPtr;
	internalTrigger=NULL;
	internalTriggerSignal=QString();
	/*======Occasionally Program above======*/
	/*
	Function: get internal trigger [defined in vars] for node.
	You need to program here when you need internal trigger (internalTrigger + internalTriggerSignal) for node.
	E.g.
	internalTrigger=&(vars->trigger);
	internalTriggerSignal=QString(SIGNAL(triggerSignal()));
	*/
}

void DECOFUNC(initializeOutputData)(void * paramsPtr, void * varsPtr, boost::shared_ptr<void> & outputDataPtr)
{
	ProcessorMulti_Processor_Core_Params * params=(ProcessorMulti_Processor_Core_Params *)paramsPtr;
	ProcessorMulti_Processor_Core_Vars * vars=(ProcessorMulti_Processor_Core_Vars *)varsPtr;
	outputDataPtr=boost::shared_ptr<void>(new SourceDrainMono_Sensor_EncoderIMU_Order_InputData());
	/*======Occasionally Program below/above======*/
	/*
	Function: initial output data.
	You need to program here when you need to manually initialize output data.
	*/
	
}

void DECOFUNC(getMultiInputDataSize)(void * paramsPtr, void * varsPtr, QList<int> & inputDataSize)
{
	ProcessorMulti_Processor_Core_Params * params=(ProcessorMulti_Processor_Core_Params *)paramsPtr;
	ProcessorMulti_Processor_Core_Vars * vars=(ProcessorMulti_Processor_Core_Vars *)varsPtr;
	inputDataSize=QList<int>();
	/*======Please Program above======*/
	/*
	Function: get input data size to be grabbed from buffer.
	Rules:
	inputDataSize=0: grab and remove all data from buffer.
	inputDataSize>0: grab inputDataSize latest data from buffer.
	inputDataSize<0: grab and remove inputDataSize ancient data from buffer.
	E.g.
	inputDataSize=QList<int>()<<0<<1<<-1...;
	*/
}


//Input Port #0: Buffer_Size = 10, Params_Type = SourceDrainMono_Sensor_EncoderIMU_Params, Data_Type = SourceDrainMono_Sensor_EncoderIMU_Data
//Input Port #1: Buffer_Size = 10, Params_Type = SensorTimer_Sensor_URG_Params, Data_Type = SensorTimer_Sensor_URG_Data
//Input Port #2: Buffer_Size = 10, Params_Type = SensorTimer_Sensor_xtion_Params, Data_Type = SensorTimer_Sensor_xtion_Data
bool DECOFUNC(processMultiInputData)(void * paramsPtr, void * varsPtr, QVector<QVector<void *> > inputParams, QVector<QVector<void *> > inputData, void * outputData, QList<int> & outputPortIndex)
{
	ProcessorMulti_Processor_Core_Params * params=(ProcessorMulti_Processor_Core_Params *)paramsPtr;
	ProcessorMulti_Processor_Core_Vars * vars=(ProcessorMulti_Processor_Core_Vars *)varsPtr;
	QVector<SourceDrainMono_Sensor_EncoderIMU_Params *> inputparams_0; copyQVector(inputparams_0,inputParams[0]);
	QVector<SensorTimer_Sensor_URG_Params *> inputparams_1; copyQVector(inputparams_1,inputParams[1]);
	QVector<SensorTimer_Sensor_xtion_Params *> inputparams_2; copyQVector(inputparams_2,inputParams[2]);
	QVector<SourceDrainMono_Sensor_EncoderIMU_Data *> inputdata_0; copyQVector(inputdata_0,inputData[0]);
	QVector<SensorTimer_Sensor_URG_Data *> inputdata_1; copyQVector(inputdata_1,inputData[1]);
	QVector<SensorTimer_Sensor_xtion_Data *> inputdata_2; copyQVector(inputdata_2,inputData[2]);
	SourceDrainMono_Sensor_EncoderIMU_Order_InputData * outputdata=(SourceDrainMono_Sensor_EncoderIMU_Order_InputData *)outputData;
	outputPortIndex=QList<int>();
	if(inputdata_0.size()==0){return 0;}
	if(inputdata_1.size()==0){return 0;}
	if(inputdata_2.size()==0){return 0;}
	/*======Please Program below======*/
	/*
	Step 1: process inputdata_index, then store it into outputdata.
	Step 2 [optional]: determine the outputPortIndex. (if not, outputdata will be sent by all ports)
	E.g. outputPortIndex=QList<int>()<<(outportindex1)<<(outportindex2)...
	*/

    //inputdata_0                                             // EncoderIMU
    //inputdata_1                                             // URG
    //inputdata_2                                             // Xtion (RGB && depth)
    //cv::imshow("color", inputdata_2.front()->cvColorImg);   // Show RGB image
    //cv::imshow("depth", inputdata_2.front()->cvDepthImg);   // Show depth image

    short steer = 0;           // [-400, 400]
    short speed = 100;           // [-180, 180]

	/* 小车行驶的距离 */
    double dis = inputdata_0.front()->odometry;
	/* IMU航向角 */
    double yaw = inputdata_0.front()->orientation * 4 * M_PI / (2 * M_PI + 4.00555081); 

    int laserSize = inputdata_1.front()->datasize;
    short* laserData = inputdata_1.front()->data;
    double laserUnit = inputparams_1.front()->unit;

	// OGM disabled
	// calc_map(inputdata_0, inputdata_1, laserUnit, vars);

    if(!laserData || !params || !vars) {    // errorData
        return 0;
    }

    double botx = inputdata_0.front()->x - vars->lastBotX;
    double boty = inputdata_0.front()->y - vars->lastBotY;

	if(posesign_stop()) {
		vars->ispause = true;
	}

	if(vars->ispause) {
		if(posesign_start()) {
			vars->ispause = false;
			vars->ifpaused = true;
		}
	}

	// 初次使用，初始化一下各个点的坐标
	if(!vars->positionsInited) {
		init_positions();
		vars->positionsInited = true;
	}

	// 判断是否需要调头
    if(adjust_lazytag(botx, boty)) {
		vars->State = ADJUST;
	}

	// 到达终点后，继续前进一段距离，以期减小误差
	if(vars->relOdom > 0) {
		// 前进的距离达到预期，那么停下来并清除目标
		if(abs(vars->relOdom - inputdata_0.front()->odometry) >= deltaOdom) {
			vars->relOdom = 0;
			clear_target();	// 清除目标
			qDebug() << inputdata_0.front()->x << ", " << inputdata_0.front()->y << endl;
		
			/* 如果中途暂停过，说明已经提前拿走了资料
			   那么不应该再在现在这个点停留，而是直接去下一个点 */
			if(vars->ifpaused) {
				next_target();
				// 重新计算x和y
				vars->lastBotX = inputdata_0.front()->x;
				vars->lastBotY = inputdata_0.front()->y;
			}

			vars->ifpaused = false;
		}
	}

	// 如果这时候到达了终点
    if(reach_target(botx, boty) && vars->relOdom == 0) {
		vars->relOdom = inputdata_0.front()->odometry;
	}

	// qDebug() << "(" << tarx << ", " << tary << ")\n";
	if(vars->ispause) {
		speed = 0;
		steer = 0;
	} else if(tarx == 0 && tary == 0) {
		// 没有目标，应该正在待机
		speed = 0;
		steer = 0;
		if(posesign_start()) {
			// 暂停一段时间，再去下一个地点
			next_target();
			// 重新计算x和y
            vars->lastBotX = inputdata_0.front()->x;
            vars->lastBotY = inputdata_0.front()->y;
//			inputdata_0.front()->x = inputdata_0.front()->y = 0;
		}
	} else {
		// 正在行进，使用PID
		std::pair<short, short> ret = calc_steer(dis, yaw, laserSize, laserData, laserUnit, params, vars);
    	speed = -ret.first;
    	steer = ret.second;
        qDebug() << speed << ' ' << steer << endl;
	}

    //=================added=================

    // Show RGB image && compass
    double ori = - ((double)steer / 400.0) * (M_PI / 2.0);
    cv::Mat img;
    inputdata_2.front()->cvColorImg.copyTo(img);
	cv::flip(img, img, 1);
    cv::Point compass = cv::Point(100, 100);
    cv::circle(img, compass, 80, cv::Scalar(0,255,0), 1, CV_AA);
    cv::line(img, compass,
             cv::Point(compass.x - sin(ori) * 80,
                       compass.y - cos(ori) * 80),
             cv::Scalar(0,255,0), 3, CV_AA);
//    cv::imshow("color", img);

    //--------------------------------------------
    int maxSpeed = 180;
    if (speed > maxSpeed) speed = maxSpeed;
    if (speed < -maxSpeed) speed = -maxSpeed;
    char dataput[20];
    dataput[0] = 0xF8;
    dataput[1] = 4;
    *(short*)&dataput[2] = (short)steer;
    *(short*)&dataput[4] = (short)speed;
    dataput[6] = 0x8F;
    outputdata->datagram = QByteArray(dataput, 7);
	return 1;
}


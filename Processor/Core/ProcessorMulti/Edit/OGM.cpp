#include "../NoEdit/ProcessorMulti_Processor_Core_PrivFunc.h"
#include "OGM.h"

void Bresenham(int x1, int y1, int x2, int y2, std::vector<Location>& locationVec)
{
        bool swapflag = false;
        if (x1 > x2){
                int tmpx = x1;
                int tmpy = y1;
                x1 = x2;
                y1 = y2;
                x2 = tmpx;
                y2 = tmpy;
                swapflag = true;
        }

        int dx = x2-x1;
        int dy = y2-y1;
        int x = x1;
        int y = y1;
        int sub = (dy<<1)-dx;
        locationVec.push_back(Location(x, y));
        while(x<x2){
                ++x;
                if (sub > 0){
                        sub += (dy<<1) - (dx<<1);
                        ++y;
                }else {
                        sub += (dy<<1);
                }
                locationVec.push_back(Location(x, y));
        }

        if (swapflag){
                unsigned int size = locationVec.size();
                for (unsigned int i = 0; i < size/2 ; ++i){
                        Location tmp = locationVec[i];
                        locationVec[i] = locationVec[size-i-1];
                        locationVec[size-i-1] = tmp;
                }
        }
}

void CalcShortestDistance(const Location& startPos, const Location& endPos, std::vector<Location>& locationVec)
{
    if (startPos.x==endPos.x && startPos.y==endPos.y)
        return ;

    if (endPos.x == startPos.x){ //x相同
        if (endPos.y > startPos.y){
            for (unsigned int i = 0; i < (unsigned int)(endPos.y-startPos.y); ++i){
                locationVec.push_back(Location(startPos.x, startPos.y+i+1));
            }
        }else{
            for (unsigned int i = 0; i < (unsigned int)(startPos.y-endPos.y); ++i){
                locationVec.push_back(Location(startPos.x, startPos.y-i-1));
            }
        }
        return ;
    }

    float k = (float)(endPos.y-startPos.y)/(endPos.x-startPos.x);

    if (k >= 0 && k <= 1){ //斜率为0~1

        Bresenham(startPos.x,startPos.y,endPos.x,endPos.y,locationVec);

    }else if (k > 1){ //斜率为1~无穷大

        Bresenham(startPos.y,startPos.x,endPos.y,endPos.x,locationVec);
        for (std::vector<Location>::iterator it = locationVec.begin(); it!=locationVec.end(); ++it){
            int tmp = (*it).x;
            (*it).x = (*it).y;
            (*it).y = tmp;
        }

    }else if (k >= -1 && k < 0){ //斜率为-1~0

        Bresenham(startPos.x,-startPos.y,endPos.x,-endPos.y,locationVec);
        for (std::vector<Location>::iterator it = locationVec.begin(); it!=locationVec.end(); ++it)
            (*it).y = -(*it).y;

    }else if (k < -1){ //斜率为无穷小~-1

        Bresenham(-startPos.y,startPos.x,-endPos.y,endPos.x,locationVec);
        for (std::vector<Location>::iterator it = locationVec.begin(); it!=locationVec.end(); ++it){
            int tmp = (*it).x;
            (*it).x = (*it).y;
            (*it).y = tmp;
            (*it).y = -(*it).y;
        }
    }

    locationVec.erase(locationVec.begin());
}

/* inputdata_0是位置，inputdata_1是激光 */
void calc_map(QVector<SourceDrainMono_Sensor_EncoderIMU_Data *> inputdata_0, QVector<SensorTimer_Sensor_URG_Data *> inputdata_1, double laserUnit, ProcessorMulti_Processor_Core_Vars * vars) {
    if(inputdata_0.size()==0){return 0;}
    if(inputdata_1.size()==0){return 0;}

    for(int i = 0 ; i < inputdata_1.front()->datasize; i++) {
		
		//目标：对每条激光束更新全局地图（Occupency Gird Map）
		//下列步骤中需要编程的有3，4，5，8
		
		//1.分别定义激光点在全局坐标系（GPS），机器人坐标系，激光雷达坐标系中的坐标变量
			double gx, gy;//激光点在全局坐标系中的位置 单位m
			double rx, ry;//激光点在机器人坐标系中的位置 单位m
			double lx, ly;//激光点在激光雷达坐标系中的位置 单位m
		
		//2.若激光点返回测距值为0，则为无效数据，将其滤除。
			if(inputdata_1.front()->data[i] == 0)
				continue;
		
        //*3.计算激光点在激光雷达坐标系下的位置，并根据参数inputparams_0.front()->isReverse判断是否将lx取相反数。
			double dis = inputdata_1.front()->data[i] / laserUnit;//计算得到单个激光点的距离返回值
            double angle = i * 180 / inputdata_1.front()->datasize * 3.1415926 / 180.0; //计算得到当前处理激光束在激光雷达坐标系中角度
            lx = -dis * cos(angle); // reverse it
            ly = dis * sin(angle);
            if(inputparams_0.front()->isReverse) {
                lx = -lx;
            }
//            qDebug() << dis << " ";
			
        //*4.进行 激光雷达坐标系->机器人坐标系 变换
			//xL,yL,aL定义见课件，其在程序中对应变量为
			//inputparams_0.front()->xL，inputparams_0.front()->yL，inputparams_0.front()->aL
            double laser_aL_rad=vars->aL* 3.1415926 / 180.0;//转换为弧度
            rx = vars->xL + (-cos(laser_aL_rad)*lx + sin(laser_aL_rad)*ly);
            ry = var->yL + (cos(laser_aL_rad)*ly + sin(laser_aL_rad)*lx);
//            qDebug() << inputparams_0.front()->xL << " " << inputparams_0.front()->yL<< " ";

        //*5.进行 机器人坐标系->全局坐标系 变换
			//机器人航向角ori定义见课件，对应变量为inputdata_1.front()->ori
            double orien=inputdata_0.front()->orientation;
            gx = inputdata_0.front()->x + (-sin(orien)*rx + cos(orien)*ry);
            gy = inputdata_0.front()->y + (sin(orien)*ry + cos(orien)*rx);
//            qDebug() << inputdata_1.front()->x << " " << inputdata_1.front()->y << "\n";

		//6.计算激光点在地图中的位置 单位 pixel
			int mapx, mapy;
            mapx = (gx) / vars->mapRes-vars->ZeroX;
			mapy = (gy) / vars->mapRes-vars->ZeroY;
		
		//7.调用函数计算当前激光束途经的栅格点坐标序列（地图中的像素坐标），存储在std::vector<Location> locationVec中，各栅格点坐标按照激光发射方向排列。
			int location_mapx,location_mapy;
			location_mapx=inputdata_0.front()->x/vars->mapRes-vars->ZeroX;
			location_mapy=inputdata_0.front()->y/vars->mapRes-vars->ZeroY;
			std::vector<Location> locationVec;
			Location startPos(location_mapx,location_mapy);
			Location endPos(mapx,mapy);
			CalcShortestDistance(startPos, endPos, locationVec);

        //*8.根据OGM地图生成方法制图。
			//地图存储在vars->map[][]中，坐标轴为vars->map[y][x]。
			//定义每个栅格vars->map[y][x]取值上下界。
            double upthres=300;
            double lowthres=-300;
            //逐个更新当前激光束途经的栅格点坐标，params->logodd_free，params->logodd_occu为检测到当前坐标为无障碍/有障碍的更新值。
            //logodd_free = -0.7, logodd_occu = 1
			for (std::vector<Location>::iterator c=locationVec.begin();c!=locationVec.end();c++){
				if((*c).x >= 0 && (*c).x < params->mapWidth && (*c).y >= 0 && (*c).y < params->mapHeight){
                    if(c!=locationVec.end()-1) {
                        if(vars->map[(*c).y][(*c).x] + vars->logodd_free >= lowthres
                        && vars->map[(*c).y][(*c).x] + vars->logodd_free <= upthres)
                            vars->map[(*c).y][(*c).x] += vars->logodd_free;
                    } else {
                        if(vars->map[(*c).y][(*c).x] + vars->logodd_occu <= upthres
                        && vars->map[(*c).y][(*c).x] + vars->logodd_occu >= lowthres)
                            vars->map[(*c).y][(*c).x] += vars->logodd_occu;
                    }
				}
			}
}
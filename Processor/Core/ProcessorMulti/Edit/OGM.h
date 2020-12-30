#ifndef OGM_H

#include <iostream>
#include <vector>

struct Location {
    int x;
    int y;
    Location() { x=0, y=0; }
    Location(int a, int b) { x=a, y=b; }
};
void Bresenham(int x1, int y1, int x2, int y2, std::vector<Location>& locationVec);
void CalcShortestDistance(const Location& startPos, const Location& endPos, std::vector<Location>& locationVec);
void calc_map(QVector<SourceDrainMono_Sensor_EncoderIMU_Data *> inputdata_0, QVector<SensorTimer_Sensor_URG_Data *> inputdata_1, double laserUnit, ProcessorMulti_Processor_Core_Vars * vars);
// 需要看一下现场的房间号设计知识
// 给定起点、终点后，由于地形相当简单，我们只需要确认应该顺时针还是逆时针，这部分需要现场看
int room_num;
bool dir_know[room_num][room_num];  //dir_know[i][j]=0表示从i到j应该顺时针走
bool need_chg[room_num][room_num]; //need_chg[i][j]=1表示如果i,j没有按照规划路线,就让他按照现有路走了
#define OGM_H

#endif

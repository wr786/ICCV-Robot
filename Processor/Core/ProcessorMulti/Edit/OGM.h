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
//记录知识，什么任务应该顺时针还是逆时针
//需要看一下现场的房间号设计知识
int room_num;
bool dir_know[room_num][room_num];
#define OGM_H

#endif

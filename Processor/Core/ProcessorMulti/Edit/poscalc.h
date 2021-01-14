#ifndef POSCALC_H

class Position {
public:
    int xpos;       // 位置坐标
    int ypos;
    int posid;      // 位置编号

    Position(int x, int y, int pid): xpos(x), ypos(y), pid(posid) {}
};

// 用于存放各个地点，一般来说即门牌号
#define POSNUM 10
Position position[POSNUM];
int tarx, tary; // 目标x、目标y，即x[j]-x[i], y[j]-y[i]
#define rlimit 10

inline int get_delta2(int a, int b);

void init_positions();
Position get_position(int address);
void set_target(int addr1, int addr2);
void clear_target();
bool reach_target(int botx, int boty);

#define POSCALC_H

#endif

#ifndef POSCALC_H

// 存放路径，里面写的是门牌号
// ROUTELEN为路径的长度，一起修改
#define ROUTELEN 20
//! 在此处填写路径
int route[ROUTELEN] = {};
// 现在运动到了route中的哪个idx
int currid = 0;

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

static int countdown;

inline int get_delta2(int a, int b);
bool poscalc_countdown();

void init_positions();
Position get_position(int address);
void set_target(int addr1, int addr2);
void clear_target();
bool reach_target(int botx, int boty);
void next_target();

#define POSCALC_H

#endif

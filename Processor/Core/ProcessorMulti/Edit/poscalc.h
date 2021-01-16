#ifndef POSCALC_H

#define ROUTELEN 20
// 存放路径，里面写的是门牌号
// ROUTELEN为路径的长度，一起修改
#define ROUTELEN 20
//! write the route here
static int route[ROUTELEN] = {};
// 现在运动到了route中的哪个idx
static int currid = 0;

class Position {
public:
    double xpos;       // 位置坐标
    double ypos;
    int posid;      // 位置编号

    Position(double x=-1, double y=-1, int pid=-1): xpos(x), ypos(y), posid(pid) {}
};

// 用于存放各个地点，一般来说即门牌号
#define POSNUM 15
static Position position[POSNUM];
static int tarx, tary; // 目标x、目标y，即x[j]-x[i], y[j]-y[i]
#define rlimit 8

static int countdown;

inline double get_delta2(double a, double b);
bool poscalc_countdown();

void init_positions();
Position get_position(int address);
void set_target(int addr1, int addr2);
void clear_target();
bool reach_target(double botx, double boty);
void next_target();

#define POSCALC_H

#endif

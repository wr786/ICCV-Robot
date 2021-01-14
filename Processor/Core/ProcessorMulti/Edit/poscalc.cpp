#include "ProcessorMulti_Processor_Core_Vars.h"
#include "poscalc.h"

/* get_delta2 - get (a-b)^2 */
inline int get_delta2(int a, int b) {
    int c = a-b;
    return c*c;
}

/* init_positions - init the positions */
void init_positions() {
    // position[i] = Position(x[i], y[i], i);
    //todo 类似上面地建立标号与坐标的关系
}

/* get_position - get the Position object of #address */
Position get_position(int address) {
    //todo 添加门牌号与编号的对应
    switch(address) {

        default:
            return Position(-1, -1, -1);
    }
}

/* set_target - set target according to the doorplates */
void set_target(int addr1, int addr2) {
    Position tar1 = get_position(addr1);
    Position tar2 = get_position(addr2);
    tarx = tar2.xpos - tar1.xpos;
    tary = tar2.ypos - tar1.ypos;
}

/* clear_target - clear the tarx and tary */
void clear_target() {
    tarx = tary = 0;
}

/* reach_target - judge if the bot reaches the target */
bool reach_target(int botx, int boty) {
    if(get_delta2(botx, tarx) + get_delta2(boty, tary) <= rlimit) {
        return true;
    } else {
        return false;
    }
}
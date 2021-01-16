#include "poscalc.h"

/* get_delta2 - get (a-b)^2 */
inline double get_delta2(double a, double b) {
    double c = a-b;
    return c*c;
}

/* poscalc_countdown - count down in poscalc */
bool poscalc_countdown() {
    if(countdown == 0) {
        countdown = 40;
    } else {
        countdown -= 1;
    }
    return countdown == 0;
}

/* init_positions - init the positions */
void init_positions() {
    // position[i] = Position(x[i], y[i], i);
    position[1] = Position(5.17197, -0.0997239, 1);
    position[2] = Position(11.1404, 0.155884, 2);
    position[3] = Position(25.9149, 0.257952, 3);
    position[4] = Position(34.1626, 11.0058, 4);
    position[5] = Position(34.7748, 14.7281, 5);
    position[6] = Position(34.9851, 16.0653, 6);
    position[7] = Position(36.2898, 23.712, 7);
    position[8] = Position(37.7327, 31.1132, 8);
    position[9] = Position(26.9643, 39.2364, 9);
    position[10] = Position(-3.91619, 40.2907, 10);
    position[11] = Position(-5.91603, 35.1465, 11);
    position[12] = Position(-6.58897, 33.4505, 12);
    position[13] = Position(-12.8488, 19.7759, 13);
    position[14] = Position(-14.6157, 14.8986, 14);
}

/* get_position - get the Position object of #address */
Position get_position(int address) {
    switch(address) {
        case 2224: return position[1];
        case 2225: return position[2];
        case 2229: return position[3];
        case 2245: return position[4];
        case 2246: return position[5];
        case 2247: return position[6];
        case 2252: return position[7];
        case 2255:
        case 2256: return position[8];
        case 2260: return position[9];
        case 2208:
        case 2209: return position[10];
        case 2210:
        case 2211: return position[11];
        case 2212: return position[12];
        case 2217:
        case 2218: return position[13];
        case 2219:
        case 2220:
        case 2221:
        case 2222: return position[14];
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
bool reach_target(double botx, double boty) {
    if(tarx == 0 && tary == 0) {
        // 没有目标，那必然不能算作达到目标
        return false;
    }
    if(get_delta2(botx, tarx) + get_delta2(boty, tary) <= rlimit) {
        return true;
    } else {
        return false;
    }
}

/* next_target - set target according to the route */
void next_target() {
    if(currid == ROUTELEN) return;
    set_target(route[currid], route[currid + 1]);
    currid++;
}

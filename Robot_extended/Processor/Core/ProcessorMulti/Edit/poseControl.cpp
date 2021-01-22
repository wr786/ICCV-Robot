#include "poseControl.h"
#include "ProcessorMulti_Processor_Core_Vars.h"
#include <opencv2/opencv.hpp>
// #include <openni2/OpenNI.h>
#include <NiTE.h>

#define NODE(i) inputdata_2.jointPos2D[i]
/*
    点从0开始
    左手 3 5 7
    右手 2 4 6
*/


/* posesign_start - 手抬到最高，则 */
bool posesign_start(SensorTimer_Sensor_xtion_Data * inputdata_2) {
    bool leftHandUp = false;
    bool rightHandUp = false;

    cv::Point leftRoot = NODE(3);
    cv::Point leftMid = NODE(5);
    cv::Point leftEnd = NODE(7);

    cv::Point rightRoot = NODE(2);
    cv::Point rightMid = NODE(4);
    cv::Point rightEnd = NODE(6);

    /* 判断左手是否抬高 */
    if(is_vertical(leftRoot, leftMid) && is_vertical(leftMid, leftEnd)) {
        if(is_higher_than(leftEnd, leftMid) && is_higher_than(leftMid, leftRoot)) {
            leftHandUp = true;
        }
    }
    
    /* 判断右手是否抬高 */
    if(is_vertical(rightRoot, rightMid) && is_vertical(rightMid, rightEnd)) {
        if(is_higher_than(rightEnd, rightMid) && is_higher_than(rightMid, rightRoot)) {
            rightHandUp = true;
        }
    }

    return leftHandUp || rightHandUp;
}

/* posesign_stop - 两手平抬，则 */
bool posesign_stop(SensorTimer_Sensor_xtion_Data * inputdata_2) {
    bool leftHandH = false;
    bool rightHandH = false;

    cv::Point leftRoot = NODE(3);
    cv::Point leftMid = NODE(5);
    cv::Point leftEnd = NODE(7);

    cv::Point rightRoot = NODE(2);
    cv::Point rightMid = NODE(4);
    cv::Point rightEnd = NODE(6);

    /* 判断是否左手平抬 */
    if(is_horizon(leftRoot, leftMid) && is_horizon(leftMid, leftEnd)) {
        leftHandH = true;
    }

    /* 判断是否右手平抬 */
    if(is_horizon(rightRoot, rightMid) && is_horizon(rightMid, rightEnd)) {
        rightHandH = true;
    }

    return leftHandH && rightHandH;
}

/* helper functions */

bool is_higher_than(cv::Point p1, cv::Point p2) {
    return p1.y < p2.y;
}

bool is_left(cv::Point p1, cv::Point p2) {
    return p1.x < p2.x;
}

bool is_right(cv::Point p1, cv::Point p2) {
    return p1.x > p2.x;
}

bool is_vertical(cv::Point p1, cv::Point p2) {
    return (p1.x - p2.x) < biasLimit;
}

bool is_horizon(cv::Point p1, cv::Point p2) {
    return (p1.y - p2.y) < biasLimit;
}
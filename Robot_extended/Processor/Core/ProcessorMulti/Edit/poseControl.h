#ifndef POSECONTROL_H
#include "ProcessorMulti_Processor_Core_Vars.h"
#include <opencv2/opencv.hpp>
// #include <openni2/OpenNI.h>
#include <NiTE.h>

#define biasLimit 5

/* HELPER FUNCTIONS */
bool is_higher_than(cv::Point p1, cv::Point p2);
bool is_left(cv::Point p1, cv::Point p2);
bool is_right(cv::Point p1, cv::Point p2);
bool is_vertical(cv::Point p1, cv::Point p2);
bool is_horizon(cv::Point p1, cv::Point p2);

/* posesign functions */
bool posesign_start(SensorTimer_Sensor_xtion_Data * inputdata_2);
bool posesign_stop(SensorTimer_Sensor_xtion_Data * inputdata_2);

#define POSECONTROL_H
#endif
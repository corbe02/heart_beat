#ifndef OPTICALFLOW_H
#define OPTICALFLOW_H

#include <opencv2/opencv.hpp>

#include "TrackedMatch.h"

class OpticalFlow{
public:
    OpticalFlow();
    void computeOpticalFlow(const cv::Mat &prev, cv::Mat &current, double &movement_threshold_);
    static void OpticalFlowTriangulation(const cv::Mat &prev_l, cv::Mat &current_l,
                              const cv::Mat &prev_r, cv::Mat &current_r,
                              double &movement_threshold_,
                              std::vector<TrackedMatch> &tracked_matches);

    
private:
    bool first_time_;
    std::vector<cv::Point2f> points_prev_;
    std::vector<bool> dynamic_points_prev;

};

#endif // OPTICALFLOW_H
#ifndef OPTICALFLOW_H
#define OPTICALFLOW_H

#include <opencv2/opencv.hpp>

class OpticalFlow{
public:
    OpticalFlow();
    void computeOpticalFlow(const cv::Mat &prev, cv::Mat &current, double &movement_threshold_);
    static std::vector<uchar> OpticalFlowTriangulation(const cv::Mat &prev, cv::Mat &current, double &movement_threshold_,std::vector<bool> &dynamic_points_prev,std::vector<cv::Point2f> &points_prev,cv::Mat left_img_RGB);
    
private:
    bool first_time_;
    std::vector<cv::Point2f> points_prev_;
    std::vector<bool> dynamic_points_prev;

};

#endif // OPTICALFLOW_H
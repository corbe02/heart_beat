#ifndef TRIANGULATION_H
#define TRIANGULATION_H

#include <opencv2/opencv.hpp>

class Triangulation {
public:
    Triangulation();
    static std::vector<cv::DMatch> good_match(cv::Mat &left_des, cv::Mat &right_des);
    static std::vector<cv::Vec3d> triangulate(cv::Mat &projMatLeft, cv::Mat &projMatRight,std::vector<cv::Point2f> &undistCoords1,std::vector<cv::Point2f> &undistCoords2);
    static cv::Mat computeProjMat(const cv::Mat &camMat, const cv::Mat &rotVec, const cv::Mat &transVec);
};

#endif // TRIANGULATION_H

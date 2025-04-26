
#ifndef TRACKED_MATCH_H
#define TRACKED_MATCH_H

#include <opencv2/opencv.hpp>

struct TrackedMatch {
    int id;
    cv::DMatch match;
    bool is_active;
    cv::Vec3d position_3d;
    bool dynamic_point;

    // Add these two lines:
    cv::Point2f pt_left;
    cv::Point2f pt_right;

};



#endif // TRACKED_MATCH_H
#ifndef FEATURE_EXTRACTOR_H
#define FEATURE_EXTRACTOR_H

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

class FeatureExtractor {
public:
    FeatureExtractor();
    static cv::Mat adaptiveHistogramEqualization(const cv::Mat &img);
    void featureDetection(const cv::Mat &prev,const cv::Mat &current,double &movement_threshold_,image_transport::Publisher image_pub_);
    cv::Mat extractFeatures(const cv::Mat &img, std::vector<cv::Point2f> &keypoints2f);
private:
    bool first_time_;
    std::vector<cv::Point2f> points_prev_left_ ;
    std::vector<bool> dynamic_points_prev_left_;
    std::vector<cv::Point2f> points_prev_right_ ;
    std::vector<bool> dynamic_points_prev_right_;

};

#endif // FEATURE_EXTRACTOR_H

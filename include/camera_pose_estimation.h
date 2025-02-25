#ifndef OPTICAL_FLOW_POSE_H
#define OPTICAL_FLOW_POSE_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

#include "Feature_extractor.h"
#include "Visualizer.h"
#include "Triangulation.h"
#include "OpticalFlow.h"


class OpticalFlowPose {
public:
    OpticalFlowPose(ros::NodeHandle &nh);
    void imageCallback(const sensor_msgs::ImageConstPtr &msg);
    static void PublishRenderedImage(image_transport::Publisher pub, cv::Mat image, std::string encoding, std::string frame_id);
    static void recoverPose(const std::vector<cv::Point2f> &good_old, const std::vector<cv::Point2f> &good_new,const std::vector<bool>& dynamic, cv::Mat &current);
    double movement_threshold_;

private:

    FeatureExtractor feature_extractor_;
    Visualizer visualizer_;
    OpticalFlow optical_flow_;
    Triangulation triangulation_;

    ros::Subscriber image_sub_;
    ros::NodeHandle private_nh_;
    image_transport::ImageTransport it_;
    static image_transport::Publisher image_pub_;
    cv::Mat prev_img_;
    cv::Mat current_img_;
    std::vector<cv::Point2f> points_prev_;
    bool first_time_;
    std::vector<bool> dynamic_points_prev;  
    
};

#endif // OPTICAL_FLOW_POSE_H

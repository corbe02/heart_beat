#ifndef CAMERA_POSE_ESTIMATION_H
#define CAMERA_POSE_ESTIMATION_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int8MultiArray.h>


class OpticalFlowPose
{
public:
    OpticalFlowPose(ros::NodeHandle &nh);
    void imageCallback(const sensor_msgs::ImageConstPtr &msg);
    void PublishRenderedImage(image_transport::Publisher pub, cv::Mat image, std::string encoding, std::string frame_id);
    double movement_threshold_;
private:
    void computeOpticalFlow(const cv::Mat &prev, const cv::Mat &current);
    void recoverPose(const std::vector<cv::Point2f> &good_old, const std::vector<cv::Point2f> &good_new, const std::vector<bool> &dynamic);
    cv::Mat extractFeatures(const cv::Mat &img, std::vector<cv::Point2f> &keypoints2f);
    void drawDelaunay(const cv::Mat &current, std::vector<cv::Point2f> &good_new, const cv::Scalar& color);
    void drawVoronoi(const cv::Mat &current, std::vector<cv::Point2f> &good_new, const cv::Scalar& color);

    ros::Subscriber image_sub_;
    ros::NodeHandle private_nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;
    cv::Mat prev_img_;
    cv::Mat current_img_;
    std::vector<cv::Point2f> points_prev_;
    bool first_time_;
    std::vector<bool> dynamic_points_prev;  // Quando è posto a 1, la feature è dinamica e lo resterà sempre
};

#endif // OPTICAL_FLOW_POSE_H
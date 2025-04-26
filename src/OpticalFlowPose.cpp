#include "OpticalFlowPose.h"
#include "TrackedMatch.h"
#include "data.h"

image_transport::Publisher OpticalFlowPose::image_pub_;

OpticalFlowPose::OpticalFlowPose(ros::NodeHandle &nh) 
    : it_(nh), private_nh_(nh), first_time_(true), feature_extractor_(movement_threshold_), visualizer_(), optical_flow_(), triangulation_(), wavelet_() {
    
    private_nh_.param("threshold", movement_threshold_, 0.0);

    // Initialize the image subscriber
    image_sub_ = nh.subscribe("/video1/image_raw", 1, &OpticalFlowPose::imageCallback, this);

    // Initialize the image publisher
    image_pub_ = it_.advertise("/optical_flow/output_video", 1);  

}

void OpticalFlowPose::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;

    try {

        //Salvo l'immagine e ne creo una copia
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        current_img_ = cv_ptr->image;
        cv::Mat current_copy = current_img_.clone();

        // Dimensioni dell'immagine
        int height = current_img_.rows;
        int width = current_img_.cols;

        // Rettangolo superiore e inferiore dell'immagine
        // cv::Rect name(coord_x_alto_sx, coord_y_alto_sx, larghezza, altezza )
        cv::Rect topROI(0, 0, width, height / 2);         // Left cam
        cv::Rect bottomROI(0, height / 2, width, height / 2); // Right cam

        // EEstraggo la left e la right dall'immagine originale 
        cv::Mat left_raw = current_img_(topROI);
        cv::Mat right_raw = current_img_(bottomROI);

        // Con i coefficienti di distorsione undistorgo le immagini
        cv::Mat left_undistorted, right_undistorted;
        cv::undistort(left_raw, left_undistorted, cameraMatrixLeft, distCoeffsLeft);
        cv::undistort(right_raw, right_undistorted, cameraMatrixRight, distCoeffsRight);

        // Idem con l'immagine precedente se disponibile 
        if (!prev_img_.empty()) {
            cv::Mat prev_left_raw = prev_img_(topROI);
            cv::Mat prev_right_raw = prev_img_(bottomROI);

            cv::Mat prev_left_undistorted, prev_right_undistorted;
            cv::undistort(prev_left_raw, prev_left_undistorted, cameraMatrixLeft, distCoeffsLeft);
            cv::undistort(prev_right_raw, prev_right_undistorted, cameraMatrixRight, distCoeffsRight);

            // Riunisco le immagini correnti e precedenti e le passo alla funzione per la feature detection 
            // cv::Mat combined_undistorted;
            // cv::Mat prev_combined;
            // cv::vconcat(left_undistorted, right_undistorted, combined_undistorted);
            // cv::vconcat(prev_left_undistorted, prev_right_undistorted, prev_combined);

            // Then pass to your feature detector
            feature_extractor_.featureDetection(prev_left_undistorted,prev_right_undistorted,left_undistorted,right_undistorted, image_pub_);
        }

        prev_img_ = current_copy.clone();  // Save raw for future use

    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}


void OpticalFlowPose::PublishRenderedImage(image_transport::Publisher pub, cv::Mat image, std::string encoding, std::string frame_id) {
    //ROS_INFO("Publishing image...");
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = frame_id;
    const sensor_msgs::ImagePtr rendered_image_msg = cv_bridge::CvImage(header, encoding, image).toImageMsg();
    pub.publish(rendered_image_msg);
}

void OpticalFlowPose::recoverPose(const std::vector<cv::Point2f> &good_old, const std::vector<cv::Point2f> &good_new,const std::vector<bool>& dynamic, cv::Mat &current)
{

    // Example intrinsic matrix, update with your camera parameters
    //cv::Mat K = (cv::Mat_<float>(3, 3) << 745.0165, 0.0, 667.0261, 0.0, 745.6793, 366.4256, 0.0, 0.0, 1.0);

    // Assuming we have some camera motion model here
    //cv::Mat R, t;
    // Calculate essential matrix and recover pose
    // You might need to filter points or ensure they're in the same frame
    //cv::Mat E = cv::findEssentialMat(good_old, good_new, K, cv::RANSAC);
    //cv::recoverPose(E, good_old, good_new, K, R, t);

    
    /*
    for (const auto &point : good_new)
    {
        cv::circle(current_img_, point, 5, cv::Scalar(0, 255, 0), -1); // Green color, filled circle
    }
    */

    for (size_t i = 0; i < good_new.size(); ++i)
    {
        if (dynamic[i])
            cv::circle(current, good_new[i], 7, cv::Scalar(0, 0, 255), -1);  // Red color, filled circle
        else 
            cv::circle(current, good_new[i], 7, cv::Scalar(255, 0, 0), -1);  // Blue color, filled circle

    }


    if (current.channels() == 3 && current.depth() == CV_8U) {
        //ROS_INFO("Image is in the correct format: CV_8UC3");
    } else {
        //ROS_ERROR("Image format is incorrect. Converting...");
        cv::cvtColor(current, current, cv::COLOR_BGR2RGB);
    }
    //ROS_INFO_STREAM("Entered recover function: " << good_new.size());
    PublishRenderedImage(image_pub_, current, "bgr8", "endoscope");

    //ROS_INFO("Rotation Matrix:\n %s", R);
    //std::cout << " R " << R << std::endl;
    //std::cout << " t " << t << std::endl;
    //ROS_INFO("Translation Vector:\n %s", t);
}
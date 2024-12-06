#include "camera_pose_estimation.h"

OpticalFlowPose::OpticalFlowPose(ros::NodeHandle &nh) : it_(nh),private_nh_("~"), first_time_(true)
{
    private_nh_.param("threshold",movement_threshold_, 0.0);
        
    image_sub_ = nh.subscribe("/video1/image_raw", 1, &OpticalFlowPose::imageCallback, this);

    //ecm_status_sub_ = nh.subscribe("/arm_detection/status", 1, &OpticalFlowPose::GetECMStatus, this); // you don't need this

    prev_img_ = cv::Mat(); // // Costruisce una matrice vuota che può essere poi popolata con un'immagine/valore
    image_pub_ = it_.advertise("/optical_flow/output_video", 1);
    //ecm_status_.data.resize(4, 0);
}

void OpticalFlowPose::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    //cv::Mat mask;
    //cv::Rect rect(193, 0, 1087 - 193, 667 - 0);
    //cv::Mat maskedImage;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        current_img_ = cv_ptr->image;
        //mask = cv::Mat::zeros(current_img_.size(), CV_8UC1);
        //mask(rect).setTo(255);
        //current_img_.copyTo(maskedImage, mask);
        //current_img_ = maskedImage.clone();

        //ROS_INFO_STREAM("CALLBACK");
        //if (ecm_status_.data[2] == 1) // you don't need this
        //{
            if (!prev_img_.empty())
            {
                computeOpticalFlow(prev_img_, current_img_);
            }

            prev_img_ = current_img_.clone();
        //}
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void OpticalFlowPose::PublishRenderedImage(image_transport::Publisher pub, cv::Mat image, std::string encoding, std::string frame_id)
{
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = frame_id;
    const sensor_msgs::ImagePtr rendered_image_msg = cv_bridge::CvImage(header, encoding, image).toImageMsg();
    pub.publish(rendered_image_msg);
}

void OpticalFlowPose::computeOpticalFlow(const cv::Mat &prev, const cv::Mat &current)
{

    cv::Mat old_gray;
    cv::Mat new_gray;
    cv::Mat mask;
    cv::Rect rect(386, 0, 1087 - 386, 200 - 0);
    mask = cv::Mat::zeros(current.size(), CV_8UC1);
    mask(rect).setTo(255); 

    if (prev.type() != CV_8UC1)
    {
        cv::cvtColor(prev, old_gray, cv::COLOR_BGR2GRAY);
    }
    else
    {
        old_gray = prev; // If it's already grayscale
    }

    if (current.type() != CV_8UC1)
    {
        cv::cvtColor(current, new_gray, cv::COLOR_BGR2GRAY);
    }
    else
    {
        new_gray = current; // If it's already grayscale
    }


    std::vector<cv::Point2f> points_current; //vettore dinamico che conterrà un numero variabile di punti 2D (cv::Point2f)
    // Extract good features to track
    if(first_time_)
    {
        cv::goodFeaturesToTrack(old_gray, points_prev_, 10000, 0.01, 15,mask); //input image, output array, max_points, quality level, min distance
        //cv::Mat descriptor = extractFeatures(old_gray, points_prev_);
        first_time_ = false;
    }

    std::vector<uchar> status; //vettore che contiene valori di tipo unsigned char (numeri interi senza segno, che varia da 0 a 255)
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(old_gray, new_gray, points_prev_, points_current,status,err); //immagine precedente, immagine successiva, punti precedenti, punti successivi

    //points_current contiene le nuove posizioni delle features

    std::vector<cv::Point2f> good_old; //conterrà i punti del frame precedente che sono stati correttamente mappati in quello successivo
    std::vector<cv::Point2f> good_new; //punti mappati nel frame corrente 



    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) 
        {  // Se il punto è stato tracciato correttamente
            good_old.push_back(points_prev_[i]);  //punti precedenti mappati nel frame corrente 
            good_new.push_back(points_current[i]); //punti mappati nel frame corrente
        }
    }

    std::vector<bool> dynamic(good_new.size(), false); // 1 se dinamico, 0 se statico

    //ORA CHE HO I NUOVI PUNTI DELLE FEATURE POSSO CONTROLLARE SE SONO STATICHE O DINAMICHE 
    for (size_t i = 0; i < good_old.size(); ++i) 
    {
        double movement = cv::norm(good_new[i]-good_old[i]);
        if(movement >= movement_threshold_) //the feature is dynamic
            dynamic[i] = true; 
        else //the feature is static
            dynamic[i] = false;
    }



    // You can use these for pose recovery
    recoverPose(good_old, good_new,dynamic);
    points_prev_ = good_new;
}

void OpticalFlowPose::recoverPose(const std::vector<cv::Point2f> &good_old, const std::vector<cv::Point2f> &good_new,const std::vector<bool>& dynamic)
{

    // Example intrinsic matrix, update with your camera parameters
    cv::Mat K = (cv::Mat_<float>(3, 3) << 745.0165, 0.0, 667.0261, 0.0, 745.6793, 366.4256, 0.0, 0.0, 1.0);

    // Assuming we have some camera motion model here
    cv::Mat R, t;
    // Calculate essential matrix and recover pose
    // You might need to filter points or ensure they're in the same frame
    cv::Mat E = cv::findEssentialMat(good_old, good_new, K, cv::RANSAC);
    cv::recoverPose(E, good_old, good_new, K, R, t);

    
    /*
    for (const auto &point : good_new)
    {
        cv::circle(current_img_, point, 5, cv::Scalar(0, 255, 0), -1); // Green color, filled circle
    }
    */

    for (size_t i = 0; i < good_new.size(); ++i)
    {
        if (dynamic[i])
            cv::circle(current_img_, good_new[i], 5, cv::Scalar(0, 0, 255), -1);  // Red color, filled circle
        else 
            cv::circle(current_img_, good_new[i], 5, cv::Scalar(0, 255, 0), -1);  // Green color, filled circle

    }



    PublishRenderedImage(image_pub_, current_img_, "bgr8", "endoscope");

    //ROS_INFO("Rotation Matrix:\n %s", R);
    std::cout << " R " << R << std::endl;
    std::cout << " t " << t << std::endl;
    //ROS_INFO("Translation Vector:\n %s", t);
}
cv::Mat OpticalFlowPose::extractFeatures(const cv::Mat &img, std::vector<cv::Point2f> &keypoints2f)
{
    cv::Mat gray_img;

    if (img.type() != CV_8UC1)
    {
        cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);
    } 
    else
    {
        gray_img = img; // If it's already grayscale
    }

    cv::Ptr<cv::ORB> features = cv::ORB::create();
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    features->detectAndCompute(gray_img, cv::Mat(), keypoints, descriptors);

    cv::KeyPoint::convert(keypoints, keypoints2f);

    return descriptors;
}
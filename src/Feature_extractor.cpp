#include "Feature_extractor.h"
#include "Triangulation.h"
#include "camera_pose_estimation.h"
#include "OpticalFlow.h"


FeatureExtractor::FeatureExtractor() {}

cv::Mat FeatureExtractor::adaptiveHistogramEqualization(const cv::Mat &img) {
    //Increase contrast
    cv::Mat lab_image;
    cv::cvtColor(img, lab_image, cv::COLOR_BGR2Lab);

    // Estrazione L channel
    std::vector<cv::Mat> lab_planes(3);
    cv::split(lab_image, lab_planes);


    // CLAHE algorithm per L channel
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(4);
    cv::Mat dst;
    clahe->apply(lab_planes[0], dst);

    // Merge
    dst.copyTo(lab_planes[0]);
    cv::merge(lab_planes,lab_image);


    
    // GRAY
    cv::Mat image_clahe_RGB;
    cv::cvtColor(lab_image, image_clahe_RGB, cv::COLOR_Lab2BGR);
    cv::Mat image_clahe_gray;
    cv::cvtColor(image_clahe_RGB,image_clahe_gray , cv::COLOR_BGR2GRAY);

    return image_clahe_gray;

}

void FeatureExtractor::featureDetection(const cv::Mat &current,image_transport::Publisher image_pub_) {
// Camera Matrix for Left Camera
    cv::Mat cameraMatrixLeft = (cv::Mat_<double>(3,3) << 
        1039.6275634765625, 0, 596.5435180664062,
        0, 1039.4129638671875, 502.235595703125,
        0, 0, 1);

    // Camera Matrix for Right Camera
    cv::Mat cameraMatrixRight = (cv::Mat_<double>(3,3) << 
        1040.2305908203125, 0, 693.2049560546875,
        0, 1040.0146484375, 502.83544921875,
        0, 0, 1);

    // Distortion Coefficients for Left Camera
    cv::Mat distCoeffsLeft = (cv::Mat_<double>(1,5) << 
        -0.0038167661987245083, 0.01493496261537075, 0.0, 0.0, 0.012531906366348267);

    // Distortion Coefficients for Right Camera
    cv::Mat distCoeffsRight = (cv::Mat_<double>(1,5) << 
        -0.003966889809817076, 0.016439124941825867, 0.0, 0.0, 0.01571226492524147);

    // Rotation Matrix for Right Camera (from Left Camera to Right Camera)
    cv::Mat rotationMatrixRight = (cv::Mat_<double>(3,3) << 
        0.9999994731380644, 1.988273724971613e-05, -0.001026317821383099,
        -1.9900816839777227e-05, 0.9999999996469973, -1.7605766674560638e-05,
        0.001026317470969973, 1.762618196173469e-05, 0.9999994731807444);

    // Convert Rotation Matrix to Rotation Vector
    cv::Mat rotationVectorRight;
    cv::Rodrigues(rotationMatrixRight, rotationVectorRight);

    // Rotation Vector for Left Camera (Identity, so it's a zero vector)
    cv::Mat rotationVectorLeft = (cv::Mat_<double>(3,1) << 0, 0, 0);

    // Translation Vector for Left Camera (No translation, so it's a zero vector)
    cv::Mat translationVectorLeft = (cv::Mat_<double>(3,1) << 0, 0, 0);

    // Translation Vector for Right Camera (relative to the Left Camera)
    cv::Mat translationVectorRight = (cv::Mat_<double>(3,1) << 
        -4.3841071128845215, -0.027004247531294823, 0.028849583119153976);

    // Get image size
    int height = current.rows;
    int width = current.cols;

    // Define ROIs
    cv::Rect topROI(0, 0, width, height / 2);
    cv::Rect bottomROI(0, height / 2, width, height / 2);

    // Extract regions
    cv::Mat left_img_RGB = current(topROI);
    cv::Mat right_img_RGB  = current(bottomROI);

    //Increase contrast + grayscale 
    cv::Mat left_img = adaptiveHistogramEqualization(left_img_RGB);
    cv::Mat right_img = adaptiveHistogramEqualization(right_img_RGB);


    // Feature detection with GFTT
    cv::Ptr<cv::GFTTDetector> gftt = cv::GFTTDetector::create(500, 0.01, 1, 3, false, 0.04);
    std::vector<cv::KeyPoint> left_kpt, right_kpt;
    gftt->detect(left_img, left_kpt);
    gftt->detect(right_img, right_kpt);

    ROS_INFO_STREAM("Descriptors in bottom: " << right_kpt.size());
    ROS_INFO_STREAM("Descriptors in top: " << left_kpt.size());

    //GFTT non restituisce i descrittori quindi usiamo orb per trovare i descrittori
    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    cv::Mat left_des, right_des;
    orb->compute(left_img, left_kpt, left_des);
    orb->compute(right_img, right_kpt, right_des);

    ROS_INFO_STREAM("Features in bottom: " << right_des.size());
    ROS_INFO_STREAM("Features in top: " << left_des.size());

    std::vector<cv::DMatch> good_matches;
    good_matches = Triangulation::good_match(left_des, right_des);

    std::vector<cv::Point2f> good_matches_img1_coords;
    std::vector<cv::Point2f> good_matches_img2_coords;

    for (size_t i = 0; i < good_matches.size(); i++) {
        // Get the keypoints corresponding to the good matches

        cv::Point2f pt1 = left_kpt[good_matches[i].queryIdx].pt;  // Query image
        cv::Point2f pt2 = right_kpt[good_matches[i].trainIdx].pt;  // Train image
        
        // Store the coordinates
        good_matches_img1_coords.push_back(pt1);
        good_matches_img2_coords.push_back(pt2);
        //printf("Good Match %d: Image1 Keypoint (%.2f, %.2f) <--> Image2 Keypoint (%.2f, %.2f)\n",(int)i, pt1.x, pt1.y, pt2.x, pt2.y);
    }

    // Draw the good matches
    cv::Mat img_matches;
    drawMatches(left_img, left_kpt, right_img, right_kpt, good_matches, img_matches);


    OpticalFlowPose::PublishRenderedImage(image_pub_, img_matches, "bgr8", "endoscope");    

    std::vector<cv::Point2f> undistCoords1, undistCoords2;
    cv::undistortPoints(good_matches_img1_coords, undistCoords1, cameraMatrixLeft, distCoeffsLeft, cv::noArray(), cameraMatrixLeft);
    cv::undistortPoints(good_matches_img2_coords, undistCoords2, cameraMatrixRight, distCoeffsRight, cv::noArray(), cameraMatrixRight);

    cv::Mat projMatLeft, projMatRight;
    projMatLeft = Triangulation::computeProjMat(cameraMatrixLeft, rotationVectorLeft, translationVectorLeft);
    projMatRight = Triangulation::computeProjMat(cameraMatrixRight, rotationVectorRight, translationVectorRight);

    std::vector<cv::Vec3d> triangulatedPoints3D;

    triangulatedPoints3D = Triangulation::triangulate(projMatLeft, projMatRight, undistCoords1,undistCoords2 );
}

cv::Mat FeatureExtractor::extractFeatures(const cv::Mat &img, std::vector<cv::Point2f> &keypoints2f)
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

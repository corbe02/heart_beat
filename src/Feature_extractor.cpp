#include "Feature_extractor.h"
#include "Triangulation.h"
#include "OpticalFlowPose.h"
#include "OpticalFlow.h"
#include "Wavelets.h"
#include "data.h"
#include "TrackedMatch.h"



FeatureExtractor::FeatureExtractor(double thre):first_time_(true), thres_(thre) {
}

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

void FeatureExtractor::featureDetection(const cv::Mat &left_prev,const cv::Mat &right_prev, const cv::Mat &left_curr,const cv::Mat &right_curr,image_transport::Publisher image_pub_) {
    //Aumenta il contrasto e converti in scala di grigi
    cv::Mat left_img = adaptiveHistogramEqualization(left_curr);
    cv::Mat right_img = adaptiveHistogramEqualization(right_curr);

    cv::Mat left_img_prev = adaptiveHistogramEqualization(left_prev);
    cv::Mat right_img_prev = adaptiveHistogramEqualization(right_prev);


    if(first_time_)
    {    

        //-----------------------FEATURE DETECTION AND FILTERING---------------------------------------------------------------------
        //Svuoto tracked matches 
        tracked_matches.clear();
        // Feature detection with GFTT
        // Crea un rilevatore di feature GFTT (Good Features To Track) con:
        // - massimo numero punti da rilevare
        // - qualità minima richiesta dei punti 
        // - distanza minima tra i punti = 10 pixel
        // - finestra di analisi 3x3 pixel
        // - usare Harris detector --> true
        // - parametro k (Harris) settato a 0.04 (non usato qui)
        cv::Ptr<cv::GFTTDetector> gftt = cv::GFTTDetector::create(2500, 0.001, 10, 3, true, 0.04); 
        std::vector<cv::KeyPoint> left_kpt, right_kpt;
        gftt->detect(left_img, left_kpt);
        gftt->detect(right_img, right_kpt);

        ROS_INFO_STREAM("Features in bottom: " << right_kpt.size());
        ROS_INFO_STREAM("Features in top: " << left_kpt.size());

        //GFTT non restituisce i descrittori quindi usiamo orb per trovare i descrittori
        cv::Ptr<cv::ORB> orb = cv::ORB::create(2500);
        cv::Mat left_des, right_des;
        orb->compute(left_img, left_kpt, left_des);
        orb->compute(right_img, right_kpt, right_des);

        ROS_INFO_STREAM("Descriptors in bottom: " << right_des.size());
        ROS_INFO_STREAM("Descriptors in top: " << left_des.size());

        tracked_matches = Triangulation::good_match(left_des, right_des); //vettore di struct 

        std::vector<cv::Point2f> good_matches_img1_coords;
        std::vector<cv::Point2f> good_matches_img2_coords;

    // ---------------------------- POINT EXTRACTION --------------------------------------------------
        for (size_t i = 0; i < tracked_matches.size(); i++) {
            // Get the keypoints corresponding to the good matches
            cv::Point2f pt1 = left_kpt[tracked_matches[i].match.queryIdx].pt;  // Query image 
            cv::Point2f pt2 = right_kpt[tracked_matches[i].match.trainIdx].pt;  // Train image
            tracked_matches[i].pt_right = pt2;
            tracked_matches[i].pt_left = pt1;
            
            // Store the coordinates of the points for the triangulation
            good_matches_img1_coords.push_back(pt1);
            good_matches_img2_coords.push_back(pt2);
        }


    // ------------------------------- TRIANGULATION -----------------------------------------------
        //Computation of projection matrix
        cv::Mat projMatLeft, projMatRight;
        projMatLeft = Triangulation::computeProjMat(cameraMatrixLeft, rotationVectorLeft, translationVectorLeft);
        projMatRight = Triangulation::computeProjMat(cameraMatrixRight, rotationVectorRight, translationVectorRight);

        triangulatedPoints3D = Triangulation::triangulate(projMatLeft, projMatRight, good_matches_img1_coords,good_matches_img2_coords);
        for (size_t i = 0; i < tracked_matches.size(); ++i) {
            tracked_matches[i].position_3d = triangulatedPoints3D[i];
        }
        ROS_INFO_STREAM(" triangulated point size 1: " << triangulatedPoints3D.size());
        //points_prev_left_ = good_matches_img1_coords; 
        //points_prev_right_  = good_matches_img2_coords;
        first_time_ = false;

    }

    //----------------------------------------TRIANGULATION-------------------------------------------------------
    // First, you run the optical flow triangulation for both left and right images --> optical flow of triangulated points
    OpticalFlow::OpticalFlowTriangulation(left_img_prev, left_img,right_img_prev, right_img, thres_, tracked_matches);

    std::vector<cv::Point2f> of_left;
    std::vector<cv::Point2f> of_right;
    std::vector<cv::Vec3d> tracked3dpts;

    for (const auto& match : tracked_matches) {
        if (!match.is_active) continue; 

        // Separiamo left e right points 
        of_left.push_back(cv::Point2f(match.pt_left)); 
        of_right.push_back(cv::Point2f(match.pt_right)); 
        tracked3dpts.push_back(match.position_3d);
    }
    points_prev_left_ = of_left;
    points_prev_right_ = of_right;
    triangulatedPoints3D = tracked3dpts;

    // std::cout << "Right image feature points:\n";
    // for (const auto& pt : of_right) {
    //     std::cout << "  (" << pt.x << ", " << pt.y << ")\n";
    // }

    tracked3dpts_previous = tracked3dpts;


    // Draw points on top half (left image)
    for (size_t i = 0; i < of_left.size(); i++) {
        cv::Point2f pt = of_left[i];
        cv::circle(left_curr, pt, 5, cv::Scalar(0, 255, 0), -1);
    }

    // Draw points on bottom half (right image)
    for (size_t i = 0; i < of_right.size(); i++) {
        cv::Point2f pt = of_right[i];
        // DO NOT subtract roi_offset_y here
        cv::circle(right_curr, pt, 5, cv::Scalar(0, 255, 0), -1);
    }

    // Create a combined image with the left and right images stacked vertically.
    cv::Mat img_combined;
    cv::vconcat(left_curr, right_curr, img_combined);
    // Finally, you can publish the image with the visualized points.
    OpticalFlowPose::PublishRenderedImage(image_pub_, img_combined, "bgr8", "endoscope");

    if(of_left.size()<70){
        first_time_ = true;
    }
    

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
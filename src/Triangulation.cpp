#include "Triangulation.h"
#include <ros/ros.h>

Triangulation::Triangulation() {}

std::vector<cv::DMatch> Triangulation::good_match(cv::Mat &left_des, cv::Mat &right_des)
{
    /*
    //Brute Force per fare il matching 
    cv::BFMatcher matcher(cv::NORM_HAMMING, true);
    std::vector<cv::DMatch> matches;
    matcher.match(left_des, right_des, matches);

    // Filter matches 
    std::vector<cv::DMatch> good_matches;
    for (int i = 0; i < matches.size(); i++) {
        if (matches[i].distance <= 35 && matches[i].distance >= -35) { 
            good_matches.push_back(matches[i]);
        }
    }

    ROS_INFO_STREAM("Good matches: " << good_matches.size());
    */

    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher.knnMatch(left_des, right_des, knn_matches, 2);

    // Apply Lowe's ratio test
    std::vector<cv::DMatch> good_matches;
    float ratio_thresh = 0.65f;
    for (size_t i = 0; i < knn_matches.size(); i++) {
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
            good_matches.push_back(knn_matches[i][0]);
        }
    }

    
    // Find the minimum distance among good matches
    double min_dist = std::numeric_limits<double>::max();
    for (const auto &match : good_matches) {
        if (match.distance < min_dist) {
            min_dist = match.distance;
        }
    }

    // Apply stricter filtering: Keep matches with distance < 2 * min_dist
    std::vector<cv::DMatch> best_matches;
    double threshold = 6.0 * min_dist;
    for (const auto &match : good_matches) {
        if (match.distance < threshold) {
            best_matches.push_back(match);
        }
    }
    ROS_INFO_STREAM("good_matches: " << good_matches.size());
    ROS_INFO_STREAM("best_matches: " << best_matches.size());
    
    return best_matches;

}

std::vector<cv::Vec3d> Triangulation::triangulate(cv::Mat &projMatLeft, cv::Mat &projMatRight,std::vector<cv::Point2f> &undistCoords1,std::vector<cv::Point2f> &undistCoords2)
{
    cv::Mat triangCoords4D;
    cv::triangulatePoints(projMatLeft, projMatRight, undistCoords1, undistCoords2, triangCoords4D);

    std::vector<cv::Vec3d> triangulatedPoints3D;

    for (int i = 0; i < triangCoords4D.cols; i++) {
        cv::Vec4d homogeneousPoint = triangCoords4D.col(i);
        cv::Vec3d euclideanPoint;

        for (int j = 0; j < 3; j++) {
            euclideanPoint[j] = homogeneousPoint[j] / homogeneousPoint[3];  // Conversione da omogeneo a euclideo
        }

        triangulatedPoints3D.push_back(euclideanPoint);
    }

    /*Stampa i punti 3D ottenuti
    for (const auto& point : triangulatedPoints3D) {
        std::cout << "Punto 3D: " << point << std::endl;
    }
    */ 

    return triangulatedPoints3D;


}

cv::Mat Triangulation::computeProjMat(const cv::Mat &camMat, const cv::Mat &rotVec, const cv::Mat &transVec)
{
    cv::Mat rotMat(3, 3, CV_64F), RTMat(3, 4, CV_64F);

    // 1. Convert rotation vector into rotation matrix
    cv::Rodrigues(rotVec, rotMat);

    // 2. Append translation vector to rotation matrix
    cv::hconcat(rotMat, transVec, RTMat);

    // 3. Compute projection matrix: P = K * [R | t]
    return camMat * RTMat;
}

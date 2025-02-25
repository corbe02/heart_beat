#include "Triangulation.h"
#include <ros/ros.h>

Triangulation::Triangulation() {}

std::vector<cv::DMatch> Triangulation::good_match(cv::Mat &left_des, cv::Mat &right_des)
{
    //Brute Force per fare il matching 
    cv::BFMatcher matcher(cv::NORM_HAMMING, true);
    std::vector<cv::DMatch> matches;
    matcher.match(left_des, right_des, matches);

    /*
    cv::Mat match_img;
    cv::drawMatches(left_img, query_kpt, right_img, cand_kpt, matches, match_img);
    cv::imshow("Feature Matches", match_img);
    cv::waitKey(0);
    */

    //EXTRACT GOOD MATCHES
    // Calculate the min and max distances between keypoints
    /*
    double max_dist = 0;
    double min_dist = 100;

    for (int i = 0; i < left_des.rows; i++) {
        double dist = matches[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }

    printf("Max distance: %f\n", max_dist);
    printf("Min distance: %f\n", min_dist);

    */
    // Filter matches 
    std::vector<cv::DMatch> good_matches;
    for (int i = 0; i < matches.size(); i++) {
        if (matches[i].distance <= 50 && matches[i].distance >= -50) { 
            good_matches.push_back(matches[i]);
        }
    }

    ROS_INFO_STREAM("Good matches: " << good_matches.size());

    return good_matches;

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

    // Stampa i punti 3D ottenuti
    for (const auto& point : triangulatedPoints3D) {
        std::cout << "Punto 3D: " << point << std::endl;
    }

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

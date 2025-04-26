#include "Triangulation.h"
#include <ros/ros.h>


Triangulation::Triangulation() {}


std::vector<TrackedMatch> Triangulation::good_match(cv::Mat &left_des, cv::Mat &right_des)
{
    // uso brute force per il matching e poi filtro le match in base a 2 criteri:
    // 1. Lowe's ratio test
    // 2. distanza

    // Crea un matcher Brute-Force usando la distanza di Hamming (adatto per descrittori binari tipo ORB)
    // Prepara una struttura per salvare i match: per ogni descrittore, memorizziamo i 2 migliori candidati
    // Esegue il KNN matching tra descrittori di sinistra (left_des) e destra (right_des), trovando i 2 match migliori per ciascun punto
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher.knnMatch(left_des, right_des, knn_matches, 2);


    //Lowe's ratio test
    //Serve a prendere solo i match in cui il migliore è chiaramente migliore del secondo!
    //Ho preso le due match migliori e confronto tutti gli elementi delle due match e vedo se
    //la prima match è nettamente migliore della seoconda 
    std::vector<cv::DMatch> good_matches;
    float ratio_thresh = 0.65f;
    for (size_t i = 0; i < knn_matches.size(); i++) {
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
            good_matches.push_back(knn_matches[i][0]);
        }
    }
    
    // Trovo la distanza minima tra tutte le features 
    double min_dist = std::numeric_limits<double>::max();
    for (const auto &match : good_matches) {
        if (match.distance < min_dist) {
            min_dist = match.distance;
        }
    }

    // Apply stricter filtering: Keep matches with distance < x * min_dist
    std::vector<cv::DMatch> best_matches;
    double threshold = 6.0 * min_dist;
    for (const auto &match : good_matches) {
        if (match.distance < threshold) {
            best_matches.push_back(match);
        }
    }
    ROS_INFO_STREAM("good_matches: " << good_matches.size());
    ROS_INFO_STREAM("best_matches: " << best_matches.size());

    std::vector<TrackedMatch> best_tracked_matches; //vettore di struct
    int next_id = 0; // Incremental ID counter

    for (const auto &match : best_matches) {
        TrackedMatch tracked;
        tracked.id = next_id++;        // Assign a new unique ID
        tracked.match = match;
        tracked.is_active = true;      // New matches are active by default
        best_tracked_matches.push_back(tracked);
    }
    

    return best_tracked_matches;

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

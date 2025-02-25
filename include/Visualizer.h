#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <opencv2/opencv.hpp>

class Visualizer {
public:
    Visualizer(); 
    static void drawDelaunay(const cv::Mat &current, std::vector<cv::Point2f> &good_new, const cv::Scalar& color);
    static void drawVoronoi(const cv::Mat &current, std::vector<cv::Point2f> &good_new, const cv::Scalar& color);
};

#endif // VISUALIZER_H

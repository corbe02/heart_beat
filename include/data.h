#ifndef DATA_H
#define DATA_H

#include <opencv2/opencv.hpp>

extern cv::Mat cameraMatrixLeft;
extern cv::Mat cameraMatrixRight;
extern cv::Mat distCoeffsLeft;
extern cv::Mat distCoeffsRight;
extern cv::Mat rotationMatrixRight;
extern cv::Mat rotationVectorRight;
extern cv::Mat rotationVectorLeft;
extern cv::Mat translationVectorLeft;
extern cv::Mat translationVectorRight;

#endif // DATA_H

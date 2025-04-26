#include "data.h"

// Initialize matrices

cv::Mat cameraMatrixLeft = (cv::Mat_<double>(3,3) << 
    1039.6275634765625, 0, 596.5435180664062,
    0, 1039.4129638671875, 502.235595703125,
    0, 0, 1);

cv::Mat cameraMatrixRight = (cv::Mat_<double>(3,3) << 
    1040.2305908203125, 0, 693.2049560546875,
    0, 1040.0146484375, 502.83544921875,
    0, 0, 1);

cv::Mat distCoeffsLeft = (cv::Mat_<double>(1,5) << 
    -0.0038167661987245083, 0.01493496261537075, 0.0, 0.0, 0.012531906366348267);

cv::Mat distCoeffsRight = (cv::Mat_<double>(1,5) << 
    -0.003966889809817076, 0.016439124941825867, 0.0, 0.0, 0.01571226492524147);

cv::Mat rotationMatrixRight = (cv::Mat_<double>(3,3) << 
    0.9999994731380644, 1.988273724971613e-05, -0.001026317821383099,
    -1.9900816839777227e-05, 0.9999999996469973, -1.7605766674560638e-05,
    0.001026317470969973, 1.762618196173469e-05, 0.9999994731807444);

cv::Mat rotationVectorRight = [](){
    cv::Mat vec;
    cv::Rodrigues(rotationMatrixRight, vec);
    return vec;
}(); // <-- lambda trick to initialize properly

cv::Mat rotationVectorLeft = (cv::Mat_<double>(3,1) << 0, 0, 0);
cv::Mat translationVectorLeft = (cv::Mat_<double>(3,1) << 0, 0, 0);
cv::Mat translationVectorRight = (cv::Mat_<double>(3,1) << 
    -4.3841071128845215, -0.027004247531294823, 0.028849583119153976);

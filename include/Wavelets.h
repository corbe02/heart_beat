#ifndef WAVELETS_H
#define WAVELETS_H

#include <opencv2/opencv.hpp>

class Wavelets {
public:
    Wavelets(); 
    static void Haar_transform(std::vector<double> &movement_series);
    static void Daubechies_transform(std::vector<double> &movement_series);

};

#endif // WAVELETS_H

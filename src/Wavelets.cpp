#include "Wavelets.h"
#include "wavelib.h"
#include "TrackedMatch.h"

Wavelets::Wavelets() {}

void Wavelets::Haar_transform(std::vector<double> &movement_series)
{
    // Initialize wavelet transform
    wave_object wave = wave_init("db4"); // Daubechies wavelet with 4 coefficients
    wt_object wt = wt_init(wave, "dwt", movement_series.size(), 1);

    // Perform DWT
    setDWTExtension(wt, "sym"); // Symmetric extension
    setWTConv(wt, "direct");
    dwt(wt, movement_series.data());

    // Print Approximation & Detail coefficients
    std::cout << "Approximation coefficients:\n ";
    for (int i = 0; i < wt->outlength; i++)
        std::cout << wt->output[i] << " ";
    
    std::cout << "\nDetail coefficients: \n";
    for (int i = wt->outlength; i < movement_series.size(); i++)
        std::cout << wt->output[i] << " ";

    // Cleanup
    wave_free(wave);
    wt_free(wt);

}

void Wavelets::Daubechies_transform(std::vector<double> &movement_series)
{
    // Initialize Daubechies wavelet with 4 coefficients (db4)
    wave_object wave = wave_init("db4");  // You can change "db4" to other Daubechies like "db2", "db8", etc.
    
    // Initialize Wavelet Transform (DWT)
    wt_object wt = wt_init(wave, "dwt", movement_series.size(), 1);

    // Perform Discrete Wavelet Transform (DWT)
    setDWTExtension(wt, "sym"); // Symmetric extension of the signal
    setWTConv(wt, "direct");    // Direct convolution for transform
    dwt(wt, movement_series.data());  // Apply DWT to movement_series

    // Print Approximation coefficients
    std::cout << "Approximation coefficients: \n";
    for (int i = 0; i < wt->outlength; i++)
        std::cout << wt->output[i] << " ";
    
    // Print Detail coefficients
    std::cout << "\nDetail coefficients: \n";
    for (int i = wt->outlength; i < movement_series.size(); i++)
        std::cout << wt->output[i] << " ";

    // Cleanup
    wave_free(wave);
    wt_free(wt);
}
#include "OpticalFlow.h"
#include "Visualizer.h"
#include "OpticalFlowPose.h"
#include <image_transport/image_transport.h>

OpticalFlow::OpticalFlow() {}

void OpticalFlow::computeOpticalFlow(const cv::Mat &prev, cv::Mat &current, double &movement_threshold_)
{

    cv::Mat old_gray;
    cv::Mat new_gray;
    /*
    cv::Mat mask;
    cv::Rect rect(386, 0, 1087 - 386, 200 - 0);
    mask = cv::Mat::zeros(current.size(), CV_8UC1);
    mask(rect).setTo(255); 
    */

    if (prev.type() != CV_8UC1)
    {
        cv::cvtColor(prev, old_gray, cv::COLOR_BGR2GRAY);
    }
    else
    {
        old_gray = prev; // If it's already grayscale
    }

    if (current.type() != CV_8UC1)
    {
        cv::cvtColor(current, new_gray, cv::COLOR_BGR2GRAY);
    }
    else
    {
        new_gray = current; // If it's already grayscale
    }


    std::vector<cv::Point2f> points_current; //vettore dinamico che conterrà un numero variabile di punti 2D (cv::Point2f)
    // Extract good features to track
    if(first_time_)
    {
        //cv::goodFeaturesToTrack(old_gray, points_prev_, 10000, 0.01, 5,mask); //input image, output array, max_points, quality level, min distance, mask
        cv::goodFeaturesToTrack(old_gray, points_prev_, 1000, 0.000001, 100);
        dynamic_points_prev = std::vector<bool>(points_prev_.size(), false);
        //cv::Mat descriptor = extractFeatures(old_gray, points_prev_);
        first_time_ = false;
    }

    std::vector<uchar> status; //vettore che contiene valori di tipo unsigned char (numeri interi senza segno, che varia da 0 a 255)
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(old_gray, new_gray, points_prev_, points_current,status,err); //immagine precedente, immagine successiva, punti precedenti, punti successivi
    // points prev sono solo i punti correttamente tracciati nel frame precedente 


    //points_current contiene le nuove posizioni delle features

    std::vector<cv::Point2f> good_old; //conterrà i punti del frame precedente che sono stati correttamente mappati in quello corrente 
    std::vector<cv::Point2f> good_new; //punti mappati nel frame corrente 


    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) 
        {  // Se il punto è stato tracciato correttamente
            good_old.push_back(points_prev_[i]);  //punti precedenti mappati nel frame corrente 
            good_new.push_back(points_current[i]); //punti mappati nel frame corrente
        }
    }

    std::vector<bool> dynamic_current(good_new.size(), false); // 1 se dinamico, 0 se statico
    int counter = 0;
    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) 
        {  // Se il punto è stato tracciato correttamente
            dynamic_current[counter] = dynamic_points_prev[i];
            counter ++;
        }
    }

    //Dynamic current contiene i punti che sono stati correttamente tracciati e che erano dinamici nel frame precedente

    

    //Features statiche o dinamiche 
    for (size_t i = 0; i < good_old.size(); ++i) 
    {
        double movement = cv::norm(good_new[i]-good_old[i]);
        if(movement >= movement_threshold_) //the feature is dynamic
            dynamic_current[i] = true; //aggiungo eventuali altri punti considerati dinamici, ma quelli dinamici non possono tornare statici
        //else //the feature is static
            //dynamic[i] = false;
    }
 
    Visualizer::drawDelaunay(current, good_new, cv::Scalar(255, 0, 0)); //disegno i triangoli di Delaunay 
    Visualizer::drawVoronoi(current, good_new, cv::Scalar(0, 255, 0)); // disegno il diagramma di Voronoi

    OpticalFlowPose::recoverPose(good_old, good_new,dynamic_current, current);
    points_prev_ = good_new;

    if (dynamic_points_prev.size() != dynamic_current.size()) 
    {
        dynamic_points_prev.resize(dynamic_current.size(), false);  // Inizializza con valore false (statico)
    }
    dynamic_points_prev = dynamic_current;

}

std::vector<uchar> OpticalFlow::OpticalFlowTriangulation(const cv::Mat &prev, cv::Mat &current, double &movement_threshold_,
                                          std::vector<bool> &dynamic_points_prev, std::vector<cv::Point2f> &points_prev,cv::Mat left_img_RGB)
{


    //optical flow on one of the two images (ex left)

    std::vector<cv::Point2f> points_current; //vettore dinamico che conterrà un numero variabile di punti 2D (cv::Point2f)

    std::vector<uchar> status; //vettore che contiene valori di tipo unsigned char (numeri interi senza segno, che varia da 0 a 255)
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(prev, current, points_prev, points_current,status,err);

    //points_current contiene le nuove posizioni delle features

    std::vector<cv::Point2f> good_old; //conterrà i punti del frame precedente che sono stati correttamente mappati in quello corrente 
    std::vector<cv::Point2f> good_new; //punti mappati nel frame corrente 

    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) 
        {  // Se il punto è stato tracciato correttamente
            good_old.push_back(points_prev[i]);  //punti precedenti mappati nel frame corrente 
            good_new.push_back(points_current[i]); //punti mappati nel frame corrente
        }
    }


    std::vector<bool> dynamic_current(good_new.size(), false); // 1 se dinamico, 0 se statico
    int counter = 0;
    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) 
        {  // Se il punto è stato tracciato correttamente
            dynamic_current[counter] = dynamic_points_prev[i];
            counter ++;
        }
    }

    //Features statiche o dinamiche 
    for (size_t i = 0; i < good_old.size(); ++i) 
    {
        double movement = cv::norm(good_new[i]-good_old[i]);
        if(movement >= movement_threshold_) //the feature is dynamic
            dynamic_current[i] = true; //aggiungo eventuali altri punti considerati dinamici, ma quelli dinamici non possono tornare statici
        //else //the feature is static
            //dynamic[i] = false;
    }

    //OpticalFlowPose::recoverPose(good_old, good_new,dynamic_current, left_img_RGB);
    points_prev = good_new;

    if (dynamic_points_prev.size() != dynamic_current.size()) 
    {
        dynamic_points_prev.resize(dynamic_current.size(), false);  // Inizializza con valore false (statico)
    }
    dynamic_points_prev = dynamic_current;

    return status;



}
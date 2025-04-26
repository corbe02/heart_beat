#include "OpticalFlow.h"
#include "Visualizer.h"
#include "OpticalFlowPose.h"
#include <image_transport/image_transport.h>
#include "TrackedMatch.h"

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

void OpticalFlow::OpticalFlowTriangulation(const cv::Mat &prev_l, cv::Mat &current_l,
                                           const cv::Mat &prev_r, cv::Mat &current_r,
                                           double &movement_threshold_,
                                           std::vector<TrackedMatch> &tracked_matches) {
    // 1. Extract coordinates of active features
    std::vector<cv::Point2f> prevPointsLeft, prevPointsRight;

    // Keep mapping between optical flow index and tracked_matches index
    std::vector<size_t> opticalFlowToMatchIdx;

    //Per fare l'optical flow, devo utilizzare solo le features attive (che ho tracciato nel frame precedente)
    //Estraggo quindi le features attive e le metto in prevPointsLeft e prevPointsRight
    //in opticalFlowToMatchIdx metto gli indici delle features attive (ex: 2,7,13...)
    for (size_t i = 0; i < tracked_matches.size(); ++i) {
        if (tracked_matches[i].is_active) {
            // Retrieve left and right 2D points from your TrackedMatch structure
            cv::Point2f pt_left = tracked_matches[i].pt_left;   
            cv::Point2f pt_right = tracked_matches[i].pt_right; 

            prevPointsLeft.push_back(pt_left);
            prevPointsRight.push_back(pt_right);
            opticalFlowToMatchIdx.push_back(i);
        }
    }

    if (prevPointsLeft.empty() || prevPointsRight.empty())
        return; // No active points to track

    // -------------------------------------------- OPTICAL FLOW ----------------------------------------
    // 2. Perform optical flow on active points only
    std::vector<cv::Point2f> nextPointsLeft, nextPointsRight;
    std::vector<uchar> statusLeft, statusRight;
    std::vector<float> errLeft, errRight;

    cv::calcOpticalFlowPyrLK(prev_l, current_l, prevPointsLeft, nextPointsLeft, statusLeft, errLeft);
    cv::calcOpticalFlowPyrLK(prev_r, current_r, prevPointsRight, nextPointsRight, statusRight, errRight);

    // 3. Iterate through statuses and map back to tracked_matches
    // Itero su tutte le features che prima dell'optical flow erano attive (correttamente tracciate nel frame precedente)
    // Se statusleft o status right sono false, metto is_active a false ---> feature non tracciata 
    // Per trovare la posizione di quella feature nel vettore generale, utilizzo l'indice che ho salvato in opticalFlowToMatchIdx
    for (size_t flow_idx = 0; flow_idx < opticalFlowToMatchIdx.size(); ++flow_idx) {
        size_t match_idx = opticalFlowToMatchIdx[flow_idx];

        bool left_ok = statusLeft[flow_idx];
        bool right_ok = statusRight[flow_idx];

        if (!left_ok || !right_ok) {
            tracked_matches[match_idx].is_active = false;
            continue;
        }
            
        // Check movement thresholds
        float dx_left = nextPointsLeft[flow_idx].x - prevPointsLeft[flow_idx].x;
        float dy_left = nextPointsLeft[flow_idx].y - prevPointsLeft[flow_idx].y;
        float movementLeft = std::sqrt(dx_left * dx_left + dy_left * dy_left);

        float dx_right = nextPointsRight[flow_idx].x - prevPointsRight[flow_idx].x;
        float dy_right = nextPointsRight[flow_idx].y - prevPointsRight[flow_idx].y;
        float movementRight = std::sqrt(dx_right * dx_right + dy_right * dy_right);

        if (movementLeft < movement_threshold_ || movementRight < movement_threshold_) {
            tracked_matches[match_idx].dynamic_point= false;
        } else {
            // Update with new positions
            tracked_matches[match_idx].pt_left = nextPointsLeft[flow_idx];
            tracked_matches[match_idx].pt_right = nextPointsRight[flow_idx];

            tracked_matches[match_idx].dynamic_point = true;

            //re-triangulate if needed
            //tracked_matches[match_idx].position_3d = TriangulatePoint(...);
        }
    }
}

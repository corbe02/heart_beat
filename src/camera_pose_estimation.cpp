#include "camera_pose_estimation.h"

OpticalFlowPose::OpticalFlowPose(ros::NodeHandle &nh) : it_(nh),private_nh_("~"), first_time_(true)
{
    private_nh_.param("threshold",movement_threshold_, 0.0);
        
    image_sub_ = nh.subscribe("/video1/image_raw", 1, &OpticalFlowPose::imageCallback, this);

    //ecm_status_sub_ = nh.subscribe("/arm_detection/status", 1, &OpticalFlowPose::GetECMStatus, this); // you don't need this

    prev_img_ = cv::Mat(); // // Costruisce una matrice vuota che può essere poi popolata con un'immagine/valore
    image_pub_ = it_.advertise("/optical_flow/output_video", 1);
    //ecm_status_.data.resize(4, 0);
}

void OpticalFlowPose::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat current_copy;
    //cv::Mat mask;
    //cv::Rect rect(193, 0, 1087 - 193, 667 - 0);
    //cv::Mat maskedImage;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        current_img_ = cv_ptr->image;
        current_copy = current_img_.clone();
        //mask = cv::Mat::zeros(current_img_.size(), CV_8UC1);
        //mask(rect).setTo(255);
        //current_img_.copyTo(maskedImage, mask);
        //current_img_ = maskedImage.clone();

        //ROS_INFO_STREAM("CALLBACK");
        //if (ecm_status_.data[2] == 1) // you don't need this
        //{
            if (!prev_img_.empty())
            {
                computeOpticalFlow(prev_img_, current_img_);
            }

            prev_img_ = current_copy.clone();
        //}
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void OpticalFlowPose::PublishRenderedImage(image_transport::Publisher pub, cv::Mat image, std::string encoding, std::string frame_id)
{
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = frame_id;
    const sensor_msgs::ImagePtr rendered_image_msg = cv_bridge::CvImage(header, encoding, image).toImageMsg();
    pub.publish(rendered_image_msg);
}

void OpticalFlowPose::computeOpticalFlow(const cv::Mat &prev, const cv::Mat &current)
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

    std::vector<bool> dynamic(good_new.size(), false); // 1 se dinamico, 0 se statico

    //Features statiche o dinamiche 
    for (size_t i = 0; i < good_old.size(); ++i) 
    {
        double movement = cv::norm(good_new[i]-good_old[i]);
        if(movement >= movement_threshold_) //the feature is dynamic
            dynamic[i] = true; 
        else //the feature is static
            dynamic[i] = false;
    }
 
    drawDelaunay(current, good_new, cv::Scalar(255, 0, 0)); //disegno i triangoli di Delaunay 
    drawVoronoi(current, good_new, cv::Scalar(0, 255, 0)); // disegno il diagramma di Voronoi



    // You can use these for pose recovery
    recoverPose(good_old, good_new,dynamic);
    points_prev_ = good_new;
}

void OpticalFlowPose::recoverPose(const std::vector<cv::Point2f> &good_old, const std::vector<cv::Point2f> &good_new,const std::vector<bool>& dynamic)
{

    // Example intrinsic matrix, update with your camera parameters
    cv::Mat K = (cv::Mat_<float>(3, 3) << 745.0165, 0.0, 667.0261, 0.0, 745.6793, 366.4256, 0.0, 0.0, 1.0);

    // Assuming we have some camera motion model here
    cv::Mat R, t;
    // Calculate essential matrix and recover pose
    // You might need to filter points or ensure they're in the same frame
    cv::Mat E = cv::findEssentialMat(good_old, good_new, K, cv::RANSAC);
    cv::recoverPose(E, good_old, good_new, K, R, t);

    
    /*
    for (const auto &point : good_new)
    {
        cv::circle(current_img_, point, 5, cv::Scalar(0, 255, 0), -1); // Green color, filled circle
    }
    */

    for (size_t i = 0; i < good_new.size(); ++i)
    {
        if (dynamic[i])
            cv::circle(current_img_, good_new[i], 7, cv::Scalar(0, 0, 255), -1);  // Red color, filled circle
        else 
            cv::circle(current_img_, good_new[i], 7, cv::Scalar(255, 0, 0), -1);  // Blue color, filled circle

    }



    PublishRenderedImage(image_pub_, current_img_, "bgr8", "endoscope");

    //ROS_INFO("Rotation Matrix:\n %s", R);
    //std::cout << " R " << R << std::endl;
    //std::cout << " t " << t << std::endl;
    //ROS_INFO("Translation Vector:\n %s", t);
}
cv::Mat OpticalFlowPose::extractFeatures(const cv::Mat &img, std::vector<cv::Point2f> &keypoints2f)
{
    cv::Mat gray_img;

    if (img.type() != CV_8UC1)
    {
        cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);
    } 
    else
    {
        gray_img = img; // If it's already grayscale
    }

    cv::Ptr<cv::ORB> features = cv::ORB::create();
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    features->detectAndCompute(gray_img, cv::Mat(), keypoints, descriptors);

    cv::KeyPoint::convert(keypoints, keypoints2f);

    return descriptors;
}
void OpticalFlowPose::drawDelaunay(const cv::Mat &current, std::vector<cv::Point2f> &good_new, const cv::Scalar& color)
{
    if (good_new.size() < 3) // devo avere almeno 3 punti
        return;

    cv::Size size = current.size(); //trovo le dimensioni del frame 
    cv::Rect rect(0, 0, size.width, size.height); //creo una "maschera" delle dimensioni del mio frame 
    cv::Subdiv2D subdiv(rect);
    /*The function subdiv creates an empty Delaunay subdivision where 2D points can be added using the function insert() .
     All of the points to be added must be within the specified rectangle, otherwise a runtime error is raised.
    */

    // Inserimento punti in subdiv 
    for (const auto &point : good_new) //itero su tutto good_new
    {
        if (rect.contains(point))
        {
            try
            {
                subdiv.insert(point);
            }
            catch (const cv::Exception &e)
            {
                std::cerr << "Error inserting point in Delaunay: " << point << ", " << e.what() << std::endl;
            }
        }
    }

    // Ottieni triangoli e disegna
    std::vector<cv::Vec6f> triangleList;
    try
    {
        subdiv.getTriangleList(triangleList);
    }
    catch (const cv::Exception &e)
    {
        std::cerr << "Error retrieving triangle list: " << e.what() << std::endl;
        return;
    }

    std::vector<cv::Point> pt(3);
    for (size_t i = 0; i < triangleList.size(); i++)
    {
        cv::Vec6f t = triangleList[i];
        pt[0] = cv::Point(cvRound(t[0]), cvRound(t[1]));
        pt[1] = cv::Point(cvRound(t[2]), cvRound(t[3]));
        pt[2] = cv::Point(cvRound(t[4]), cvRound(t[5]));

        // Disegna triangoli completamente dentro l'immagine
        if (rect.contains(pt[0]) && rect.contains(pt[1]) && rect.contains(pt[2]))
        {
            cv::line(current, pt[0], pt[1], color, 1, cv::LINE_AA, 0);
            cv::line(current, pt[1], pt[2], color, 1, cv::LINE_AA, 0);
            cv::line(current, pt[2], pt[0], color, 1, cv::LINE_AA, 0);
        }
    }
}

void OpticalFlowPose::drawVoronoi(const cv::Mat &current, std::vector<cv::Point2f> &good_new, const cv::Scalar& color)
{
    if (good_new.size() < 2) //controllo di avere almeno 2 punti 
        return;

    cv::Size size = current.size();
    cv::Rect rect(0, 0, size.width, size.height);
    cv::Subdiv2D subdiv(rect);

    // Inserimento punti
    for (const auto &point : good_new)
    {
        if (rect.contains(point))
        {
            try
            {
                subdiv.insert(point);
            }
            catch (const cv::Exception &e)
            {
                std::cerr << "Error inserting point in Voronoi: " << point << ", " << e.what() << std::endl;
            }
        }
    }

    // Ottieni faccette di Voronoi
    std::vector<std::vector<cv::Point2f>> facets;
    std::vector<cv::Point2f> centers;
    try
    {
        subdiv.getVoronoiFacetList(std::vector<int>(), facets, centers);
    }
    catch (const cv::Exception &e)
    {
        std::cerr << "Error retrieving Voronoi facets: " << e.what() << std::endl;
        return;
    }

    std::vector<cv::Point> ifacet;
    std::vector<std::vector<cv::Point>> ifacets(1);

    // Disegna le faccette
    for (size_t i = 0; i < facets.size(); ++i)
    {
        ifacet.clear();
        bool all_inside = true;

        // Converti e verifica punti
        for (size_t j = 0; j < facets[i].size(); ++j)
        {
            cv::Point pt(cvRound(facets[i][j].x), cvRound(facets[i][j].y));
            ifacet.push_back(pt);
            if (!rect.contains(pt))
            {
                all_inside = false;
                break;
            }
        }

        // Disegna solo faccette valide
        if (all_inside)
        {
            ifacets[0] = ifacet;
            cv::polylines(current, ifacets, true, color, 1, cv::LINE_AA, 0); // Contorno faccetta
            cv::circle(current, centers[i], 3, color, cv::FILLED, cv::LINE_AA, 0); // Centro della cella
        }
    }
}
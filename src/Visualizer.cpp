#include "Visualizer.h"

Visualizer::Visualizer() {}

void Visualizer::drawDelaunay(const cv::Mat &current, std::vector<cv::Point2f> &good_new, const cv::Scalar& color)
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

void Visualizer::drawVoronoi(const cv::Mat &current, std::vector<cv::Point2f> &good_new, const cv::Scalar& color)
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
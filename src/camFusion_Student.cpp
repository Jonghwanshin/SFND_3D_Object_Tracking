
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); 
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0); 

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. 
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    std::vector<double> distances;
    std::vector<cv::DMatch> candidateKptMatch;
    double sum = 0.0f;
    for(auto match: kptMatches) 
    {
        cv::KeyPoint kpt_prev = kptsPrev[match.queryIdx];
        cv::KeyPoint kpt_curr = kptsCurr[match.trainIdx];

        if(boundingBox.roi.contains(kpt_curr.pt))
        {
            double dist = cv::norm(kpt_curr.pt - kpt_prev.pt);
            distances.push_back(dist);
            candidateKptMatch.push_back(match);
            sum += dist;
        }
    }
    // get mean and std dev
    double mean = sum / distances.size();
    std::vector<double> diff(distances.size());
    std::transform(distances.begin(), distances.end(), diff.begin(),
                std::bind2nd(std::minus<double>(), mean));
    double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / distances.size());

    for(auto match : candidateKptMatch)
    {
        cv::KeyPoint kpt_prev = kptsPrev[match.queryIdx];
        cv::KeyPoint kpt_curr = kptsCurr[match.trainIdx];
        double dist = cv::norm(kpt_curr.pt - kpt_prev.pt);
        if(abs(dist - mean) < 2*stdev)
        {
            boundingBox.kptMatches.push_back(match);
        }
    }
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    vector<double> distRatios;
    double minDist = 100.0;

    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    {
        cv::KeyPoint kpCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = it1; it2 != kptMatches.end(); ++it2)
        {
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            double distCurr = cv::norm(kpCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            {
                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } 
    }

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    // compute median dist. ratio to remove outlier influence
    std::sort(distRatios.begin(), distRatios.end());
    double medDistRatio = distRatios[distRatios.size()/2];

    double dT = 1 / frameRate;
    TTC = -dT / (1 - medDistRatio);
}

void getMeanStdDev(std::vector<LidarPoint> & lidarPoints, double& mean, double& std_dev)
{
    double sum = 0.0f;
    std::accumulate(lidarPoints.begin(), lidarPoints.end(), 0, [](const int& a, LidarPoint& p)
    {
        return a + p.x; 
    });

    // get mean and std dev
    mean = sum / lidarPoints.size();
    std::vector<double> diff(lidarPoints.size());
    for(auto p : lidarPoints)
    {
        diff.push_back(p.x - mean);
    }
    double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    std_dev = std::sqrt(sq_sum / diff.size());
}

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // auxiliary variables
    double dT = 1 / frameRate;        // time between two measurements in seconds
    double deltaX = 1 / frameRate;

    // find mean and std.dev from lidar measurements
    double mean_prev = 0.0f, stddev_prev = 0.0f;
    double mean_curr = 0.0f, stddev_curr = 0.0f;
    getMeanStdDev(lidarPointsPrev, mean_prev, stddev_prev);
    getMeanStdDev(lidarPointsCurr, mean_curr, stddev_curr);

    // find inliers which resides in 3 * std_dev
    std::vector<double> inliers_prev;
    std::vector<double> inliers_curr;

    for (auto p : lidarPointsPrev)
    {
        if(abs(p.x - mean_prev) < 3 * stddev_prev)
        {
            inliers_prev.push_back(p.x);
        }
    }

    for (auto p : lidarPointsCurr)
    {
        if(abs(p.x - mean_curr) < 3 * stddev_curr)
        {
            inliers_curr.push_back(p.x);
        }
    }

    // get mean value of inliers
    double d0 = 1e9, d1 = 1e9;
    d0 = std::accumulate(inliers_prev.begin(), inliers_prev.end(), 0.0) / inliers_prev.size();
    d1 = std::accumulate(inliers_curr.begin(), inliers_curr.end(), 0.0) / inliers_curr.size();

    // Using the constant-velocity model 
    TTC = d1 * dT / (d0 - d1);
}



void matchBoundingBoxes(
    std::vector<cv::DMatch> &matches, 
    std::map<int, int> &bbBestMatches, 
    DataFrame &prevFrame, DataFrame &currFrame)
{
    std::map<int, std::map<int, int>> matchCounts {};
    std::vector<int> maxMatchIdx(currFrame.boundingBoxes.size(), 0);
    std::vector<int> maxMatchCounts(currFrame.boundingBoxes.size(), 0);
    std::vector<bool> isUsed(prevFrame.boundingBoxes.size());

    // Associate keypoints in matches with bounding boxes
    for (auto match: matches) 
    {
        cv::KeyPoint prevKp = prevFrame.keypoints[match.queryIdx];
        cv::KeyPoint currKp = currFrame.keypoints[match.trainIdx];

        int prevBoxId = -1, currBoxId = -1;

        //find matching bbox ids from previous frame
        for (auto bbox: prevFrame.boundingBoxes) 
        {
            if(bbox.roi.contains(prevKp.pt)) {
                prevBoxId = bbox.boxID;
            }
        }
        //find matching bbox ids from current frame
        for (auto bbox: currFrame.boundingBoxes) 
        {
            if(bbox.roi.contains(currKp.pt)) {
                currBoxId = bbox.boxID;
            }
        }
        
        if((prevBoxId != -1) && (currBoxId != -1)) 
        {
            if(matchCounts.find(currBoxId) == matchCounts.end())
            {
                matchCounts.insert({prevBoxId, std::map<int, int>()});
                matchCounts[currBoxId][prevBoxId] = 0;
            }
            else
            {
                if(matchCounts[currBoxId].find(prevBoxId) == matchCounts[currBoxId].end())
                {
                    matchCounts[currBoxId][prevBoxId] = 0;
                }
            }
            matchCounts[currBoxId][prevBoxId] += 1;
            int count = matchCounts[currBoxId][prevBoxId];
            if(count > maxMatchCounts[currBoxId]) {
                maxMatchCounts[currBoxId] = count;
                maxMatchIdx[currBoxId] = prevBoxId;
            }
        }
    }

    for(int boxId = 0; boxId < currFrame.boundingBoxes.size(); ++boxId)
    {
        if((maxMatchCounts[boxId] != 0) && (!isUsed[maxMatchIdx[boxId]]))
        {
            bbBestMatches.insert({boxId, maxMatchIdx[boxId]});
            isUsed[maxMatchIdx[boxId]] = true;
        }    
    }
}

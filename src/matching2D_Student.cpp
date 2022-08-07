#include <numeric>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType == "MAT_BF")
    {
        int normType = (descriptorType.compare("DES_BINARY") == 0) ? cv::NORM_HAMMING : cv::NORM_L2;
        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType == "MAT_FLANN")
    {
        if(descriptorType == "DES_BINARY")
        {
            matcher = cv::FlannBasedMatcher::create();
        }
        else if(descriptorType == "DES_HOG")
        {
            matcher = cv::FlannBasedMatcher::create();
        }
        else
        {
            throw invalid_argument("descriptorType Invalid, Available Matchers: DES_BINARY, DES_HOG");
        }
    } 
    else
    {
        throw invalid_argument("matcherType Invalid, Available Matchers: MAT_BF, MAT_FLANN");
    }

    // perform matching task
    if (selectorType == "SEL_NN")
    { // nearest neighbor (best match)

        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
    }
    else if (selectorType == "SEL_KNN")
    { 
        // k nearest neighbors (k=2)
        vector<vector<cv::DMatch>> knn_matches;
        matcher->knnMatch(descSource, descRef, knn_matches, 2);

        // added descriptor distance ratio test
        double minDescDistRatio = 0.8;
        for (auto it = knn_matches.begin(); it != knn_matches.end(); ++it)
        {

            if ((*it)[0].distance < minDescDistRatio * (*it)[1].distance)
            {
                matches.push_back((*it)[0]);
            }
        }
    }
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType == "BRISK")
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if (descriptorType == "BRIEF")
    {
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    }
    else if (descriptorType == "FREAK")
    {
        extractor = cv::xfeatures2d::FREAK::create();
    }
    else if (descriptorType == "ORB")
    {
        extractor = cv::ORB::create();
    }
    else if (descriptorType == "AKAZE")
    {
        extractor = cv::AKAZE::create();
    }
    else if (descriptorType == "SIFT")
    {
        extractor = cv::SIFT::create();
    }
    else
    {
        throw invalid_argument("Descriptor Name" + descriptorType + " Invalid, Available Detectors: BRIEF, FREAK, BRISK, ORB, AKAZE, SIFT");
    }

    extractor->compute(img, keypoints, descriptors);
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {
        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }

    // visualize results
    if (bVis)
    {
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
     // Detector parameters
    int blockSize = 2; // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 3; // aperture parameter for Sobel operator (must be odd)
    int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    double maxOverlap = 0.0;
    double k = 0.04; // Harris parameter (see equation for details)

    // Detect Harris corners and normalize output
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1 );
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT ); 
    cv::normalize( dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );
    cv::convertScaleAbs( dst_norm, dst_norm_scaled );
    
    for (int r = 0; r < dst_norm.rows; r++) // rows
    {
        for (int c = 0; c < dst_norm.cols; c++) // cols
        {
            // loop over all pixels within sliding window around the current pixel 
            int response = (int)round(dst_norm.at<float>(r, c));
            if(response < minResponse) continue;

            cv::KeyPoint kp(cv::Point2f(c, r), 2 * apertureSize, response);

            bool isOverlapped = false;
            for (auto& kpCmp : keypoints)
            {
                if(cv::KeyPoint::overlap(kp, kpCmp) > maxOverlap)
                {
                    isOverlapped = true;
                    if(kp.response > kpCmp.response)
                    {
                        kpCmp = kp;
                        break;
                    }
                }
            }

            if (!isOverlapped)
                keypoints.push_back(kp);
        }
    }
}

void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis)
{
    cv::Ptr<cv::FeatureDetector> extractor;

    if (detectorType == "FAST")
    {
        extractor = cv::FastFeatureDetector::create();
    }
    else if (detectorType == "BRISK")
    {
        extractor = cv::BRISK::create();
    }
    else if (detectorType == "ORB")
    {
        extractor = cv::ORB::create();
    }
    else if (detectorType == "AKAZE")
    {
        extractor = cv::AKAZE::create();
    }
    else if (detectorType == "SIFT")
    {
        extractor = cv::SIFT::create();
    }
    else
    {
        throw invalid_argument("Detector Name" + detectorType + "Invalid, Available Detectors: FAST, BRISK, ORB, AKAZE, SIFT");
    }

    extractor->detect(img, keypoints);

    if(bVis) 
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Keypoint Detection Results of " + detectorType + "Detector";
        cv::namedWindow(windowName);
        cv::imshow(windowName, visImage);
        cv::waitKey(0); // wait for key to be pressed
    }
}
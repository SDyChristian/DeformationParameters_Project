#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H

#include <opencv2/opencv.hpp>

// Function to process image, binarizing and applying morphological operations
cv::Mat ImageProcessing(cv::Mat Image);

// Function to obtain the skeleton of the image
cv::Mat Skeleton(cv::Mat img);

// Function to clean isolated points from the binary image
cv::Mat Clean(cv::Mat binImg);

// Function to find the root point in the binary image
void findRootPt(cv::Mat binImg, double& x, double& y);

// Function to find the top point in the skeleton
void findTopPt(int xr, int yr, int lgth, int xv[], int yv[], double& x, double& y);

#endif // IMAGE_PROCESSING_H

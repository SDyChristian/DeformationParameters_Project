#include "image_processing.h"

// Global variables for image processing
extern int ero_size; 
extern int dil_size; 
extern cv::Mat ROIimg;

// Routine to process the image, binarize it and perform morphological operations
cv::Mat ImageProcessing(cv::Mat Image)
{    
    // Create a structuring element for morphological operations
    cv::Mat open_element = getStructuringElement(cv::MORPH_ELLIPSE,
        cv::Size(2 * ero_size + 1, 2 * ero_size + 1),
        cv::Point(ero_size, ero_size));

    cv::Mat close_element = getStructuringElement(cv::MORPH_ELLIPSE,
        cv::Size(2 * dil_size + 1, 2 * dil_size + 1),
        cv::Point(dil_size, dil_size));

    cv::Mat binaryImg;         // Binary Image
    cv::Mat grayImg;           // Gray Scale Image

    // Convert the image from RGB to grayscale
    cv::cvtColor(Image, grayImg, cv::COLOR_RGB2GRAY);

    // Extract the region of interest (ROI)
    ROIimg = grayImg(cv::Rect(110, 35, 400, 270));

    // Binarize the image
    cv::threshold(ROIimg, binaryImg, 125, 255, 0);

    // Perform morphological operations to remove noise and fill gaps
    cv::morphologyEx(binaryImg, binaryImg, cv::MORPH_OPEN, open_element);
    cv::morphologyEx(binaryImg, binaryImg, cv::MORPH_CLOSE, close_element);

    return binaryImg;
}

// Routine to extract the skeleton of the binary image
cv::Mat Skeleton(cv::Mat img)
{
    cv::Mat skel(img.size(), CV_8UC1, cv::Scalar(0));
    cv::Mat temp;
    cv::Mat eroded;

    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
    bool done;		
    do
    {
        cv::erode(img, eroded, element);
        cv::dilate(eroded, temp, element); // temp = open(img)
        cv::subtract(img, temp, temp);
        cv::bitwise_or(skel, temp, skel);
        eroded.copyTo(img);

        done = (cv::countNonZero(img) == 0);
    } while (!done);

    return skel;
}

// Routine to clean isolated points from the image
cv::Mat Clean(cv::Mat binImg)
{
    // Define a structural element
    cv::Mat kernel(3, 3, CV_8UC1, cv::Scalar(0));
    kernel.at<uchar>(1, 1) = 255;

    // Variables for image comparison
    cv::Mat ROI;
    cv::Mat comparison;

    // Loop through the image to find isolated white pixels
    for (int i = 1; i < binImg.rows - 1; i++)
    {
        for (int j = 1; j < binImg.cols - 1; j++)
        {
            if (binImg.at<uchar>(i, j) == 255)
            {
                ROI = binImg(cv::Rect(j - 1, i - 1, 3, 3));
                cv::bitwise_xor(kernel, ROI, comparison); 

                if (cv::countNonZero(comparison) <= 1)
                    binImg.at<uchar>(i, j) = 0;
            }
        }
    }

    return binImg;
}

// Routine to find the root point of the skeleton
void findRootPt(cv::Mat binImg, double& x, double& y)
{
    int i = 1, j = 0, a = 0;
    while (a == 0) {
        for (j = 0; j < binImg.cols; j++)
        {
            if (binImg.at<uchar>(binImg.rows - i, j) == 255)
            {
                x = j;
                y = binImg.rows - i;
                a = 1;
                break;
            }
        }
        i++;
    }
}

// Routine to find the top point of the skeleton
void findTopPt(int xr, int yr, int lgth, int xv[], int yv[], double& x, double& y)
{
    int i = 0;
    double dist_past = 0, dist_current = 0;

    for (i = 0; i < lgth; i++)
    {
        dist_current = sqrt(pow(xv[i] - xr, 2) + pow(yv[i] - yr, 2));
        if (dist_current > dist_past)
        {
            x = xv[i];
            y = yv[i];
            dist_past = dist_current;
        }
    }
}

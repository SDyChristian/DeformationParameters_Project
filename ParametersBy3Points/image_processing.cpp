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

// Routine to find the end point
void endPoint(cv::Mat img, double& xt, double& yt, double& xm, double& ym)
{
  // Matrix to save the color space [hsv] and matrix for Saturation [S] channel 
  cv::Mat hsv, S, H;
  // Vector to save the three channels
  std::vector<cv::Mat> ch;
  // Vector to save the contour
  std::vector<std::vector<cv::Point>> contours;  
  
  // Convert RBG to HSV
  cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
  // Region of interes
  hsv= hsv(cv::Rect(110,35,430,270));
  // Split image into channels
  cv::split(hsv,ch);
  // Apply threshold to Saturation channel to detect colors (the robot and background are in grayscale)
  cv::threshold(ch[1], S, 140, 255, 0);
  //S = Clean(S,3);
  
  // Apply morph. operations to clean the image
  cv::Mat Kernel = getStructuringElement(cv::MORPH_ELLIPSE,
      cv::Size(2 * 5 + 1, 2 * 5 + 1),
      cv::Point(3, 3) );
  //cv::morphologyEx(S,S,cv::MORPH_OPEN, Kernel);
  cv::morphologyEx(S,S,cv::MORPH_CLOSE, Kernel);
  
  for(int i=0; i<S.rows; i++)
	  {
	    for(int j=0; j<S.cols; j++)
	    {
	      // Filter
	      S_f.at<uchar>(i,j) = S_f.at<uchar>(i,j)-.5*(S_f.at<uchar>(i,j)-S.at<uchar>(i,j)); 
	      // Set threshold
	      if(S_f.at<uchar>(i,j)<=215)
	      {
		S.at<uchar>(i,j)=0;
	      }
	    }
	  }
  
  // Find the contours on images  
  cv::findContours(S,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
  
  std::vector<cv::Point2f> center(contours.size());                       // Radio
  std::vector<std::vector<cv::Point> > contours_poly( contours.size() );  // Poly
  std::vector<float>radius( contours.size() );                            // Radius of circunference that encloses contour 
  
  // Get the centroidal point of the robot's cap
  int ind1=0, ind2=0;
  for( size_t i = 0; i < contours.size(); i++ )
  {
    cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
    cv::minEnclosingCircle( contours_poly[i], center[i], radius[i] );
    double area = cv::contourArea(contours[i]);
    if(area>400)
      ind1 = i;
    if(area>150 && area<300)
      ind2 = i;
  }    
  cv::imshow("SATURATION", S);
  //cv::imshow("SATURATION FILTERED",S_f);
  if(ind1==0){
    xt=0; yt=0;}
  else{
  xt = center[ind1].x;
  yt = center[ind1].y;}
  
  if(ind1==0){
    xm=0; ym=0;}
  else{
  xm = center[ind2].x;
  ym = center[ind2].y;}
}



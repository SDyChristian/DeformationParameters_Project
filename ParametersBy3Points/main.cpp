#include <iostream>
#include <stdio.h>
#include <sstream>
#include <unistd.h>     // Add delays usleep(microseconds)
#include <cmath>
#include <math.h>
#include <vector>
#include <fstream>  
#include "realsense.h"
#include "image_processing.h"  // Include the header file for image processing functions

// Global variables for image processing
cv::Mat ROIimg;             
int ero_size = 2; 
int dil_size = 4; 
// The robot measure 9.5 cm == 0.095 m and is contained in 222 pixels 
double factor = .00042792792; // relation between cm and pixels [m/pix]
double tapa_length = 66; // pixels
// Root point   
yroot=269;  xroot=202; // root point is fixed 

void applyFilter(double x, double& x_f, alpha) {
	// alpha = filtering coeficient 
    x_f = x_f - alpha * (x_f - x);  // Apply filter
}

void computeRadius(double xtop_f, double ytop_f, double xmid_f, double ymid_f, double &center_xf, double &center_yf, double &radius_f) {
                   	
    // Calculate the determinant B
    double B = xtop_f * (ymid_f - yroot) + xmid_f * (yroot - ytop_f) + xroot * (ytop_f - ymid_f);

    // square norm of s
    double s1n2 = pow(xtop_f, 2) + pow(ytop_f, 2);
    double s2n2 = pow(xmid_f, 2) + pow(ymid_f, 2);
    double s3n2 = pow(xroot, 2) + pow(yroot, 2);

    // Circumcenter's coordinates
    double center_x = (1 / (2 * B)) * ((ymid_f - yroot) * s1n2 + (yroot - ytop_f) * s2n2 + (ytop_f - ymid_f) * s3n2);
    double center_y = (1 / (2 * B)) * ((xroot - xmid_f) * s1n2 + (xtop_f - xroot) * s2n2 + (xmid_f - xtop_f) * s3n2);

    // If the center coordinates are NaN, use the previous valid values
    if (std::isnan(center_x)) {
        center_x = center_xf;
    }
    if (std::isnan(center_y)) {
        center_y = center_yf;
    }

    // Apply filter to center coordinates
    center_xf = center_xf - 0.1 * (center_xf - center_x);
    center_yf = center_yf - 0.1 * (center_yf - center_y);

    // Calculate the curvature radius
    double radius = sqrt(pow(center_x - xroot, 2) + pow(center_y - yroot, 2));

    // If the radius is NaN, use the previous valid value
    if (std::isnan(radius)) {
        radius = radius_f;
    }

    // Apply filter to the radius
    radius_f = radius_f - 0.1 * (radius_f - radius);
}

cv::Mat drawParameters(cv::Mat colorImg, double xtop_f, double ytop_f, double xmid_f, double ymid_f, double xroot_f, double yroot, 
                      double center_x, double center_y, double radius, double kappa, double l_fil, double factor) {
    // Draw points
    cv::circle(colorImg, cv::Point(110 + xtop_f, 35 + ytop_f), 5, cv::Scalar(0, 0, 0), 3); 
    cv::circle(colorImg, cv::Point(110 + xmid_f, 35 + ymid_f), 5, cv::Scalar(0, 0, 0), 3); 
    cv::circle(colorImg, cv::Point(110 + xroot_f, 35 + yroot), 5, cv::Scalar(0, 0, 0), 3); 
    cv::circle(colorImg, cv::Point(110 + center_x, 35 + center_y), 3, cv::Scalar(255, 255, 255), 1); 
    cv::circle(colorImg, cv::Point(110 + center_x, 35 + center_y), radius, cv::Scalar(255, 255, 255), 1); 

    // Legends
    std::string legend1 = "Curvature = " + std::to_string(kappa * factor) + " 1/m";
    std::string legend2 = "Arc length = " + std::to_string(l_fil * factor) + " m";
    
    // Text
    cv::putText(colorImg, legend1, cv::Point(225, 380), cv::FONT_HERSHEY_DUPLEX, 0.37, cv::Scalar(0, 255, 0), 1);
    cv::putText(colorImg, legend2, cv::Point(225, 400), cv::FONT_HERSHEY_DUPLEX, 0.37, cv::Scalar(0, 255, 0), 1);

    return colorImg;
}

int main( int argc, char* argv[] )
{ 
    
    cv::Mat colorImg;	       // Color Image
    cv::Mat segment;           // Segment binary image
    cv::Mat backbone;          // Backbone image
    cv::Mat save;
    
    double xmid = 0, ymid = 0, xtop = 0, ytop = 0;          // mid and top points coordinates
    double xmid_f = 0, ymid_f = 0, xtop_f = 0, ytop_f = 0;  // filtered mid and top points coordinates
    double l = 0, l_fil = 0;                                // arc length variables ( l current and l filtered)
    double radius_f = 0, center_xf = 0, center_yf = 0;
    float cont = 0, P1 = 0, P2 = 0, P3 = 0;
    int flag = 0, counter = 675;
    float jj = 4;
    double ts = 0, time = 0;
    std::ofstream outfile ("test.txt");
    outfile <<"Sample"<<"\t" <<"P1 [PSI]" << "\t" << "P2 [PSI]" << "\t" << "P3 [PSI]" << "\t" << "Arc length [PIX]"<< "\t" << "Curvature [1/PIX]" << std::endl;
    
    try{
	// Call class realsense
        RealSense realsense;
	sleep(1);
	while(1){
	  //double t = (double)cv::getTickCount();
	  int i=0, j=0, k=0;
	  
	  // Save the RGB image captured by D435 
	  colorImg = realsense.GetImage(0);
	  save = colorImg;
	  
	  // Image Processing 
	  segment = ImageProcessing(colorImg);
	  cv::imshow("Segment", segment);
	  
	  // Get the skeleton from 
	  backbone = Skeleton(segment);
	  backbone = Clean(backbone,1);
	  
	  // Vector to save the contour
	  std::vector<std::vector<cv::Point>> contours;  
	  // Find the contours on images  
	  cv::findContours(ImageProcessing(colorImg), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	  
	  std::vector<cv::Point2f> center(contours.size());                       // Radio
	  std::vector<std::vector<cv::Point> > contours_poly( contours.size() );  // Poly
	  std::vector<float>r( contours.size() );                                 // Radius of circunference that encloses contour 
	  float area;
	  float perimeter; 
	  
	  int ind;
 	  for( size_t i = 0; i < contours.size(); i++ )
 	  {
 	    area = cv::contourArea(contours[i]);
	    if(area>10000)
	      ind = i;
 	  }
 	  // Binary operations to compute perimeter
 	  cv::approxPolyDP( cv::Mat(contours[ind]), contours_poly[ind], 3, true );
 	  cv::minEnclosingCircle( contours_poly[ind], center[ind], r[ind] );
	  perimeter = cv::arcLength(contours[ind],true);
	  
      // Compute robot length
	  l = (perimeter-2*tapa_length)/2;
      applyFilter(l, l_fill, 0.1); // Apply filter to ytop
	  
	  /////////// COMPUTE MID & TOP POINTS //////////////////
	  
	  // Find top and mid point
	  endPoint(colorImg, xtop, ytop, xmid, ymid);
	  // Filtering
	  applyFilter(ytop, ytop_f, 0.1); // Apply filter to ytop
	  applyFilter(xtop, xtop_f, 0.1); // Apply filter to xtop
	  applyFilter(xmid, xmid_f, 0.1); // Apply filter to xmid
	  applyFilter(ymid, ymid_f, 0.1); // Apply filter to ymid
	  
	  computeRadius(xtop_f, ytop_f, xmid_f, ymid_f, &center_xf, &center_yf, &radius_f)
      double kappa = 1/radius_f; // Compute Curvature
      
	  /////////////////////Plot poitns and deformation parameters//////////////////////

 	  colorImg drawParameters(colorImg, xtop_f, ytop_f, xmid_f, ymid_f, xroot_f, yroot, center_x, center_y, radius, kappa, l_fil, factor)
	  cv::imshow("raduis",colorImg);   
	  std::cout<<l_fil<<"\t"<<kappa<<"\n";
	  
	  ///////// End Visualization//////////
	  if(xtop_f>xroot){
	    flag = 1;
	  }
	  if(xtop_f<xroot){
	    flag = -1;
	  }
	  if(xtop_f==xroot){
	    flag = 0;
	  }
	  // Save data //
	  const int32_t key = cv::waitKey( 10 );
	  if( key == 'a' ){
	    if(cont>13.5){
	      cont=0;
	      jj=jj+.2;
	    }
	    if(cont<=6){
	      P1 = cont;
	      P2 = cont;
	      P3 = cont;}
	    else{
	      P1 = cont-6;
	      P2 = jj;
	      P3 = jj;}
	     
	    outfile <<counter<<"\t" <<P1 << "\t" << P2 << "\t" << P3 << "\t" << l_fil<< "\t" << kappa*flag <<std::endl;
	    std::cout<<"data saved"<<"\n \n";
	    std::string x_str = std::to_string(counter) + ".png";
	    cv::imwrite(x_str,save);
	    cont = cont + 0.1;
	    counter = counter +1;
	    
	  }
	  if( key == 'q' ){
            break;
	  }
	  
	  /////////////////////////////////////
	}
    } catch( std::exception& ex ){
        std::cout << ex.what() << std::endl;
    }
    outfile.close();
    return 0;
}






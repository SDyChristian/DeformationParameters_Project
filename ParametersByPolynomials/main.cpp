#include <iostream>
#include<iomanip>
#include <stdio.h>
#include <sstream>
#include <unistd.h>     // Add delays usleep(microseconds)
#include <cmath>
#include <math.h>
#include <vector>
#include "realsense.h"
#include "image_processing.h"  // Include the header file for image processing functions
#include "matplotlibcpp.h"  // Include for plotting

namespace plt = matplotlibcpp;

void PolyFitting(int, int, double&, double&, double&); // Polynomial Fit

int main( int argc, char* argv[] )
{ 
    cv::Mat colorImg;               // Color Image
    cv::Mat segment;
    cv::Mat backbone;
    cv::Mat test;
    double c1, c2, c3, c4; 
    double xroot, yroot, xtop, ytop;
    int n = 100;                    // Length of samples for s
    std::vector<double> s(n);

    try {
        // Initialize RealSense object
        RealSense realsense;

        while (1) {
            int i = 0, j = 0, k = 0;
            
            // Capture the RGB image from the camera
            colorImg = realsense.GetImage(0);

            // Image Processing 
            segment = ImageProcessing(colorImg);
            cv::imshow("Binary Image", segment);

            // Get the skeleton from the segment
            backbone = Skeleton(segment);
            backbone = Clean(backbone);

            cv::imshow("Backbone", backbone);

            // Find root point
            findRootPt(backbone, xroot, yroot);

            // Save points for further processing
            int length = cv::countNonZero(backbone);
            int x[length], y[length];

            // Iterate through the image
            for (i = 0; i < backbone.rows; i++) {
                for (j = 0; j < backbone.cols; j++) {
                    if (backbone.at<uchar>(i, j) == 255) {
                        y[k] = i;
                        x[k] = j;
                        k++;
                    }
                }
            }

            // Find the top point
            findTopPt(xroot, yroot, length, x, y, xtop, ytop);

            // Curve Fitting for X axis
            PolyFitting(x, y, 3, length, c1, c2, c3, c4);
            test = backbone; // To rewrite over original image

            // Apply curve fitting to the image
            for (k = xroot; k < xtop; k++) {
                int val = c1 + k * c2 + k * k * c3 + k * k * k * c4;
                if (val > 269)
                    val = 269;
                if (val < 0)
                    val = 0;
                test.at<uchar>(val, k) = 125;
            }

            // Show the curve fitting result
            cv::imshow("Curve Fitting", test);

            // Compute curvature and arc length
            double dx = (xtop - xroot) / n;   // differential of x
            double factor = 0.039408867;      // conversion factor (cm to pixels)
            double kappa[n];                  // Curvature vector

            double lpast = 0, lcurrent;

            // Numerical integration for arc length
            for (k = 0; k < n; k++) {
                double X = (xroot + k * dx);  // X position on the curve

                // Compute the first and second derivatives of the polynomial
                double first_der = c2 + 2 * X * c3 + 3 * X * X * c4;
                double second_der = 2 * c3 + 6 * X * c4;

                // Compute and save the curvature
                kappa[k] = (second_der / pow(1 + first_der * first_der, 1.5)) * factor;
                s[k] = kappa[k];

                // Arc length calculation using numerical integration
                double f = sqrt(1 + pow(c2 + 2 * c3 * X + 3 * c4 * X * X, 2));
                lcurrent = lpast + f * dx;
                lpast = lcurrent;
            }

            double l = lcurrent * factor;
            std::cout << l << "\n";

            const int32_t key = cv::waitKey(10);
            if (key == 'q') {
                break;
            }
        }
    } catch (std::exception& ex) {
        std::cout << ex.what() << std::endl;
    }

    // Plot the curvature as a curve
    plt::plot(s);
    plt::show();

    return 0;
}

void PolyFitting(int x[], int y[], int n, int N, double& a1, double& a2, double& a3, double& a4)
{
    int i,j,k;
            
    // n is the degree of Polynomial 
    double X[2*n+1];                        //Array that will store the values of sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
    for (i=0;i<2*n+1;i++)
    {
        X[i]=0;
        for (j=0;j<N;j++)
            X[i]=X[i]+pow(x[j],i);        //consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
    }
    double B[n+1][n+2],a[n+1];            //B is the Normal matrix(augmented) that will store the equations, 'a' is for value of the final coefficients
    for (i=0;i<=n;i++)
        for (j=0;j<=n;j++)
            B[i][j]=X[i+j];            //Build the Normal matrix by storing the corresponding coefficients at the right positions except the last column of the matrix
    double Y[n+1];                    //Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
    for (i=0;i<n+1;i++)
    {    
        Y[i]=0;
        for (j=0;j<N;j++)
        Y[i]=Y[i]+pow(x[j],i)*y[j];        //consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
    }
    for (i=0;i<=n;i++)
        B[i][n+1]=Y[i];                //load the values of Y as the last column of B(Normal Matrix but augmented)
    n=n+1;                //n is made n+1 because the Gaussian Elimination part below was for n equations, but here n is the degree of polynomial and for n degree we get n+1 equations
   
    
    for (i=0;i<n;i++)                    //From now Gaussian Elimination starts(can be ignored) to solve the set of linear equations (Pivotisation)
        for (k=i+1;k<n;k++)
            if (B[i][i]<B[k][i])
                for (j=0;j<=n;j++)
                {
                    double temp=B[i][j];
                    B[i][j]=B[k][j];
                    B[k][j]=temp;
                }
    
    for (i=0;i<n-1;i++)            //loop to perform the gauss elimination
        for (k=i+1;k<n;k++)
            {
                double t=B[k][i]/B[i][i];
                for (j=0;j<=n;j++)
                    B[k][j]=B[k][j]-t*B[i][j];    //make the elements below the pivot elements equal to zero or elimnate the variables
            }
    for (i=n-1;i>=0;i--)                //back-substitution
    {                        //x is an array whose values correspond to the values of x,y,z..
        a[i]=B[i][n];                //make the variable to be calculated equal to the rhs of the last equation
        for (j=0;j<n;j++)
            if (j!=i)            //then subtract all the lhs values except the coefficient of the variable whose value                                   is being calculated
                a[i]=a[i]-B[i][j]*a[j];
        a[i]=a[i]/B[i][i];            //now finally divide the rhs by the coefficient of the variable to be calculated
    }
    a1 = a[0];
    a2 = a[1];
    a3 = a[2];
    a4 = a[3]; 
}

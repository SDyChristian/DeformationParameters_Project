#ifndef __REALSENSE__
#define __REALSENSE__

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

class RealSense
{
private:
    // RealSense
    rs2::pipeline pipeline;
    rs2::pipeline_profile pipeline_profile;
    rs2::frameset frameset;

    // Color Buffer
    rs2::frame color_frame;
    cv::Mat color_mat;
    uint32_t color_width = 640;
    uint32_t color_height = 480;
    uint32_t color_fps = 30;

    // Depth Buffer
    rs2::frame depth_frame;
    cv::Mat depth_mat;
    uint32_t depth_width = 640;
    uint32_t depth_height = 480;
    uint32_t depth_fps = 30;

public:
    // Constructor
    RealSense();

    // Destructor
    ~RealSense();

    // Processing
    cv::Mat GetImage(int image);

private:
    // Initialize
    void initialize();

    // Initialize Sensor
    inline void initializeSensor();

    // Finalize
    void finalize();

    // Update Data
    void update();

    // Update Frame
    inline void updateFrame();

    // Update Color
    inline void updateColor();

    // Update Depth
    inline void updateDepth();


    // Parallel

    // Draw Data
    void draw();

    // Draw Color
    inline void drawColor();

    // Draw Depth
    inline void drawDepth();

};

#endif // __REALSENSE__

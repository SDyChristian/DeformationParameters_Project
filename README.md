<div align="center">
 
# Deformation Parameters of a Soft Robot

<img alt="Soft Robot Setup" src="https://github.com/SDyChristian/DeformationParameters_Project/blob/main/Images/SetUp.png" width="600" />

</div>

In this repo you will C++ programs to drive proportional valves to actuate a soft robot and to estimate its deformation parameters. 
 - The complete setup is composed of a [soft robot](https://www.youtube.com/watch?v=eaKgIXw65vs) (manufactured by me), a camera sensor [Realsense D435](https://www.intelrealsense.com/depth-camera-d435/), a [Phidget 1002](https://www.phidgets.com/?prodid=1018#Using_Your_Own_Program) module (a device to command analog output signals) and a set of three [proportional valves](https://www.festo.com/tw/en/a/557776/).
 - Deformation parameters are robot arc length and curvature.
 - After an image processing using OpenCV libraries, deformation parameters are computed using [three points](https://github.com/SDyChristian/DeformationParameters_Project/tree/main/ParametersBy3Points) over robot's backbone or using [polynomials](https://github.com/SDyChristian/DeformationParameters_Project/tree/main/ParametersByPolynomials). 

<div align="center">

Click on the image to watch the video
<br> 

[<img alt="Deformation Parameters" src="https://github.com/SDyChristian/DeformationParameters_Project/blob/main/Images/Def_Parameters.PNG" width="400" />](https://www.youtube.com/watch?v=g2lw66Rl5O8)

</div>



## Installing software and dependencies
 - [Phidget Libraries](https://www.phidgets.com/docs/OS_-_Windows), Start with [Phidget](https://www.phidgets.com/docs/Phidget_Programming_Basics)
 - [OpenCV Instalation](https://docs.opencv.org/4.x/df/d65/tutorial_table_of_content_introduction.html)
 - [Realsense D435 SDK](https://www.intelrealsense.com/sdk-2/)




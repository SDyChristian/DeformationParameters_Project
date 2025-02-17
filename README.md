<div align="center">
 
# Deformation Parameters of a Soft Robot

<img alt="Soft Robot Setup" src="https://github.com/SDyChristian/DeformationParameters_Project/blob/main/Images/SetUp.png" width="600" />

</div>

This repository contains C++ programs designed to control proportional valves for actuating a soft robot and estimating its deformation parameters.
 - The complete setup includes a [soft robot](https://www.youtube.com/watch?v=eaKgIXw65vs) (manufactured by me), a [Realsense D435](https://www.intelrealsense.com/depth-camera-d435/) camera sensor, a [Phidget 1002](https://www.phidgets.com/?prodid=1018#Using_Your_Own_Program) module (used to control analog output signals), and a set of three [proportional valves](https://www.festo.com/tw/en/a/557776/) to drive soft robot.
 - The deformation parameters of the robot, including arc length and curvature, are calculated after image processing with OpenCV libraries.
 - The deformation is estimated using [three key points](https://github.com/SDyChristian/DeformationParameters_Project/tree/main/ParametersBy3Points) along the robot's backbone or by applying [polynomials](https://github.com/SDyChristian/DeformationParameters_Project/tree/main/ParametersByPolynomials) models.

<div align="center">

Click on the image to watch the video
<br> 

[<img alt="Deformation Parameters" src="https://github.com/SDyChristian/DeformationParameters_Project/blob/main/Images/Def_Parameters.PNG" width="400" />](https://www.youtube.com/watch?v=g2lw66Rl5O8)

</div>



## Installing software and dependencies
 - [Phidget Libraries](https://www.phidgets.com/docs/OS_-_Windows), Start with [Phidget](https://www.phidgets.com/docs/Phidget_Programming_Basics)
 - [OpenCV Instalation](https://docs.opencv.org/4.x/df/d65/tutorial_table_of_content_introduction.html)
 - [Realsense D435 SDK](https://www.intelrealsense.com/sdk-2/)




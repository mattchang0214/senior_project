# Introduction
Our project this semester was to design a computer vision autonomous docking rover that can work in GPS denied areas. It uses computer vision to navigate a 1/10th scale rover to a dock signified by ArUco markers.

# Hardware Architecture
Our team used four main hardware component to complete our goal.
* Intel Sense D400 Stereo Camera
* Nvidia Jetson TX1
* Pixhawk 2.1 Cube
* Modified 1/10th scale Traxxas Rover

# Software Architecture
The software architecture for our project required us to integrate a variety of different open source modules. It was important that we 
had a solid understanding of each module to avoid compatibility issues. 

* ArUco Markers: An ArUco marker is a square marker made up of a wide black border and an inner binary matrix which determines its ID. The black border facilitates its fast detection in the image. The markers were used to identify the sides of the dock to allow the rover to align itself to the center of the dock. An additional marker at the back of the dock signals the rover to stop upon reaching a certain distance to the marker.
* OpenCV: OpenCV is an open source computer vision and machine learning software library. It is highly optimized with focus on real-time applications. It has C++, Python and Java interfaces and support Windows, Linux, etc. OpenCV functions were used in the real-time detection and identification of ArUco markers. Functions were also used to visualize the camera feed with bounding boxes indicating the position of the markers.
* ROS: Robot Operating System (ROS) is a collection of tools, libraries, and conventions that simplifies the task of creating robust robot behavior across various robotic platforms. ROS offers a message passing interface (middleware) that provides inter-process communication. ROS functions allowed our program to send velocity commands to the Pixhawk while receiving camera feed from RealSense processes. The tools were also used to log errors and warnings for analysis.
* QGroundControl: QGroundControl is an open source Ground Control Station that provides full flight control and mission planning for any vehicle that communicates using the MAVLink protocol. QGroundControl allowed us to adjust the parameters of the Extended Kalman Filter on the Pixhawk, which helped smoothen the movement of the rover. We were also able to enable input data signal from the wheel encoders to the filter on QGroundControl.
* ArduRover: ArduRover [5] is an open source autopilot software for guiding ground vehicles. It can receive both manually inputted commands from a remote or it can run fully autonomous using mission planning software. The navigation software runs on the Pixhawk and combines sensory data from various sources (compass, GPS, accelerometers, etc) to navigate the vehicle autonomously.

# Results
The rover’s docking port was represented with a box that is 0.5 inch larger than the rover on all sides. Two parallel posts represented the booms of the TF-2 airplane. ArUco markers on the posts aid in navigation while the one in the back of the dock signals the rover to stop. The rover started from a maximum distance of 5 meters from the location, with one or more markers clearly visible to the camera. The rover was able to autonomously navigate itself between the posts facing directly forward in under 20 seconds, 10 seconds better than the original goal of 30 seconds.

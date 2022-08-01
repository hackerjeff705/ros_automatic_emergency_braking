# ros_automatic_emergency_braking
C/C++ and python implementation of an automatic emergency breaking system in ROS.
The vehicle uses Turtlebot3 LDS-02 lidar to find the average distance of a range of points directly in front of the vehicle. If the vehicle is greater than the distance threshold, the velocity of the vehicle will come to a stop at a specified distance as it gets closer to the object.

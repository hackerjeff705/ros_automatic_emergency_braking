# ros_automatic_emergency_braking
C/C++ and python implementation of an automatic emergency breaking system in ROS.

The vehicle uses Turtlebot3 LDS-02 lidar to find the average distance of a range of points directly in front of the vehicle.

The vehicle slows down and comes to a complete stop if its current distance is below a given threshold.

#include <ros/ros.h>
#include <numeric>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;

// Initialize rate
int freqs = 10;

// Initalize Publisher
ros::Publisher pub;

// Initialize Twist message
geometry_msgs::Twist msg;

// Vehicle parameters
float obj_dist;
int ANGLE_RANGE = 360;
int STEERING_ANGLE = 0;
float DISTANCE_THRESHOLD = 0.2;
float VELOCITY = 0.05;
float TIME_THRESHOLD = 0.5;

// P-Controller Parameters
float kp_dist = 0.75; // initialize;
float kp_ttc = 0.5; // initialize;
float dist_error = 0.0;
float time_error = 0.0;

float get_average(std::vector<float> v) {
    if (v.empty()) {
        return 0;
    }

    float sum;
    float v_len = v.size();
    for (int i = 0; i < v_len; i++) {
    	sum += v[i];
    }
    return  sum / v_len;
}

vector<float> get_data(int angle, vector<float> data) {
	int ilen = data.size();
	float ipd = (float)ilen / (float)ANGLE_RANGE; // index per division
	int half_angle = angle / 2;
	int lwr_bound = (ipd * half_angle);
	int upr_bound = (ilen - lwr_bound);

	vector<float> idx_ranges;

	for (int i = 0; i < data.size(); i++) {
		if (i < lwr_bound || i > upr_bound) {
			idx_ranges.push_back(data[i]);
		}
	}

	return idx_ranges;
}

float get_dstance(const sensor_msgs::LaserScan::ConstPtr& msg) {
	// find average distance of a range of points directly in fornt of the vehicle
	int angle_front = 30; // Range of angle in the front of the vehicle we want to observe
	float avg_dist = 0.;

	// Get the corresponding list of indices for given range of angles
	vector<float> index_front = get_data(angle_front, msg->ranges);

	// Find avg range distance
	avg_dist = get_average(index_front);

	return avg_dist;
}

void dist_control(float distance) {
	float velocity;

	if (distance > DISTANCE_THRESHOLD) {
		if (distance <= 0.4) {
			dist_error = distance - DISTANCE_THRESHOLD;
			velocity = kp_dist * dist_error;
		} else {
			velocity = VELOCITY;
		}
	} else {
		velocity = 0.;
	}

	ROS_INFO("Distance to collision = %f, Vehicle velocity = %f", distance, velocity);

	msg.linear.x = velocity;
	msg.angular.z = STEERING_ANGLE;
	pub.publish(msg);
}

void counterCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    obj_dist = get_dstance(msg); 
}


// Main function
int main(int argc, char** argv) {
	// Initialize node
    ros::init(argc, argv, "automatic_emergemcy_breaking");
    ros::NodeHandle nh;

    // Initialize Subscriber
    ros::Subscriber sub = nh.subscribe("/scan", 1000, counterCallback);

    // Initizlize Publisher
    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);


    // Initialize rate
    ros::Rate rate(freqs);
    ros::spinOnce();

    while(ros::ok())
    {
    	cout << "distance = " << obj_dist << endl;
    	dist_control(obj_dist);

    	ros::spinOnce();
    	rate.sleep();
    }
    

    return 0;
}
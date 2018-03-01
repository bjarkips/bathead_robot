#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <numeric>
#include <math.h>
#include <ctime>
//#include <wiringPi.h>

class bathead_control_node
{
public:
	bathead_control_node(ros::NodeHandle);
	virtual ~bathead_control_node();
    void run();
private:
	// Enums
	enum Control {seek_goal, follow_left, follow_right};
	
	// Structs
	struct vec
	{
		double x;
		double y;
	};
	
	// ROS
	ros::NodeHandle nh;

	ros::Subscriber range_left_subscriber;
	void rangeLeftSubscriberCallback(const std_msgs::Float64::ConstPtr& msg);
    ros::Subscriber range_right_subscriber;
    void rangeRightSubscriberCallback(const std_msgs::Float64::ConstPtr& msg);
    ros::Subscriber pose_subscriber;
    void poseSubscriberCallback(const nav_msgs::Odometry::ConstPtr& msg);

	ros::Publisher cmd_vel_publisher;

	// Methods
	void land();
	static void toEulerAngle(double w, double x, double y, double z, double& roll, double& pitch, double& yaw);
	double vlength(vec v);
	double angleDiff(double a, double b);
	int sign(double x);

	// Attributes
    double range_left, range_right, range_max = 1.5,
    	integral_left = 0.0, integral_right = 0.0, integral_weight = .2, // TODO check
    	max_ang_vel = 1.5, max_lin_vel = .5;
	nav_msgs::Odometry odom;
	//vec goal = {7.64, -9.07}; // Goal in next room
	vec goal = {27.5, -17.5}; // Goal in top right corner
	//vec goal = {-2.5, -10.5}; // Goal in side room
	//vec goal = {-6.0, -4.0}; // Goal in same room
	//vec goal = {38.0, 11.5}; // Goal outside, top left
	// -17, -16: Outside, bottom right
	std::clock_t t_obstacle = std::clock();
	//int chirp_trigger_pin = 7;
	
	Control control_state = Control::seek_goal;
};

// Angle difference
double bathead_control_node::angleDiff(double a, double b)
{
    double dif = fmod(b - a + M_PI, M_PI*2);
    if (dif < 0)
        dif += M_PI*2;
    return dif - M_PI;
}

// Euclidean norm of euclidean vector (x,y)
double bathead_control_node::vlength(vec v)
{
	return sqrt( v.x * v.x + v.y * v.y );
}

void bathead_control_node::toEulerAngle(double w, double x, double y, double z, double& roll, double& pitch, double& yaw)
{
	// roll (x-axis rotation)
	double sinr = 2.0 * (w * x + y * z);
	double cosr = 1.0 - 2.0 * (x * x + y * y);
	roll = atan2(sinr, cosr);

	// pitch (y-axis rotation)
	double sinp = 2.0 * (w * y - z * x);
	if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny = 2.0 * (w * z + x * y);
	double cosy = 1.0 - 2.0 * (y * y + z * z);  
	yaw = atan2(siny, cosy);
}

bathead_control_node::bathead_control_node(ros::NodeHandle n)
{
    nh = n;

    range_left_subscriber = nh.subscribe<std_msgs::Float64>("/bathead/range/left", 1, &bathead_control_node::rangeLeftSubscriberCallback, this);
    range_right_subscriber = nh.subscribe<std_msgs::Float64>("/bathead/range/right", 1, &bathead_control_node::rangeRightSubscriberCallback, this);
    pose_subscriber = nh.subscribe<nav_msgs::Odometry>("/Pioneer3AT/odom", 1, &bathead_control_node::poseSubscriberCallback, this);

    cmd_vel_publisher = nh.advertise<geometry_msgs::Twist>("/Pioneer3AT/cmd_vel",1);
	    
}

// Sign of double
int bathead_control_node::sign(double x)
{
	return (int) ((x > 0) - (x < 0));
}

void bathead_control_node::rangeLeftSubscriberCallback(const std_msgs::Float64::ConstPtr& msg)
{
	range_left = (msg->data) / range_max;
}

void bathead_control_node::rangeRightSubscriberCallback(const std_msgs::Float64::ConstPtr& msg)
{
	range_right = (msg->data) / range_max;
}

void bathead_control_node::poseSubscriberCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	odom = *msg;
}

void bathead_control_node::run()
{
	if ( range_left < 0.01 || range_right < 0.01 ) {
		ROS_INFO("Waiting for sensor readings.");
	}
	else {
		
	geometry_msgs::Twist vel;
	vel.angular.z = 0.0;
	vel.linear.x = 0.0;

/// Integrate difference between max and current values to avoid walls and especially corners

	//if (range_left == 1.0) integral_left -= 10 * integral_weight;
	integral_left += (.95 - range_left) * integral_weight;
	if (integral_left < 0.0) integral_left = 0.0;

	//if (range_right == 1.0) integral_right -= 10 * integral_weight;
	integral_right += (.95 - range_right) * integral_weight;
	if (integral_right < 0.0) integral_right = 0.0;

	// Clamp integrals
	if (integral_left > 1.0) integral_left = 1.0;
	if (integral_right > 1.0) integral_right = 1.0;

/// Slow down if average of both ranges is low

	double range_avg = (range_left + range_right) / 2.0;
	double integral_avg = (integral_left + integral_right) / 2.0;

/// Use robot odometry and goal point to encourage turning

	vec v_robot_goal = {goal.x - odom.pose.pose.position.x, goal.y - odom.pose.pose.position.y};
	double roll, pitch, yaw;
	toEulerAngle(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, roll, pitch, yaw);
	double a_robot = yaw;
	double a_robot_goal = atan2( v_robot_goal.y, v_robot_goal.x );
	double angle_diff = angleDiff(a_robot, a_robot_goal);
	double angle_diff_norm = angle_diff / (M_PI / 2);

	double L = range_left, R = range_right, G = -angle_diff_norm, Tw = .85;

	ROS_INFO("L = %f R = %f G = %f", L, R, G);

/// Enter three-state machine
	switch ( control_state ) {
		case Control::seek_goal: 
			ROS_INFO("Seeking goal. Int_L = %f Int_R = %f", integral_left, integral_right);
			if ( .95 <= L && .95 <= R ) { 
				// Case #1: No obstacle, turn towards goal at maximum forward speed
				vel.angular.z = max_ang_vel * -angle_diff_norm * (1.0 - integral_avg); // TODO check
				vel.linear.x = max_lin_vel;
				break;
			}
			if ( L < .95 && G < 0.0 ) { 
				// Case #2: Obstacle and goal on left side, follow_left
				control_state = Control::follow_left;
			}
			if ( .95 <= L && R < .95 && G < 0.0 ) {
				// Case #3: Obstacle on right and goal on left, follow_right
				control_state = Control::follow_right;
			}
			if ( R < .95 && 0.0 <= G ) {
				// Case #4: Obstacle and goal on right side, follow_right
				control_state = Control::follow_right;
			}
			if ( L < .95 && .95 <= R && 0.0 <= G ) {
				// Case #5: Obstacle on left and goal on right, follow_left
				control_state = Control::follow_left;
			}
			if (control_state == Control::seek_goal ) {
				ROS_ERROR("UNKNOWN CASE FOR SEEK_GOAL");
				break;
			}
	
		case Control::follow_left:
			ROS_INFO("Following left. Int_L = %f Int_R = %f", integral_left, integral_right);
			if ( (Tw <= L && L < .95) && .95 <= R && G < 0.0 ) {
				// Case #1: Wall visible on left side, goal on left side, drive straight
				vel.angular.z = 0.0;
				vel.linear.x = max_lin_vel;
				break;
			}
			if ( L < .95 && R < .95 ) {
				// Case #2: Obstacle in front, turn right
				vel.angular.z = max_ang_vel * integral_avg;
				vel.linear.x = max_lin_vel; // TODO check
				break;
			}
			if ( L < Tw && .95 <= R && G < 0.0 ) {
				// Case #3: Wall too close on left side, turn right
				vel.angular.z = max_ang_vel * (1.0 - range_left);
				vel.linear.x = max_lin_vel; // TODO check
				break;
			}
			if ( .95 <= L && G < 0.0 ) {
				// Case #4: No wall on left side, turn left
				// TODO check that this properly avoids obstacles on the right
				vel.angular.z = -.9*max_ang_vel * (1.1 - integral_avg);
				vel.linear.x = max_lin_vel;
				break;
			}
			if ( L < .95 && .95 <= R && 0.0 <= G ) {
				// Case #5: Goal on right and wall on left, turn right
				vel.angular.z = max_ang_vel * (1.0 - range_left);
				vel.linear.x = max_lin_vel;
				break;
			}
			if ( .95 <= L && R < .95 && 0.0 <= G ) {
				// Case #6: No wall on left side, goal and obstacle on right, follow_right
				control_state = Control::follow_right;
				vel.angular.z = 0.0;
				vel.linear.x = 0.0;
				break;
			}
			if ( .95 <= L && .95 <= R && 0.0 <= G ) {
				// Case #7: No obstacle and goal on right, seek_goal
				control_state = Control::seek_goal;
				vel.angular.z = 0.0;
				vel.linear.x = max_lin_vel;
				break;
			}
			ROS_ERROR("UNKNOWN CASE FOR FOLLOW_LEFT");
			break;
	
		case Control::follow_right:
			ROS_INFO("Following right. Int_L = %f Int_R = %f", integral_left, integral_right);
			if ( .95 <= L && (Tw <= R && R < .95) && 0.0 <= G ) {
				// Case #1: Wall visible on right side, goal on right side, drive straight
				vel.angular.z = 0.0;
				vel.linear.x = max_lin_vel;
				break;
			}
			if ( L < .95 && R < .95) {
				// Case #2: Obstacle in front, turn left
				vel.angular.z = -max_ang_vel * integral_avg;
				vel.linear.x = max_lin_vel;
				break;
			}
			if ( .95 <= L && R < Tw && 0.0 <= G ) {
				// Case #3: Wall too close on right side, turn left
				vel.angular.z = -max_ang_vel * (1.0 - range_right);
				vel.linear.x = max_lin_vel;				
				break;
			}
			if ( .95 <= R && 0.0 <= G ) {
				// Case #4: No wall on right side, turn right
				vel.angular.z = .9*max_ang_vel * (1.1 - integral_avg);
				vel.linear.x = max_lin_vel;
				break;
			}
			if ( .95 <= L && R < .95 && G < 0.0 ) {
				// Case #5: Goal on left and wall on right, turn left
				vel.angular.z = -max_ang_vel * (1.0 - range_right);
				vel.linear.x = max_lin_vel;
				break;
			}
			if ( L < .95 && .95 <= R && G < 0.0 ) {
				// Case #6: No wall on right side, goal and obstacle on left, follow_left
				control_state = Control::follow_left;
				vel.angular.z = 0.0;
				vel.linear.x = 0.0;
				break;
			}
			if ( .95 <= L && .95 <= R && G < 0.0 ) {
				// Case #7: No obstacle and goal on left, seek_goal
				control_state = Control::seek_goal;
				vel.angular.z = 0.0;
				vel.linear.x = max_lin_vel;
				break;
			}
			ROS_ERROR("UNKNOWN CASE FOR FOLLOW_RIGHT");
			break;
		
		default:
			ROS_ERROR("INVALID CONTROL STATE");
			break;
	
	} // switch (control_state)
	
	// Calculate linear velocity
	double lin_vel = max_lin_vel * (1.0 - 1.5 * (1.0 - range_avg));
	vel.linear.x = lin_vel;

	// Correct angle
	vel.angular.z *= -1.0;

/// Publish twist command to pioneer robot
	vel.linear.y = 0.0;
	vel.linear.z = 0.0;
	vel.angular.x = 0.0;
	vel.angular.y = 0.0;
	cmd_vel_publisher.publish(vel);
	
	} // else
	/**
	
/// Turn according to difference between ranges

	double range_diff = range_right - range_left;
	double integral_diff = integral_right - integral_left;
	
	// Normalize difference 
	double range_diff_norm = range_diff / range_max;
	double integral_diff_norm = integral_diff * integral_weight / range_max;
	
	// Clamp integral difference
	// If integral is maxed out, encourage turning by resetting opposite integral
	if (integral_diff_norm > 1.0 ) {
		integral_diff_norm = 1.0;
		integral_left = 0.0;	// TODO check
	}
	if (integral_diff_norm < -1.0 ) {
		integral_diff_norm = -1.0;
		integral_right = 0.0;
	}
	
	
	// Calculate angular velocity (rad/s) from difference
	double ang_vel = (range_diff_norm - integral_diff_norm) * max_ang_vel / 2.0;
	
	// Influence turning only when avoiding an obstacle
	if ( fabs(ang_vel) > .01 ) {
		
		vel.angular.z = -ang_vel;
		
		// Speed up turning if obstacle is detected
		vel.angular.z *= (fabs(integral_diff_norm) + fabs(range_diff_norm) / 2.0); // TODO tune
	}
	
	//if () t_obstacle = std::clock(); // TODO Reset obstacle timer when obstacle is found
	
	ROS_INFO("t_obstacle: %f\nintegral_diff_norm: %f\nrange_diff_norm: %f",
				(std::clock() - t_obstacle) * 100.0 / ((double) CLOCKS_PER_SEC),
				integral_diff_norm,
				range_diff_norm);

	
	**/
}

bathead_control_node::~bathead_control_node( ) {}

int main(int argc, char** argv)
{
	//wiringPiSetup();
	//pinMode(chirp_trigger_pin, OUTPUT);
	//digitalWrite(chirp_trigger_pin, HIGH);
	
	ros::init(argc, argv, "bathead_control_node");
	ros::NodeHandle nh;
	bathead_control_node bcn(nh);
	ros::Rate rate(20);
	while(ros::ok())
	{
		bcn.run();
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}

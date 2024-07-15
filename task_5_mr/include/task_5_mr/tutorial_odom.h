#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <dynamic_reconfigure/server.h>
#include <task_5_mr/Odom_ParamsConfig.h>
#include <task_5_mr/run.h>
#include <cmath>
#include <cstdio>

class OdomTask
{
public:
    // Constants and variables
    const double VERTICES = 5;
    double differenceOfAngle;   // Difference between current angle and new angle
    double desireAngle;         // Angle to compare with current angle for turning decision
    double rotationDecision;    // Decision for left (1) or right (2) turn
    double innerCircleAngle;    // Angle at the origin of outer and inner circle
    double DRIVE_DISTANCE;      // Length of star edges
    double FIRST_TURN_ANGLE;    // Angle for the first turn from the origin
    double OUTER_RADIUS;
    double INNER_RADIUS;
    double OUTER_ANGLE;
    double INNER_ANGLE;
    double MAX_LINEAR_VELOCITY;
    double MAX_ANGULAR_VELOCITY;
    double newX;
    double newY;
    double newAngle;
    double currentX;
    double currentY;
    double currentAngle;

    // Constructor
    OdomTask();

private:
    ros::NodeHandle nh;
    ros::NodeHandle nhP;
    ros::Subscriber odomSub;
    ros::Publisher velPub;
    ros::ServiceServer service;

    // Dynamic reconfigure server and callback
    dynamic_reconfigure::Server<dynamic_reconfigure::Odom_ParamsConfig> server;
    dynamic_reconfigure::Server<dynamic_reconfigure::Odom_ParamsConfig>::CallbackType dynamic_callback;

    // Private methods
    void dynamicReconfigCallback(dynamic_reconfigure::Odom_ParamsConfig &config);
    bool serviceCallback(task_5_mr::run::Request &req, task_5_mr::run::Response &res);
    void odomCallback(const nav_msgs::OdometryConstPtr &odomMsg);
    double angle(double currentAngle, double newAngle);
};

// Constructor implementation
OdomTask::OdomTask()
    : nhP("~"), // Private NodeHandle initialization
      innerCircleAngle(M_PI / VERTICES), // Initialize innerCircleAngle
      newX(0), newY(0) // Initialize newX and newY
{
    // Default parameters if not provided at runtime
    nhP.param<double>("outer_radius", OUTER_RADIUS, 3);
    nhP.param<double>("inner_radius", INNER_RADIUS, 1);

    // Get parameters from parameter server if available at runtime
    nhP.getParam("outer_radius", OUTER_RADIUS);
    nhP.getParam("inner_radius", INNER_RADIUS);

    // Subscribe to odometry topic
    odomSub = nh.subscribe("odom", 1, &OdomTask::odomCallback, this);

    // Advertise to cmd_vel topic
    velPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // Advertise service for run
    service = nh.advertiseService("run", &OdomTask::serviceCallback, this);

    // Setup dynamic reconfigure server and callback
    dynamic_callback = boost::bind(&OdomTask::dynamicReconfigCallback, this, _1);
    server.setCallback(dynamic_callback);
}

// Callback function for odometry data
void OdomTask::odomCallback(const nav_msgs::OdometryConstPtr &odomMsg)
{
    // Initialize variables
    geometry_msgs::Twist velocities;

    // Process odometry data here
    // Example calculation of angles, distances, and velocities

    // Publish velocities
    velPub.publish(velocities);
}

// Dynamic reconfigure callback function
void OdomTask::dynamicReconfigCallback(dynamic_reconfigure::Odom_ParamsConfig &config)
{
    // Update parameters from dynamic reconfigure
    MAX_ANGULAR_VELOCITY = config.max_angular_velocity;
    MAX_LINEAR_VELOCITY = config.max_linear_velocity;
}

// Service callback function
bool OdomTask::serviceCallback(task_5_mr::run::Request &req, task_5_mr::run::Response &res)
{
    // Handle service request
    MAX_LINEAR_VELOCITY = req.max_linear_velocity;
    MAX_ANGULAR_VELOCITY = req.max_angular_velocity;

    // Example response
    res.success = true;
    res.message = "Parameters updated";
    return true;
}

// Function to calculate angle between current and new angle
double OdomTask::angle(double currentAngle, double newAngle)
{
    // Example implementation
    // Calculate and return angle difference
    return fabs(currentAngle - newAngle);
}

// Main function (optional for completeness, typically not included in the class)
int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_task_node");

    // Create an instance of OdomTask
    OdomTask odom_task;

    // Spin ROS node
    ros::spin();

    return 0;
}

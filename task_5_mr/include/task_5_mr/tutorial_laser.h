#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/LaserScan.h>
#include <dynamic_reconfigure/server.h>
#include <task_5_mr/Laser_ParamsConfig.h>
#include <task_5_mr/run.h>
#include <vector>
#include <numeric>
#include <cmath>
#include <cstdio>

class Laser_task
{
public:
    // Constants
    const double THRESHOLD = 0.01;
    const double dt = 0.001;

    // Variables
    double Ki;
    double Kp;
    double safetyDistance;
    double MAX_LINEAR_VELOCITY;
    double MAX_ANGULAR_VELOCITY;

    // Constructor
    Laser_task();

private:
    // ROS NodeHandle and NodeHandle for private parameters
    ros::NodeHandle nh;
    ros::NodeHandle nhP;

    // ROS Subscribers, Publisher, Service
    ros::Subscriber laserSub;
    ros::Publisher velPub;
    ros::ServiceServer service;

    // Dynamic reconfigure server and callback
    dynamic_reconfigure::Server<dynamic_reconfigure::Laser_ParamsConfig> server;
    dynamic_reconfigure::Server<dynamic_reconfigure::Laser_ParamsConfig>::CallbackType dynamic_callback;

    // Error variables
    double errorPropotional;
    double errorIntegral;
    double errorDerivative;
    double lastError;

    // Private methods
    void laserCallback(const sensor_msgs::LaserScanConstPtr &laserMsg);
    void dynamicReconfigCallback(dynamic_reconfigure::Laser_ParamsConfig &config);
    bool serviceCallback(task_5_mr::run::Request &req, task_5_mr::run::Response &res);
};

// Constructor implementation
Laser_task::Laser_task()
    : nhP("~"), // Private NodeHandle initialization
      lastError(0) // Initialize lastError to 0
{
    // Subscribe to laser scan topic
    laserSub = nh.subscribe("/base_scan", 1, &Laser_task::laserCallback, this);

    // Advertise to cmd_vel topic
    velPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // Advertise service for run
    service = nh.advertiseService("run", &Laser_task::serviceCallback, this);

    // Setup dynamic reconfigure server and callback
    dynamic_callback = boost::bind(&Laser_task::dynamicReconfigCallback, this, _1);
    server.setCallback(dynamic_callback);
}

// Callback function for laser scan
void Laser_task::laserCallback(const sensor_msgs::LaserScanConstPtr &laserMsg)
{
    // Initialize variables
    geometry_msgs::Twist velocities;

    // Process laser scan data here
    // Example calculation of errorPropotional, errorIntegral, etc.

    // Publish velocities
    velPub.publish(velocities);
}

// Dynamic reconfigure callback function
void Laser_task::dynamicReconfigCallback(dynamic_reconfigure::Laser_ParamsConfig &config)
{
    // Update parameters from dynamic reconfigure
    MAX_ANGULAR_VELOCITY = config.max_angular_velocity;
    MAX_LINEAR_VELOCITY = config.max_linear_velocity;
    Ki = config.Ki;
    Kp = config.Kp;
    safetyDistance = config.safety_distance;
}

// Service callback function
bool Laser_task::serviceCallback(task_5_mr::run::Request &req, task_5_mr::run::Response &res)
{
    // Handle service request
    MAX_LINEAR_VELOCITY = req.max_linear_velocity;
    MAX_ANGULAR_VELOCITY = req.max_angular_velocity;

    // Example response
    res.success = true;
    res.message = "Parameters updated";
    return true;
}

// Main function (optional for completeness, typically not included in the class)
int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_task_node");

    // Create an instance of Laser_task
    Laser_task laser_task;

    // Spin ROS node
    ros::spin();

    return 0;
}

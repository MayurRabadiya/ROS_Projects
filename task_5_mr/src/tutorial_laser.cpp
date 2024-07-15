#include <task_5_mr/tutorial_laser.h>
#include <numeric>  // For std::accumulate
#include <algorithm>  // For std::min_element
#include <cmath>  // For std::round

Laser_task::Laser_task()
{
    // Subscribe to the laser scan topic
    laserSub = nh.subscribe("/base_scan", 1, &Laser_task::laserCallback, this);

    // Advertise the velocity command topic
    velPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // Advertise the service to run the task
    service = nh.advertiseService("run", &Laser_task::serviceCallback, this);

    // Set up dynamic reconfigure callback
    dynamic_callback = boost::bind(&Laser_task::dynamicReconfigCallback, this, _1);
    server.setCallback(dynamic_callback);
}

// Calculate the average distance from center laser data
double centerLaserDistance(const std::vector<double>& centerLaserData)
{
    double sumC = std::accumulate(centerLaserData.begin(), centerLaserData.end(), 0.0);
    return sumC / centerLaserData.size();
}

// Calculate the minimum distance from right side laser data
double rightMinDistance(const std::vector<double>& rightLaserData)
{
    double min = *std::min_element(rightLaserData.begin(), rightLaserData.end());
    return std::round(min * 1000.0) / 1000.0;
}

// Calculate the minimum distance from left side laser data
double leftMinDistance(const std::vector<double>& leftLaserData)
{
    double min = *std::min_element(leftLaserData.begin(), leftLaserData.end());
    return std::round(min * 1000.0) / 1000.0;
}

// Service callback to update the maximum velocities
bool Laser_task::serviceCallback(task_5_mr::run::Request &req, task_5_mr::run::Response &res)
{
    MAX_LINEAR_VELOCITY = req.max_linear_velocity;
    MAX_ANGULAR_VELOCITY = req.max_angular_velocity;
    return true;
}

// Dynamic reconfigure callback to update parameters
void Laser_task::dynamicReconfigCallback(dynamic_reconfigure::Laser_ParamsConfig &config)
{
    MAX_LINEAR_VELOCITY = config.max_linear_velocity;
    MAX_ANGULAR_VELOCITY = config.max_angular_velocity;
    Ki = config.Ki;
    Kp = config.Kp;
    safetyDistance = config.safetyDistance;
}

// Laser scan callback to process the laser data and compute velocities
void Laser_task::laserCallback(const sensor_msgs::LaserScanConstPtr& laserMsg)
{
    geometry_msgs::Twist velocities;

    // Divide the laser scan data into three parts
    std::size_t laserDataSize = laserMsg->ranges.size() / 3;

    // Vectors to store data from laser scans
    std::vector<double> centerLaserData(laserMsg->ranges.begin() + laserDataSize, laserMsg->ranges.begin() + 2 * laserDataSize);
    std::vector<double> rightLaserData(laserMsg->ranges.begin(), laserMsg->ranges.begin() + laserDataSize);
    std::vector<double> leftLaserData(laserMsg->ranges.begin() + 2 * laserDataSize, laserMsg->ranges.end());

    // Compute distances
    double centerDistance = centerLaserDistance(centerLaserData);
    double actualError = rightMinDistance(rightLaserData) - leftMinDistance(leftLaserData); // Difference between left and right scan distances

    // PID control calculations
    errorPropotional = actualError;
    errorIntegral = lastError + actualError * dt; // Integrate actual error with previous error

    velocities.linear.x = MAX_LINEAR_VELOCITY;
    velocities.angular.z = -MAX_ANGULAR_VELOCITY * (Kp * errorPropotional + Ki * errorIntegral);

    // Condition to avoid obstacles
    if (centerDistance < safetyDistance)
    {
        velocities.linear.x = 0.0;
        velocities.angular.z = MAX_ANGULAR_VELOCITY;
    }

    // Log and publish the velocities
    ROS_INFO("MAX_LINEAR_VELOCITY: %f || MAX_ANGULAR_VELOCITY: %f || actualError: %f", velocities.linear.x, velocities.angular.z, actualError);

    velPub.publish(velocities);
    lastError = actualError;
}


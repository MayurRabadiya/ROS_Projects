#include <task_5_mr/tutorial_odom.h>
#include <cmath>    // For mathematical functions like sqrt and acos

OdomTask::OdomTask() : nhP("~")
{
    // Parameters initialization with default values or from parameter server
    nhP.param<double>("outer_radius", OUTER_RADIUS, 3);
    nhP.param<double>("inner_radius", INNER_RADIUS, 1);

    // ROS subscribers, publishers, services, and dynamic reconfigure setup
    odomSub = nh.subscribe("odom", 1, &OdomTask::odomCallback, this);
    velPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    service = nh.advertiseService("run", &OdomTask::serviceCallback, this);
    dynamic_callback = boost::bind(&OdomTask::dynamicReconfigCallback, this, _1);
    server.setCallback(dynamic_callback);
}

// Service callback to handle run requests
bool OdomTask::serviceCallback(task_5_mr::run::Request &req, task_5_mr::run::Response &res)
{
    MAX_LINEAR_VELOCITY = req.max_linear_velocity;
    MAX_ANGULAR_VELOCITY = req.max_angular_velocity;
    return true;
}

// Dynamic reconfigure callback to update velocity parameters
void OdomTask::dynamicReconfigCallback(dynamic_reconfigure::Odom_ParamsConfig &config)
{
    MAX_LINEAR_VELOCITY = config.max_linear_velocity;
    MAX_ANGULAR_VELOCITY = config.max_angular_velocity;
}

// Function to calculate desired angle based on current and new angle
double OdomTask::angle(double currentAngle, double newAngle)
{
    DRIVE_DISTANCE = sqrt((pow(OUTER_RADIUS, 2) + pow(INNER_RADIUS, 2)) - (2 * OUTER_RADIUS * INNER_RADIUS * cos(innerCircleAngle)));
    OUTER_ANGLE = 2 * M_PI - 2 * (acos((pow(DRIVE_DISTANCE, 2) + pow(INNER_RADIUS, 2) - pow(OUTER_RADIUS, 2)) / (2 * DRIVE_DISTANCE * INNER_RADIUS)));
    INNER_ANGLE = 2 * acos((pow(DRIVE_DISTANCE, 2) + pow(OUTER_RADIUS, 2) - pow(INNER_RADIUS, 2)) / (2 * DRIVE_DISTANCE * OUTER_RADIUS));
    FIRST_TURN_ANGLE = M_PI - (INNER_ANGLE / 2);

    differenceOfAngle = fabs(currentAngle - newAngle);
    if (differenceOfAngle > M_PI)
    {
        desireAngle = (2 * M_PI) - differenceOfAngle;
    }
    else
    {
        desireAngle = differenceOfAngle;
    }
    return (desireAngle);
}

// Callback function to handle odometry data
void OdomTask::odomCallback(const nav_msgs::OdometryConstPtr &odomMsg)
{
    geometry_msgs::Twist velocities;

    // Extract current pose and orientation information
    currentX = odomMsg->pose.pose.position.x;
    currentY = odomMsg->pose.pose.position.y;
    currentAngle = tf::getYaw(odomMsg->pose.pose.orientation);

    // Calculate distance traveled by the robot
    double x = currentX - newX;
    double y = currentY - newY;
    double distance = sqrt(pow(y, 2) + pow(x, 2));

    // Calculate desired angle based on current and new angle
    desireAngle = angle(currentAngle, newAngle);

    // Print debug information
    printf("DRIVE_DISTANCE: %f || OUTER_ANGLE: %f || INNER_ANGLE: %f || OUTER_RADIUS: %f || INNER_RADIUS: %f \n",
            DRIVE_DISTANCE, OUTER_ANGLE * 180 / M_PI, INNER_ANGLE * 180 / M_PI, OUTER_RADIUS, INNER_RADIUS);

    // State machine to control robot behavior based on current state
    switch (currentState)
    {
        case START:
        {
            velocities.linear.x = MAX_LINEAR_VELOCITY;
            velocities.angular.z = 0.0;

            if (distance < OUTER_RADIUS)
            {
                velocities.angular.z = 0.0;
                velocities.linear.x = MAX_LINEAR_VELOCITY;
            }
            else if (desireAngle < FIRST_TURN_ANGLE)
            {
                velocities.angular.z = -MAX_ANGULAR_VELOCITY;
                velocities.linear.x = 0.0;
            }
            else
            {
                newX = currentX;
                newY = currentY;
                currentState = ROTATE_RIGHT;
            }
        }
        break;

        case ROTATE_RIGHT:
        {
            if (desireAngle < M_PI - INNER_ANGLE)
            {
                velocities.angular.z = -MAX_ANGULAR_VELOCITY;
                velocities.linear.x = 0.0;
            }
            else
            {
                newAngle = currentAngle;
                rotationDecision = 1;
                currentState = MOVE;
            }
        }
        break;

        case MOVE:
        {
            velocities.linear.x = MAX_LINEAR_VELOCITY;
            velocities.angular.z = 0.0;

            if (distance > DRIVE_DISTANCE)
            {
                newX = currentX;
                newY = currentY;

                if (rotationDecision == 1)
                {
                    currentState = ROTATE_LEFT;
                }
                else if (rotationDecision == 2)
                {
                    currentState = ROTATE_RIGHT;
                }
            }
        }
        break;

        case ROTATE_LEFT:
        {
            if (desireAngle < M_PI - OUTER_ANGLE)
            {
                velocities.angular.z = MAX_ANGULAR_VELOCITY;
                velocities.linear.x = 0.0;
            }
            else
            {
                newAngle = currentAngle;
                rotationDecision = 2;
                currentState = MOVE;
            }
        }
        break;

        default:
            printf("Error: Default case should never be active!\n");
    }

    // Publish velocities to command the robot
    velPub.publish(velocities);
}

#include <task_5_mr/tutorial_odom.h>

int main(int argc, char *argv[])
{
    // Initialize the ROS system
    ros::init(argc, argv, "OdomTask");

    // Create an instance of the OdomTask class
    OdomTask odomtask;

    // Set the loop rate to 10 Hz
    ros::Rate r(10);

    // Main ROS loop
    while (ros::ok())
    {
        // Allow ROS to process incoming messages, callbacks, etc.
        ros::spinOnce();

        // Sleep for the remaining time to achieve the loop rate
        r.sleep();
    }

    return 0;
}

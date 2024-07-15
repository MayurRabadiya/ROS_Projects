#include <task_5_mr/tutorial_laser.h>

int main(int argc, char *argv[])
{
    // Initialize the ROS system
    ros::init(argc, argv, "laser_task");

    // Create an instance of the Laser_task class
    Laser_task laser_task;

    // Enter the ROS event loop
    ros::spin();

    return 0;
}

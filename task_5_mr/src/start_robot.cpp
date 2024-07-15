#include <ros/ros.h>               // ROS core library
#include <task_5_mr/run.h>         // Service message for 'run' service
#include <cstdlib>                 // Standard library for general purpose functions

int main(int argc, char **argv)
{
  // Initialize the ROS system with the name "Start_Robot"
  ros::init(argc, argv, "Start_Robot");

  // Create a handle for this node, will be used to create clients, publishers, etc.
  ros::NodeHandle nh;

  // Create a service client for the "run" service
  ros::ServiceClient startClient = nh.serviceClient<task_5_mr::run>("run");

  // Create a service object to hold the request and response
  task_5_mr::run srv;

  // Set the maximum linear and angular velocities for the robot
  srv.request.max_linear_velocity = 0.5;
  srv.request.max_angular_velocity = 0.5;

  // Call the service and check if the call was successful
  if (startClient.call(srv))
  {
    // Service call was successful, check the response
    ROS_INFO("Service call successful");
  }
  else
  {
    // Service call failed, log an error message
    ROS_ERROR("Failed to call service run");
    return 1;  // Return with error code
  }

  return 0;  // Return success code
}

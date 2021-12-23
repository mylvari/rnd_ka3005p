#include <iostream>
#include <chrono>
#include <thread>

#include "ros/ros.h"
#include "rnd_ka3005p/rnd_ka3005p_ros.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rnd_ka3005p");
  ros::NodeHandle n;

  while (ros::ok())
  {
    try
    {
      RndKa3005pRos node;
      node.init();

      ros::Rate loop_rate(35);

      while (ros::ok())
      {
        ros::spinOnce();
        node.spin();
        loop_rate.sleep();
      }
      break;
    }
    catch (const serial::SerialException &e)
    {
      ROS_ERROR("Lost connection, trying again");
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    catch (const serial::IOException &e)
    {
      ROS_ERROR("Cannot open the port, trying again");
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
  }

  return 0;
}
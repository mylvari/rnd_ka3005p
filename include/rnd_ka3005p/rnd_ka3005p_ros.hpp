#ifndef __rnd_ka3005p_ros_H__
#define __rnd_ka3005p_ros_H__

#include "ros/ros.h"
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include "rnd_ka3005p/RndKa3005pStatus.h"
#include "rnd_ka3005p/rnd_ka3005p.hpp"

class RndKa3005pRos
{
private:
    ros::NodeHandle ros_nh_;

    RndKa3005p psu_;
    std::string psu_port_;

    ros::Publisher psu_voltage_pub_,
        psu_current_pub_,
        psu_voltage_setpoint_readback_pub_,
        psu_current_setpoint_readback_pub_,
        psu_identification_pub_,
        psu_status_pub_;

    ros::Subscriber psu_voltage_set_sub_;
    void psu_voltage_set_cb_(const std_msgs::Float32::ConstPtr &msg);
    ros::Subscriber psu_current_set_sub_;
    void psu_current_set_cb_(const std_msgs::Float32::ConstPtr &msg);
    ros::Subscriber psu_enabled_sub_;
    void psu_enabled_cb_(const std_msgs::Bool::ConstPtr &msg);

public:
    RndKa3005pRos();
    // ~RndKa3005pRos();

    void init();
    void spin();
};

#endif // __rnd_ka3005p_ros_H__
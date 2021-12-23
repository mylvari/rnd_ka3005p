#include "rnd_ka3005p/rnd_ka3005p_ros.hpp"

#include <iostream>
#include <chrono>
#include <thread>

RndKa3005pRos::RndKa3005pRos()
{
}

void RndKa3005pRos::init()
{
    ros_nh_.param<std::string>("port", psu_port_, "/dev/ttyACM0");

    psu_voltage_pub_ = ros_nh_.advertise<std_msgs::Float32>("voltage", 100);
    psu_current_pub_ = ros_nh_.advertise<std_msgs::Float32>("current", 100);
    psu_voltage_setpoint_readback_pub_ = ros_nh_.advertise<std_msgs::Float32>("voltage_setpoint_readback", 100);
    psu_current_setpoint_readback_pub_ = ros_nh_.advertise<std_msgs::Float32>("current_setpoint_readback", 100);
    psu_identification_pub_ = ros_nh_.advertise<std_msgs::String>("identification", 100, true);
    psu_status_pub_ = ros_nh_.advertise<rnd_ka3005p::RndKa3005pStatus>("status", 100);

    psu_voltage_set_sub_ = ros_nh_.subscribe("voltage_setpoint", 1, &RndKa3005pRos::psu_voltage_set_cb_, this);
    psu_current_set_sub_ = ros_nh_.subscribe("current_setpoint", 1, &RndKa3005pRos::psu_current_set_cb_, this);
    psu_enabled_sub_ = ros_nh_.subscribe("enabled", 1, &RndKa3005pRos::psu_enabled_cb_, this);

    psu_.open(psu_port_);

    std_msgs::String identification_msg;
    identification_msg.data = psu_.get_identification();
    psu_identification_pub_.publish(identification_msg);
    ROS_INFO("Connected to %s", identification_msg.data.c_str());
}

void RndKa3005pRos::psu_voltage_set_cb_(const std_msgs::Float32::ConstPtr &msg)
{
    psu_.set_voltage(msg->data);
}

void RndKa3005pRos::psu_current_set_cb_(const std_msgs::Float32::ConstPtr &msg)
{
    psu_.set_current(msg->data);
}

void RndKa3005pRos::psu_enabled_cb_(const std_msgs::Bool::ConstPtr &msg)
{
    psu_.set_output(msg->data);
}

void RndKa3005pRos::spin()
{
    std_msgs::Float32 voltage_message;
    voltage_message.data = psu_.get_voltage();
    psu_voltage_pub_.publish(voltage_message);

    std_msgs::Float32 current_message;
    current_message.data = psu_.get_current();
    psu_current_pub_.publish(current_message);

    std_msgs::Float32 psu_voltage_setpoint_readback_message;
    psu_voltage_setpoint_readback_message.data = psu_.get_voltage_setpoint();
    psu_voltage_setpoint_readback_pub_.publish(psu_voltage_setpoint_readback_message);

    std_msgs::Float32 psu_current_setpoint_readback_message;
    psu_current_setpoint_readback_message.data = psu_.get_current_setpoint();
    psu_current_setpoint_readback_pub_.publish(psu_current_setpoint_readback_message);

    RndKa3005pStatusMsg status_raw = psu_.get_status();
    rnd_ka3005p::RndKa3005pStatus status_message;
    status_message.mode = status_raw.mode;
    status_message.output_on = status_raw.output_on;
    status_message.over_current_protetion_on = status_raw.over_current_protetion_on;
    status_message.over_voltage_protection_on = status_raw.over_voltage_protection_on;
    psu_status_pub_.publish(status_message);
}
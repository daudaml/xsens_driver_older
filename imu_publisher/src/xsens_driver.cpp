#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "cmt2.h"
#include <signal.h>

int quit = 0;

void ctrlchandler(int sig)
{
    // got ctrl-c
    quit = 1;
}

using namespace xsens;

#define EXIT_ERROR(loc) {RCLCPP_ERROR(rclcpp::get_logger("imu_publisher_node"), "Error %d occurred during %s: %s\n", serial.getLastResult(), loc, xsensResultText(serial.getLastResult())); }

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("imu_publisher_node");
    
    auto imu_pub = node->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
    tf2_ros::TransformBroadcaster tf_broadcaster(node);
    rclcpp::Rate loop_rate(100); // 100 Hz

    Cmt2s serial;
    Message msg, reply;
    (void) signal(SIGINT, ctrlchandler);
    RCLCPP_INFO(node->get_logger(), "This example will connect to the MT, configure it for Euler output at 100Hz and read data messages until Control-C is pressed\n");
    RCLCPP_INFO(node->get_logger(), "Please be aware that this example has a hard-coded baud rate of 115200!\n");

    char portname[32];
    sprintf(portname, "/dev/ttyUSB0");
    serial.setTimeout(2000);

    if (serial.open(portname, B115200) != XRV_OK)
        EXIT_ERROR("open");
    
    msg.setMessageId(CMT_MID_GOTOCONFIG);
    RCLCPP_INFO(node->get_logger(), "Putting MT in config mode\n");
    if (serial.writeMessage(&msg))
        EXIT_ERROR("goto config");
    RCLCPP_INFO(node->get_logger(), "MT now in config mode\n");

    msg.setMessageId(CMT_MID_SETPERIOD);
    msg.setDataShort(1152);
    if (serial.writeMessage(&msg))
        EXIT_ERROR("set period");
    RCLCPP_INFO(node->get_logger(), "Period is now set to 100Hz\n");

    msg.setMessageId(CMT_MID_SETOUTPUTMODE);
    msg.setDataShort(CMT_OUTPUTMODE_ORIENT);
    if (serial.writeMessage(&msg))
        EXIT_ERROR("set output mode");
    RCLCPP_INFO(node->get_logger(), "Output mode is now set to orientation\n");

    msg.setMessageId(CMT_MID_SETOUTPUTSETTINGS);
    msg.setDataLong(CMT_OUTPUTSETTINGS_ORIENTMODE_EULER | CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT);
    if (serial.writeMessage(&msg))
        EXIT_ERROR("set output settings");
    RCLCPP_INFO(node->get_logger(), "Output settings now set to euler + timestamp\n");

    msg.setMessageId(CMT_MID_GOTOMEASUREMENT);
    msg.resizeData(0);
    if (serial.writeMessage(&msg))
        EXIT_ERROR("goto measurement");
    RCLCPP_INFO(node->get_logger(), "Now in measurement mode\n");

    while (rclcpp::ok() && !quit)
    {
        if (serial.waitForMessage(&reply, 0, 0, 1) != XRV_OK)
            EXIT_ERROR("read data message");

        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = node->now();
        imu_msg.header.frame_id = "imu_link";

        // Convert Euler angles (roll, pitch, yaw) from degrees to radians
        double roll = (double) reply.getDataFloat(0*4) * M_PI / 180.0;  // roll in radians
        double pitch = (double) reply.getDataFloat(1*4) * M_PI / 180.0; // pitch in radians
        double yaw = (double) reply.getDataFloat(2*4) * M_PI / 180.0;   // yaw in radians

        // Print roll, pitch, yaw in degrees
        // RCLCPP_INFO(node->get_logger(), "Roll: %f, Pitch: %f, Yaw: %f (in degrees)", roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);

        tf2::Quaternion quat;
        quat.setRPY(roll, pitch, yaw);

        imu_msg.orientation.x = quat.x();
        imu_msg.orientation.y = quat.y();
        imu_msg.orientation.z = quat.z();
        imu_msg.orientation.w = quat.w();

        // Set angular velocities and linear accelerations to zero (if not available)
        imu_msg.angular_velocity.x = 0.0;
        imu_msg.angular_velocity.y = 0.0;
        imu_msg.angular_velocity.z = 0.0;

        imu_msg.linear_acceleration.x = 0.0;
        imu_msg.linear_acceleration.y = 0.0;
        imu_msg.linear_acceleration.z = 0.0;

        imu_pub->publish(imu_msg);

        // Publish the transform
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = imu_msg.header.stamp;
        transformStamped.header.frame_id = "base_link";  // Parent frame (e.g., base of the robot)
        transformStamped.child_frame_id = "imu_link";    // Child frame (IMU frame)

        transformStamped.transform.translation.x = 0.0;  // If you have specific IMU offsets, set them here
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.0;
        transformStamped.transform.rotation.x = quat.x();
        transformStamped.transform.rotation.y = quat.y();
        transformStamped.transform.rotation.z = quat.z();
        transformStamped.transform.rotation.w = quat.w();

        tf_broadcaster.sendTransform(transformStamped);

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    RCLCPP_INFO(node->get_logger(), "Done reading\n");
    (void) signal(SIGINT, SIG_DFL);
    rclcpp::shutdown();
    return 0;
}

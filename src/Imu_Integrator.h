#ifndef SR_NODE_EXAMPLE_CORE_H
#define SR_NODE_EXAMPLE_CORE_H

// ROS includes.
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <Eigen/Dense>
#include <cmath>
// Custom message includes. Auto-generated from msg/ directory.
/*
struct Orientation
{

};

struct Position
{
    double x, y, z;
};
*/
struct Pose
{
    /*
    Orientation orien;
    Position pos;
    */
    Eigen::Vector3d pos;
    Eigen::Matrix3d orien;
};

class ImuIntegrator : public rclcpp::Node
{
private:
    Pose pose;
    rclcpp::Time time;
    Eigen::Vector3d gravity;
    Eigen::Vector3d velocity;
    visualization_msgs::msg::Marker path;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr line_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr data_sub_;
    double deltaT;
    bool firstT;
public:
    //! Constructor.
    ImuIntegrator(const std::string& topic);
    //! Destructor.

    //! Callback function for dynamic reconfigure server.
    //void configCallback(node_example::node_example_paramsConfig &config, uint32_t level);

    //! Publish the message.
    void publishMessage();

    //! Callback function for subscriber.
    void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

    void setGravity(const geometry_msgs::msg::Vector3& msg);
    void updatePath(const Eigen::Vector3d &msg);
    void calcPosition(const geometry_msgs::msg::Vector3& msg);
    void calcOrientation(const geometry_msgs::msg::Vector3& msg);
};

#endif // SR_NODE_EXAMPLE_CORE_H

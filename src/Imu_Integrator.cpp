#include "Imu_Integrator.h"

ImuIntegrator::ImuIntegrator(const std::string& topic) 
: Node("imu_integrator")
{
  Eigen::Vector3d zero(0, 0, 0);
  pose.pos = zero;
  pose.orien = Eigen::Matrix3d::Identity();
  velocity = zero;
  
  firstT = true;

  // Line strip is blue
  path.color.b = 1.0;
  path.color.a = 1.0;
  path.type = visualization_msgs::msg::Marker::LINE_STRIP;
  path.header.frame_id = "/global";
  path.ns = "points_and_lines";
  path.action = visualization_msgs::msg::Marker::ADD;
  path.pose.orientation.w = 1.0;
  path.scale.x = 0.2;
  geometry_msgs::msg::Point p;
  p.x = 0;
  p.y = 0;
  p.z = 0;
  path.points.push_back(p);


  line_pub_  = this->create_publisher<visualization_msgs::msg::Marker>(topic, 10);


  data_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", 
    10, 
    std::bind(&ImuIntegrator::ImuCallback, this, std::placeholders::_1)
  );


  RCLCPP_INFO(this->get_logger(), "Imu Integrator Node init done.");
}

void ImuIntegrator::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {


  if (firstT) {
    time = msg->header.stamp;
    deltaT = 0;
    setGravity(msg->linear_acceleration);
    firstT = false;
  } else {
    deltaT = (rclcpp::Time(msg->header.stamp) - time).seconds();
    time = msg->header.stamp;
    calcOrientation(msg->angular_velocity);
    calcPosition(msg->linear_acceleration);
    updatePath(pose.pos);
    publishMessage();
  }
  // std::cout << pose.pos << std::endl;
}

void ImuIntegrator::setGravity(const geometry_msgs::msg::Vector3& msg) {
  gravity[0] = msg.x;
  gravity[1] = msg.y;
  gravity[2] = msg.z;
}

void ImuIntegrator::updatePath(const Eigen::Vector3d &msg) {
  geometry_msgs::msg::Point p;
  p.x = msg[0];
  p.y = msg[1];
  p.z = msg[2];
  path.points.push_back(p);

  RCLCPP_INFO(this->get_logger(), "point update(%f, %f, %f).", p.x, p.y, p.z);
}

void ImuIntegrator::publishMessage() { line_pub_->publish(path); }

void ImuIntegrator::calcOrientation(const geometry_msgs::msg::Vector3& msg) {


  RCLCPP_INFO(this->get_logger(), "Orientation update(%f, %f, %f).", msg.x, msg.y, msg.z);

  Eigen::Matrix3d B;

  /*
  * 0         -z * dT     y * dT
  *
  * z * dT    0           -x * dT
  *    
  * -y * dT   x * dT      0
  */


  B << 0, -msg.z * deltaT, msg.y * deltaT, msg.z * deltaT, 0, -msg.x * deltaT,
      -msg.y * deltaT, msg.x * deltaT, 0;


  double sigma =
      std::sqrt(std::pow(msg.x, 2) + std::pow(msg.y, 2) + std::pow(msg.z, 2)) *
      deltaT;

  // std::cout << "sigma: " << sigma << std::endl << Eigen::Matrix3d::Identity()
  // + (std::sin(sigma) / sigma) * B << std::endl << pose.orien << std::endl;

  pose.orien = pose.orien *
               (Eigen::Matrix3d::Identity() + (std::sin(sigma) / sigma) * B -
                ((1 - std::cos(sigma)) / std::pow(sigma, 2)) * B * B);
}

void ImuIntegrator::calcPosition(const geometry_msgs::msg::Vector3& msg) {


  RCLCPP_INFO(this->get_logger(), "Position update(%f, %f, %f).", msg.x, msg.y, msg.z);

  Eigen::Vector3d acc_l(msg.x, msg.y, msg.z);
  Eigen::Vector3d acc_g = pose.orien * acc_l;
  // Eigen::Vector3d acc(msg.x - gravity[0], msg.y - gravity[1], msg.z -
  // gravity[2]);
  velocity = velocity + deltaT * (acc_g - gravity);
  pose.pos = pose.pos + deltaT * velocity;
}

int main(int argc, char **argv) {

  (void)argc;
  (void **)argv;

  int _argc = 0;
  char** _argv = NULL;
  rclcpp::init(_argc,_argv);


  auto imu_integrator = std::make_shared<ImuIntegrator>("/imu/path");
  
  rclcpp::spin(imu_integrator);

  rclcpp::shutdown();

  return 0;
}

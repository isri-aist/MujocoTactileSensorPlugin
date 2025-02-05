#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <mujoco_tactile_sensor_plugin/msg/tactile_sensor_data.hpp>

#include <cmath>
#include <tf2_eigen/tf2_eigen.hpp>

class MarkerPublisher
{
public:
  MarkerPublisher(rclcpp::Node::SharedPtr& nh);

  void runLoop();

protected:
  void callback(const mujoco_tactile_sensor_plugin::msg::TactileSensorData::SharedPtr msg);

protected:
  rclcpp::Node::SharedPtr nh_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_arr_pub_;
  rclcpp::Subscription<mujoco_tactile_sensor_plugin::msg::TactileSensorData>::SharedPtr tactile_sensor_sub_;

  float marker_color_alpha_ = 0.5f;
};

MarkerPublisher::MarkerPublisher(rclcpp::Node::SharedPtr& nh)
{
  nh_ = nh;

  tactile_sensor_sub_ = nh_->create_subscription<mujoco_tactile_sensor_plugin::msg::TactileSensorData>(
      "tactile_sensor", 1, std::bind(&MarkerPublisher::callback, this, std::placeholders::_1));
  marker_arr_pub_ = nh_->create_publisher<visualization_msgs::msg::MarkerArray>("marker_arr", 1);

  nh_->get_parameter("marker_color_alpha", marker_color_alpha_);
}

void MarkerPublisher::runLoop()
{
  rclcpp::spin(nh_);
}

void MarkerPublisher::callback(const mujoco_tactile_sensor_plugin::msg::TactileSensorData::SharedPtr msg)
{
  // Instantiate marker array
  visualization_msgs::msg::MarkerArray marker_arr_msg;

  // \todo Markers visualized in Rviz may blink when DELETEALL is added
  // // Delete marker
  // visualization_msgs::msg::Marker del_marker;
  // del_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  // del_marker.header = msg->header;
  // del_marker.id = static_cast<int>(marker_arr_msg.markers.size());
  // marker_arr_msg.markers.push_back(del_marker);

  // Sensor marker
  double force_max = 0.0;
  for(int sensor_idx = 0; sensor_idx < static_cast<int>(msg->forces.size()); sensor_idx++)
  {
    force_max = std::max(force_max, std::abs(msg->forces[sensor_idx]));
  }
  for(int sensor_idx = 0; sensor_idx < static_cast<int>(msg->forces.size()); sensor_idx++)
  {
    visualization_msgs::msg::Marker sensor_marker;
    sensor_marker.header = msg->header;
    sensor_marker.ns = "sensor";
    sensor_marker.id = static_cast<int>(marker_arr_msg.markers.size());
    if(msg->grid_type == msg->grid_square)
    {
      sensor_marker.type = visualization_msgs::msg::Marker::CUBE;
    }
    else // if(msg->grid_type == msg->grid_hex)
    {
      sensor_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    }
    sensor_marker.color.r = 1.0f;
    sensor_marker.color.g = 1.0f;
    sensor_marker.color.b = 0.8f;
    sensor_marker.color.a = marker_color_alpha_;
    if(msg->forces[sensor_idx] > 0)
    {
      sensor_marker.color.r = static_cast<float>(0.7 * (1.0 - msg->forces[sensor_idx] / force_max));
    }
    else if(msg->forces[sensor_idx] < 0)
    {
      sensor_marker.color.g = static_cast<float>(0.7 * (1.0 + msg->forces[sensor_idx] / force_max));
    }
    if(msg->grid_type == msg->grid_square)
    {
      sensor_marker.scale.x = msg->sensor_interval;
      sensor_marker.scale.y = msg->sensor_interval;
    }
    else // if(msg->grid_type == msg->grid_hex)
    {
      sensor_marker.scale.x = msg->sensor_interval;
      sensor_marker.scale.y = msg->sensor_interval;
    }
    sensor_marker.scale.z = 0.002;
    sensor_marker.pose.position = msg->positions[sensor_idx];
    if(msg->surface_type == msg->surface_plane)
    {
      sensor_marker.pose.orientation.w = 1.0;
      sensor_marker.pose.orientation.x = 0.0;
      sensor_marker.pose.orientation.y = 0.0;
      sensor_marker.pose.orientation.z = 0.0;
    }
    else // if(msg->surface_type == msg->surface_cylinder)
    {
      Eigen::Quaterniond quat = Eigen::Quaterniond(Eigen::AngleAxisd(
          std::atan2(-1 * msg->normals[sensor_idx].y, msg->normals[sensor_idx].z), Eigen::Vector3d::UnitX()));
      sensor_marker.pose.orientation.w = quat.w();
      sensor_marker.pose.orientation.x = quat.x();
      sensor_marker.pose.orientation.y = quat.y();
      sensor_marker.pose.orientation.z = quat.z();
    }
    sensor_marker.frame_locked = true;
    marker_arr_msg.markers.push_back(sensor_marker);
  }

  marker_arr_pub_->publish(marker_arr_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  std::shared_ptr<rclcpp::Node> nh = rclcpp::Node::make_shared("marker_publisher", node_options);
  auto marker_publisher = std::make_shared<MarkerPublisher>(nh);
  marker_publisher->runLoop();

  return 0;
}

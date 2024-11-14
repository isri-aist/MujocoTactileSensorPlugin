#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <mujoco_tactile_sensor_plugin/TactileSensorData.h>

#include <cmath>
#include <eigen_conversions/eigen_msg.h>

class MarkerPublisher
{
public:
  MarkerPublisher();

  void runLoop();

protected:
  void callback(const mujoco_tactile_sensor_plugin::TactileSensorData::ConstPtr & msg);

protected:
  ros::NodeHandle nh_;
  ros::Subscriber tactile_sensor_sub_;
  ros::Publisher marker_arr_pub_;
};

MarkerPublisher::MarkerPublisher()
{
  tactile_sensor_sub_ = nh_.subscribe("tactile_sensor", 1, &MarkerPublisher::callback, this);
  marker_arr_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("marker_arr", 1, true);
}

void MarkerPublisher::runLoop()
{
  ros::spin();
}

void MarkerPublisher::callback(const mujoco_tactile_sensor_plugin::TactileSensorData::ConstPtr & msg)
{
  // Instantiate marker array
  visualization_msgs::MarkerArray marker_arr_msg;

  // \todo Markers visualized in Rviz may blink when DELETEALL is added
  // // Delete marker
  // visualization_msgs::Marker del_marker;
  // del_marker.action = visualization_msgs::Marker::DELETEALL;
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
    visualization_msgs::Marker sensor_marker;
    sensor_marker.header = msg->header;
    sensor_marker.ns = "sensor";
    sensor_marker.id = static_cast<int>(marker_arr_msg.markers.size());
    if(msg->grid_type == mujoco_tactile_sensor_plugin::TactileSensorData::GridSquare)
    {
      sensor_marker.type = visualization_msgs::Marker::CUBE;
    }
    else // if(msg->grid_type == mujoco_tactile_sensor_plugin::TactileSensorData::GridHex)
    {
      sensor_marker.type = visualization_msgs::Marker::CYLINDER;
    }
    sensor_marker.color.r = 1.0f;
    sensor_marker.color.g = 1.0f;
    sensor_marker.color.b = 0.8f;
    sensor_marker.color.a = 0.5f;
    if(msg->forces[sensor_idx] > 0)
    {
      sensor_marker.color.r = static_cast<float>(0.7 * (1.0 - msg->forces[sensor_idx] / force_max));
    }
    else if(msg->forces[sensor_idx] < 0)
    {
      sensor_marker.color.g = static_cast<float>(0.7 * (1.0 + msg->forces[sensor_idx] / force_max));
    }
    if(msg->grid_type == mujoco_tactile_sensor_plugin::TactileSensorData::GridSquare)
    {
      sensor_marker.scale.x = msg->sensor_interval;
      sensor_marker.scale.y = msg->sensor_interval;
    }
    else // if(msg->grid_type == mujoco_tactile_sensor_plugin::TactileSensorData::GridHex)
    {
      sensor_marker.scale.x = msg->sensor_interval;
      sensor_marker.scale.y = msg->sensor_interval;
    }
    sensor_marker.scale.z = 0.002;
    sensor_marker.pose.position = msg->positions[sensor_idx];
    if(msg->surface_type == mujoco_tactile_sensor_plugin::TactileSensorData::SurfacePlane)
    {
      sensor_marker.pose.orientation.w = 1.0;
      sensor_marker.pose.orientation.x = 0.0;
      sensor_marker.pose.orientation.y = 0.0;
      sensor_marker.pose.orientation.z = 0.0;
    }
    else // if(msg->surface_type == mujoco_tactile_sensor_plugin::TactileSensorData::SurfaceCylinder)
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

  marker_arr_pub_.publish(marker_arr_msg);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "marker_publisher");

  auto marker_publisher = std::make_shared<MarkerPublisher>();
  marker_publisher->runLoop();

  return 0;
}

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <mujoco_tactile_sensor_plugin/TactileSensorData.h>

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

  // Delete marker
  visualization_msgs::Marker del_marker;
  del_marker.action = visualization_msgs::Marker::DELETEALL;
  del_marker.header = msg->header;
  del_marker.id = static_cast<int>(marker_arr_msg.markers.size());
  marker_arr_msg.markers.push_back(del_marker);

  // Sensor marker
  for(int sensor_idx = 0; sensor_idx < static_cast<int>(msg->forces.size()); sensor_idx++)
  {
    visualization_msgs::Marker sensor_marker;
    sensor_marker.header = msg->header;
    sensor_marker.ns = "sensor";
    sensor_marker.id = static_cast<int>(marker_arr_msg.markers.size());
    sensor_marker.type = visualization_msgs::Marker::CUBE;
    sensor_marker.color.r = 0.8f;
    sensor_marker.color.g = 0.8f;
    sensor_marker.color.b = 0.8f;
    sensor_marker.color.a = 0.5f;
    sensor_marker.scale.x = msg->sensor_interval;
    sensor_marker.scale.y = msg->sensor_interval;
    sensor_marker.scale.z = 0.002;
    sensor_marker.pose.position = msg->positions[sensor_idx];
    // tf::quaternionEigenToMsg(msg->normals[sensor_idx], sensor_marker.pose.orientation);
    marker_arr_msg.markers.push_back(sensor_marker);
  }
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "marker_publisher");

  auto marker_publisher = std::make_shared<MarkerPublisher>();
  marker_publisher->runLoop();

  return 0;
}

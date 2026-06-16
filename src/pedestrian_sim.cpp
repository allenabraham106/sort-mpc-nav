#include "sort_mpc_nav/pedestrian_sim.hpp"

namespace sort_mpc_nav{
  PedestrianSim::PedestrianSim() : Node("pedestrian_sim"){
    pedestrians_ = {
        {0.0, 0.0, 0.5, 0.2},
        {3.0, 1.0, -0.3, 0.4},
        {1.5, -2.0, 0.1, -0.5}
    };

    for(size_t i = 0; i < pedestrians_.size(); ++i){
        std::string topic = "/pedestrian_" + std::to_string(i) + "/pose";
        auto pub = create_publisher<geometry_msgs::msg::Point>(topic, 10);
        publishers_.push_back(pub); // store it safely
    };

    timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&PedestrianSim::update, this)
    );

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/pedestrian_markers", 10);

    RCLCPP_INFO(get_logger(), "Pedestian simulation started with %zu pedestrians", pedestrians_.size());
  }

  void PedestrianSim::update(){
    visualization_msgs::msg::MarkerArray marker_array;
    for(size_t i = 0; i < pedestrians_.size(); ++i){
      visualization_msgs::msg::Marker marker;
      // looping through the movements based on the the relation based on the velocity and original position
      // new position = old + instant velocity
      pedestrians_[i].x += pedestrians_[i].vx * 0.1; 
      pedestrians_[i].y += pedestrians_[i].vy * 0.1;

      geometry_msgs::msg::Point msg;
      msg.x = pedestrians_[i].x;
      msg.y = pedestrians_[i].y;

      marker.header.frame_id = "map";
      marker.header.stamp = this->now();
      marker.ns = "pedestrians";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.pose.position.x = pedestrians_[i].x;
      marker.pose.position.y = pedestrians_[i].y;
      marker.pose.position.z = 0.0; 
      marker.pose.orientation.w = 1.0;

      marker.scale.x = 0.4;
      marker.scale.y = 0.4;
      marker.scale.z = 0.4;

      marker.color.r = 1.0f;
      marker.color.g = 0.3f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0f;
      
      publishers_[i]->publish(msg);
      marker_array.markers.push_back(marker);
    }
    marker_pub_->publish(marker_array);
  }
}

int main(int argc, char ** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sort_mpc_nav::PedestrianSim>());
  rclcpp::shutdown();
  return 0;
}
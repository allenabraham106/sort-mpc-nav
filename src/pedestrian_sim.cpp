#include "sort_mpc_nav/pedestrian_sim.hpp"

namespace sort_mpc_nav{
  PedestrianSim::PedestrianSim() : Node("pedestrian_sim"){

    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> pos_dist(-4.0, 4.0);
    std::uniform_real_distribution<double> vel_dist(-0.5, 0.5);
    for(int i = 0; i < 7; ++i){
      pedestrians_.push_back({
        pos_dist(rng),
        pos_dist(rng),
        vel_dist(rng),
        vel_dist(rng)
      });
    }

    detection_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
      "/pedestrian_detections",
      10 
    );

    timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&PedestrianSim::update, this)
    );

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/pedestrian_markers",
      10
    );

    RCLCPP_INFO(get_logger(), "Pedestian simulation started with %zu pedestrians", pedestrians_.size());
  }

  void PedestrianSim::update(){
    visualization_msgs::msg::MarkerArray marker_array;
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.frame_id = "map";
    pose_array.header.stamp = this->now();
    for(size_t i = 0; i < pedestrians_.size(); ++i){
      visualization_msgs::msg::Marker marker;
      // looping through the movements based on the the relation based on the velocity and original position
      // new position = old + instant velocity
      pedestrians_[i].x += pedestrians_[i].vx * 0.1; 
      pedestrians_[i].y += pedestrians_[i].vy * 0.1;

      geometry_msgs::msg::Pose pose; 
      pose.position.x = pedestrians_[i].x; 
      pose.position.y = pedestrians_[i].y; 
      pose.position.z = 0.0;
      pose_array.poses.push_back(pose);

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

      // boundary checks for the pedestrians to bounce
      if(pedestrians_[i].x > 5.0 || pedestrians_[i].x <-5.0){
        pedestrians_[i].vx *= -1.0;
      }

      if(pedestrians_[i].y > 5.0 || pedestrians_[i].y < -5.0){
        pedestrians_[i].vy *= -1.0;
      }
      
      marker_array.markers.push_back(marker);
    }
    marker_pub_->publish(marker_array);
    detection_pub_->publish(pose_array);
  }
}

int main(int argc, char ** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sort_mpc_nav::PedestrianSim>());
  rclcpp::shutdown();
  return 0;
}
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

    RCLCPP_INFO(get_logger(), "Pedestian simulation started with %zu pedestrians", pedestrians_.size());
  }

  void PedestrianSim::update(){
    for(size_t i = 0; i < pedestrians_.size(); ++i){
      // looping through the movements based on the the relation based on the velocity and original position
      // new position = old + instant velocity
      pedestrians_[i].x += pedestrians_[i].vx * 0.1; 
      pedestrians_[i].y += pedestrians_[i].vy * 0.1;

      geometry_msgs::msg::Point msg;
      msg.x = pedestrians_[i].x;
      msg.y = pedestrians_[i].y;

      publishers_[i]->publish(msg);
    }
  }
}

int main(int argc, char ** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sort_mpc_nav::PedestrianSim>());
  rclcpp::shutdown();
  return 0;
}
#ifndef SORT_MPC_NAV__PEDESTRIAN_SIM_HPP_
#define SORT_MPC_NAV__PEDESTRIAN_SIM_HPP_

namespace sort_mpc_nav{
    struct Pedestrian{
        double x, y;
        double vx, vy;
    }

    class Pedestrian : public rclcpp::Node{
        public: 
            PedestrianSim()
        private:
            void update():

            std::vector<Pedestrian> pedestrians_; // listening to the pedestrian struct from above
            std::vector<rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr> publishers_; // publsiher per pedestrian
            rclcpp::TimerBase::SharedPtr timer_ // drives the whole program
    };
} // namespace sort_mpc_nav

#endif // SORT_MPC_NAV__PEDESTRIANS_SIM_HPP_

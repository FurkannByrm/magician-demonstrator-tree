#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "dsr_msgs2/srv/move_home.hpp"
#include <map>


#include "behaviortree_cpp/bt_factory.h"


class MagicianSubNode : public rclcpp::Node{

    public:

    MagicianSubNode(const std::string& node_name);
    
      BT::NodeStatus checkPos()const{ 
        if(is_home_flage_){

            RCLCPP_INFO(get_logger(),"SUCCESS");
            return BT::NodeStatus::SUCCESS;

        }
        else{
            RCLCPP_ERROR(get_logger(),"FAILURE");
            return BT::NodeStatus::FAILURE;
        }
    }

    
    private:
    
    bool is_home_flage_{true};

    std::map<std::string,double> home_joints_ = {
    {"joint_1", 0.0},
    {"joint_2", 0.605},
    {"joint_3", 1.571},
    {"joint_4", 0.0},
    {"joint_5", 0.995},
    {"joint_6", 0.0}
    
    };
    const double JOINT_TOL = 0.01;
    void homePosCallBack(const sensor_msgs::msg::JointState::ConstSharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr home_axis_pos_;
    
};



class MagicianClientNode  : public rclcpp::Node{

    public:
    MagicianClientNode(const std::string& node_name);
    
    BT::NodeStatus homingServiceCall();
    BT::NodeStatus OpcUaServiceCall();

    private:
    rclcpp::Client<dsr_msgs2::srv::MoveHome>::SharedPtr homing_client_;
};

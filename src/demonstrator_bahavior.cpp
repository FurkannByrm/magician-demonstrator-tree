#include "magician_demonstrator_tree/demonstrator_behavior.hpp"

MagicianSubNode::MagicianSubNode(const std::string& node_name) : rclcpp::Node{node_name}{
    
    home_axis_pos_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states",10, std::bind(&MagicianSubNode::homePosCallBack, this,std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Magician Subscriber initialized");

}
void MagicianSubNode::homePosCallBack(const sensor_msgs::msg::JointState::ConstSharedPtr msg){

    is_home_flage_ = true;
    for(size_t i = 0; i < msg->name.size(); ++i)
    {
        const std::string& joint = msg->name[i];

        if(!home_joints_.count(joint))
            continue;

            double current = msg->position[i];
            double target = home_joints_[joint];
            
            if(std::fabs(target - current) > JOINT_TOL){
                
                is_home_flage_ = false;
                break;
            }
        }
        
        if (is_home_flage_){

       RCLCPP_INFO(get_logger(), "ROBOT HOME POSITION");
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Robot NOT home");
    }
    
    
}


MagicianClientNode::MagicianClientNode(const std::string& node_name) :  rclcpp::Node(node_name){

    homing_client_ = create_client<dsr_msgs2::srv::MoveHome>("/motion/move_home", 10);

    while(!homing_client_->wait_for_service(std::chrono::milliseconds(100))){
        if(!rclcpp::ok()){
            RCLCPP_ERROR(get_logger(),"Client intterrupted while waiting for service to appear");
            assert(0);
        }

        RCLCPP_INFO(get_logger(), "Waiting for service to appear...!");
    }

    RCLCPP_INFO(get_logger(),"Client created!!");
}


BT::NodeStatus MagicianClientNode::homingServiceCall(){

auto request = std::make_shared<dsr_msgs2::srv::MoveHome::Request>();
request->set__target(0);
auto future = homing_client_->async_send_request(request);
auto status = future.wait_for(std::chrono::seconds(10));

if(status == std::future_status::ready)
{
    auto response = future.get();

    if(response->success){
        RCLCPP_INFO(get_logger(),"HOMING SUCCESS");
        return BT::NodeStatus::SUCCESS;
    }
    else{
        RCLCPP_ERROR(get_logger(),"HOMING FAILED");
        return BT::NodeStatus::FAILURE;
    }
}
else
{
    RCLCPP_ERROR(get_logger(),"Homing service TIMEOUT");
    return BT::NodeStatus::FAILURE;
}


}


int main(int argc, char ** argv){
rclcpp::init(argc,argv);
    
    auto subNode = std::make_shared<MagicianSubNode>("home_node");
    auto clientNode = std::make_shared<MagicianClientNode>("homing_service_node");

    rclcpp::executors::MultiThreadedExecutor exe;

    exe.add_node(subNode);
    exe.add_node(clientNode);
    
    bool initialization_done = false;
    auto timer = subNode->create_wall_timer(
        std::chrono::seconds(4),
        [&]() {
            initialization_done = true;
        });
    
    while(!initialization_done && rclcpp::ok()) {
        exe.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    timer->cancel();  

    BT::BehaviorTreeFactory factory;
    factory.registerSimpleAction("IsRobotAtHome",[&](BT::TreeNode&){return subNode->checkPos();});
    factory.registerSimpleAction("CallHoming",[&](BT::TreeNode&){return clientNode->homingServiceCall();});

    auto tree = factory.createTreeFromFile("/home/cengo/jazzy_ws/src/magician_demonstrator_tree/config/bt_tree.xml");
    tree.tickWhileRunning();

    rclcpp::shutdown();
    return 0;
}

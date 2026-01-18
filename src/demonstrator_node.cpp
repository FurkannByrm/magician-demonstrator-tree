// #include "magician_demonstrator_tree/demonstrator_node.hpp"


// DemonstratorNode::DemonstratorNode(const std::string& node_name) : Node(node_name){
    
//     RCLCPP_INFO(get_logger(), "Setting up");

//     create_behavior_tree();

//     RCLCPP_INFO(get_logger(), "BT created");

//     rclcpp::spin(shared_from_this());
    
//     rclcpp::shutdown();

// } 

// void DemonstratorNode::create_behavior_tree(){
    
//     BT::BehaviorTreeFactory factory;

//     BT::NodeBuilder builder = 
//                            [=](const std::string&name, const BT::NodeConfiguration& config){
//         return std::make_unique<IsRobotAtHome>(name,config, shared_from_this());
//     };

//     factory.registerBuilder<IsRobotAtHome>("IsRobotAtHome",builder);

//     tree_ = factory.createTreeFromFile("/home/cengo/jazzy_ws/src/magician_demonstrator_tree/config/bt_tree.xml");


// }




// int main(int argc, char **argv){
    
//     rclcpp::init(argc,argv);
//     auto node = std::make_shared<DemonstratorNode>("demostrate_node");

//     BT::NodeStatus status = BT::NodeStatus::RUNNING;
    
//     while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
//         status = node->tree_.tickRoot();
        
//         std::this_thread::sleep_for(std::chrono::milliseconds(100));
//     }
    
//     return 0;
// }
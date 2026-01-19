#include "magician_demonstrator_tree/demonstrator_behavior.hpp"




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

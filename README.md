# Magician Demonstrator Tree

A ROS 2 package demonstrating behavior tree integration with the Doosan Magician robot for intelligent home position management and service orchestration.

## Overview

This package implements a behavior tree-based system that monitors the robot's position and automatically calls homing services when needed. It showcases the integration of ROS 2 subscriptions, service clients, and BehaviorTree.CPP for creating reactive robotic behaviors.

## Features

- **Home Position Monitoring**: Continuously tracks joint states to determine if the robot is at home position
- **Automatic Homing**: Triggers homing service when robot is not at home position
- **Behavior Tree Architecture**: Uses BehaviorTree.CPP for flexible decision-making logic
- **Multi-threaded Execution**: Concurrent processing of callbacks and behavior tree logic
- **Service Integration**: Demonstrates async service calls with timeout handling

## System Architecture

### Nodes

#### 1. MagicianSubNode
- **Purpose**: Monitors robot joint states and determines home position status
- **Subscriptions**: 
  - `/joint_states` (sensor_msgs/JointState)
- **Functionality**: 
  - Compares current joint positions with predefined home positions
  - Tolerance: ±0.01 radians per joint
  - Updates home status flag in real-time

#### 2. MagicianClientNode
- **Purpose**: Handles service calls for robot homing
- **Clients**:
  - `/motion/move_home` (dsr_msgs2/srv/MoveHome)
- **Functionality**:
  - Sends asynchronous homing requests
  - Waits up to 10 seconds for service response
  - Returns BehaviorTree status based on service success/failure

### Behavior Tree Structure

```xml
<Fallback>
    <IsRobotAtHome/>    <!-- Check if robot is at home position -->
    <CallHoming/>        <!-- If not, call homing service -->
</Fallback>
```

**Logic Flow**:
1. Check if robot is at home position
   - If YES → Return SUCCESS (tree completes)
   - If NO → Proceed to next node
2. Call homing service
   - Wait for service response
   - Return SUCCESS/FAILURE based on result

### Home Position Definition

| Joint | Target Position (rad) |
|-------|----------------------|
| joint_1 | 0.0 |
| joint_2 | 0.605 |
| joint_3 | 1.571 |
| joint_4 | 0.0 |
| joint_5 | 0.995 |
| joint_6 | 0.0 |

## Dependencies

- ROS 2 Jazzy
- [Doosan Robot2](https://github.com/DoosanRobotics/doosan-robot2)
- BehaviorTree.CPP v4.x
- rclcpp
- sensor_msgs
- std_srvs
- dsr_msgs2

## Installation

### Prerequisites

1. **Install ROS 2 Jazzy**:
```bash
# Follow official ROS 2 installation guide
```

2. **Install Doosan Robot2 packages**:
```bash
cd ~/jazzy_ws/src
git clone https://github.com/DoosanRobotics/doosan-robot2.git
cd ~/jazzy_ws
colcon build --packages-select dsr_msgs2
source install/setup.bash
```

3. **Install BehaviorTree.CPP**:
```bash
sudo apt install ros-jazzy-behaviortree-cpp
```

### Build

```bash
cd ~/jazzy_ws/src
git clone <this-repository>
cd ~/jazzy_ws
colcon build --packages-select magician_demonstrator_tree
source install/setup.bash
```

## Usage

### 1. Launch Doosan Robot

```bash
# Start the robot controller (refer to doosan-robot2 documentation)
ros2 launch dsr_launcher2 dsr_bringup.launch.py
```

### 2. Run Behavior Tree Node

```bash
ros2 run magician_demonstrator_tree behavior_node
```

### Expected Output

**Robot at Home Position**:
```
[INFO] [home_node]: Magician Subscriber initialized
[INFO] [home_node]: ROBOT HOME POSITION
[INFO] [home_node]: SUCCESS
```

**Robot NOT at Home Position**:
```
[INFO] [home_node]: Robot NOT home
[INFO] [home_node]: FAILURE
[INFO] [homing_service_node]: Client created!!
[INFO] [homing_service_node]: HOMING SUCCESS
```

## Configuration

### Modify Home Position

Edit `demonstrator_behavior.hpp`:

```cpp
std::map<std::string,double> home_joints_ = {
    {"joint_1", 0.0},
    {"joint_2", 0.605},
    {"joint_3", 1.571},
    // ... modify as needed
};
```

### Adjust Joint Tolerance

```cpp
const double JOINT_TOL = 0.01;  // radians
```

### Change Initialization Time

In `main.cpp`:

```cpp
auto timer = subNode->create_wall_timer(
    std::chrono::seconds(4),  // Modify duration here
    [&]() { initialization_done = true; }
);
```

## Extending the System

### Adding OPC-UA Integration

The package includes a placeholder for OPC-UA service integration:

```cpp
BT::NodeStatus MagicianClientNode::OpcUaServiceCall(){
    // TODO: Implement OPC-UA client logic
    // 1. Create OPC-UA service client
    // 2. Send request
    // 3. Handle response
    // 4. Return BT::NodeStatus
}
```

**XML Configuration** (commented in `bt_tree.xml`):

```xml
<Sequence name="magician_sequence">
    <Fallback>
        <IsRobotAtHome name="check_home_pos"/>
        <CallHoming name="call_homing_service"/>
    </Fallback>
    <CallOpcUI name="call_opcua_service"/>
</Sequence>
```

## Technical Details

### Multi-threaded Execution Model

```cpp
rclcpp::executors::MultiThreadedExecutor exe;
exe.add_node(subNode);      // Handles /joint_states subscription
exe.add_node(clientNode);   // Handles /move_home service client

// Initialization phase: spin for 4 seconds
while(!initialization_done && rclcpp::ok()) {
    exe.spin_some();  // Process callbacks
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

// Execution phase: run behavior tree
tree.tickWhileRunning();
```

**Note**: The current implementation uses `tickWhileRunning()` which blocks. For continuous callback processing during tree execution, consider using a background spinner thread:

```cpp
std::thread spinner([&exe]() { exe.spin(); });
tree.tickWhileRunning();
exe.cancel();
spinner.join();
```

### Service Call Pattern

```cpp
// Async request with timeout
auto future = homing_client_->async_send_request(request);
auto status = future.wait_for(std::chrono::seconds(10));

if(status == std::future_status::ready) {
    auto response = future.get();
    return response->success ? BT::NodeStatus::SUCCESS 
                             : BT::NodeStatus::FAILURE;
}
return BT::NodeStatus::FAILURE;  // Timeout
```

## Troubleshooting

### Issue: "Client interrupted while waiting for service"

**Solution**: Ensure Doosan robot controller is running:
```bash
ros2 service list | grep move_home
```

### Issue: "Robot NOT home" persists

**Causes**:
1. Joint state topic not publishing
2. Joint names mismatch
3. Position tolerance too tight

**Debug**:
```bash
ros2 topic echo /joint_states
ros2 topic hz /joint_states
```

### Issue: Behavior tree doesn't update after homing

**Cause**: `tickWhileRunning()` blocks callback processing.

**Solution**: Implement background spinner (see Technical Details).

## Future Improvements

- [ ] Add configurable YAML parameter file for home positions
- [ ] Implement OPC-UA service integration
- [ ] Add visualization with Groot2
- [ ] Implement StatefulActionNode for better async handling
- [ ] Add ROS 2 parameters for tolerances and timeouts
- [ ] Create launch file with configurable arguments
- [ ] Add unit tests for behavior tree logic

## Contributing

Contributions are welcome! Please ensure:
- Code follows ROS 2 style guidelines
- Behavior tree XML is well-documented
- Service calls include proper error handling

## License

TODO: License declaration

## Author

Maintainer: cengo (frknbyrm05@gmail.com)

## Acknowledgments

- [Doosan Robotics](https://github.com/DoosanRobotics) for the robot driver
- [BehaviorTree.CPP](https://www.behaviortree.dev/) community for the framework
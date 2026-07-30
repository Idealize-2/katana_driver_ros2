#include <atomic>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("moveit_pose_mover");
    
    // Use a separate thread to spin the executor so we can use blocking calls
    // (e.g. std::getline) in the main thread for user input.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    auto spin_thread = std::thread([&executor]() { executor.spin(); });
    auto shutdown = [&]() {
        executor.cancel();
        spin_thread.join();
        rclcpp::shutdown();
    };

    while(rclcpp::ok()) {
       
    }

    shutdown();
    return 0;
}

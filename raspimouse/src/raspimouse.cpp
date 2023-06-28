// Copyright 2018 Geoffrey Biggs, AIST
// The above lines denote the copyright for this code.

#include "raspimouse/raspimouse_component.hpp"
// This line includes the header file for the "raspimouse" component.

#include <memory>
#include <rclcpp/rclcpp.hpp>
// These lines include standard C++ memory and the main ROS 2 library.

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // This line initializes the ROS 2 communication system. It's necessary for every ROS 2 program.

  rclcpp::executors::SingleThreadedExecutor exe;
  // This line creates a single-threaded executor. The executor is responsible for task management. It will handle the execution of tasks for this ROS 2 node.

  rclcpp::NodeOptions options;
  // Create an instance of NodeOptions, which can be used to configure the node during its construction.

  std::shared_ptr<raspimouse::Raspimouse> raspimouse_node =
    std::make_shared<raspimouse::Raspimouse>(options);
  // This line creates a new instance of the "Raspimouse" node using a shared_ptr. This is the main part of the node where all the logic for controlling the Raspberry Pi Mouse will be located.

  exe.add_node(raspimouse_node->get_node_base_interface());
  // This line tells the executor to handle tasks for this node.

  exe.spin();
  // This line starts the executor to begin processing tasks, effectively starting the ROS 2 node. It will keep the node running and process incoming tasks and handle events.

  rclcpp::shutdown();
  // This line shuts down the ROS 2 communication system. It should be called after your ROS 2 application is done executing.

}

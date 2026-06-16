#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include "uuv_interfaces/srv/generate_cone.hpp"

#include <optional>

using GenerateCone = uuv_interfaces::srv::GenerateCone;

class SampleViewPosesCone : public BT::StatefulActionNode
{
    public:
        SampleViewPosesCone(
            const std::string& name,
            const BT::NodeConfig& config,
            rclcpp::Node::SharedPtr node
        );

        static BT::PortsList providedPorts();

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Client<GenerateCone>::SharedPtr service_client_;
        std::optional<rclcpp::Client<GenerateCone>::FutureAndRequestId> future_;

};

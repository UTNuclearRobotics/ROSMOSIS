#include "demo_behaviors/behaviors/sample_view_poses_cone.hpp"

#include <chrono>

using namespace std::chrono_literals;

SampleViewPosesCone::SampleViewPosesCone(
    const std::string& name,
    const BT::NodeConfig& config,
    rclcpp::Node::SharedPtr node
    ):
        BT::StatefulActionNode(name,config),
        node_(node)
        {
            service_client_ = node_->create_client<GenerateCone>("generate_cone");
            RCLCPP_INFO(node_->get_logger(), "SampleViewPosesCone client ready");
        }

 BT::PortsList SampleViewPosesCone::providedPorts(){
    return {
                BT::InputPort<int>("num_rings"),
                BT::InputPort<double>("clearance"),
                BT::InputPort<double>("cone_height"),
                BT::InputPort<double>("delta_theta"),
                BT::InputPort<double>("mount_angle_deg"),
                BT::OutputPort<geometry_msgs::msg::PoseArray>("view_poses")};
 }

 BT::NodeStatus SampleViewPosesCone::onStart() {

    // Validate all input ports
    auto num_rings = getInput<int>("num_rings");
    if (!num_rings.has_value()) {
        RCLCPP_ERROR(node_->get_logger(), "num_rings is not set on blackboard!");
        return BT::NodeStatus::FAILURE;
    }

    auto clearance = getInput<double>("clearance");
    if (!clearance.has_value()) {
        RCLCPP_ERROR(node_->get_logger(), "clearance is not set on blackboard!");
        return BT::NodeStatus::FAILURE;
    }

    auto cone_height = getInput<double>("cone_height");
    if (!cone_height.has_value()) {
        RCLCPP_ERROR(node_->get_logger(), "cone_height is not set on blackboard!");
        return BT::NodeStatus::FAILURE;
    }

    auto delta_theta = getInput<double>("delta_theta");
    if (!delta_theta.has_value()) {
        RCLCPP_ERROR(node_->get_logger(), "delta_theta is not set on blackboard!");
        return BT::NodeStatus::FAILURE;
    }

    auto mount_angle_deg = getInput<double>("mount_angle_deg");
    if (!mount_angle_deg.has_value()) {
        RCLCPP_ERROR(node_->get_logger(), "mount_angle_deg is not set on blackboard!");
        return BT::NodeStatus::FAILURE;
    }
    // validation block

    //request

    auto req = std::make_shared<GenerateCone::Request>();
    req->num_rings = num_rings.value();
    req->clearance = clearance.value();
    req->cone_height = cone_height.value();
    req->delta_theta_deg = delta_theta.value();
    req->mount_angle_deg = mount_angle_deg.value();

    future_ = service_client_->async_send_request(req);

    return BT::NodeStatus::RUNNING;
}

 BT::NodeStatus SampleViewPosesCone::onRunning() {

    if (!future_.has_value()){
        RCLCPP_ERROR(node_->get_logger(), "onRunning called without an active request!");
        return BT::NodeStatus::FAILURE;
    }

    if (future_->wait_for(0ms) != std::future_status::ready) {
        return BT::NodeStatus::RUNNING;
    }

    auto resp = future_->get();

    // Check we got something back
    if (!resp) {
        RCLCPP_ERROR(node_->get_logger(), "Cone service returned null response");
        future_ = std::nullopt;
        return BT::NodeStatus::FAILURE;
    }

    // Check poses array is not empty
    if (resp->poses.poses.empty()) {
        RCLCPP_WARN(node_->get_logger(), "Cone service returned no poses");
        future_ = std::nullopt;
        return BT::NodeStatus::FAILURE;
    }


    setOutput<geometry_msgs::msg::PoseArray>("view_poses", resp->poses);
    RCLCPP_INFO(node_->get_logger(), "Generated %zu cone poses", resp->poses.poses.size());

    future_ = std::nullopt;
    return BT::NodeStatus::SUCCESS;
}


void SampleViewPosesCone::onHalted() {
    if (future_.has_value()) {
        service_client_->remove_pending_request(future_.value());
    }
    future_ = std::nullopt;
}

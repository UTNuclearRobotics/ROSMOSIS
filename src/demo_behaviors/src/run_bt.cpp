#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <fstream>
#include <filesystem>
#include <thread>

// Demo behaviors (custom for this mission)
#include "demo_behaviors/behaviors/detect_and_sort_queue.hpp"
#include "demo_behaviors/behaviors/is_queue_not_empty.hpp"
#include "demo_behaviors/behaviors/sample_view_poses_helix.hpp"
#include "demo_behaviors/behaviors/sample_view_poses_cone.hpp"
#include "demo_behaviors/behaviors/dubins_client.hpp"
#include "demo_behaviors/behaviors/path_length_client.hpp"
#include "demo_behaviors/behaviors/mark_target_complete.hpp"
#include "demo_behaviors/behaviors/get_next_search_pose.hpp"
#include "demo_behaviors/behaviors/is_within_time_limit.hpp"

// Sample NBV behaviors (nbv_behaviors package - the sample_nbv_behaviors directory)
// Each behavior creates its own internal node; no shared node arg needed at registration.
#include "nbv_behaviors/set_roi_extents.hpp"
#include "nbv_behaviors/activate_policy.hpp"
#include "nbv_behaviors/set_views.hpp"
#include "nbv_behaviors/check_score_saturation.hpp"
#include "nbv_behaviors/get_best_view.hpp"
#include "nbv_behaviors/get_best_view_with_cost.hpp"
#include "nbv_behaviors/calculate_planning_pose.hpp"
#include "nbv_behaviors/conclude_policy.hpp"
#include "nbv_behaviors/save_model_to_file.hpp"

// nrg_behaviors - convenience registration covers PublishTransform, RepeatWhile, etc.
// Templated behaviors (like MakeShared<T>) must be registered manually with a specific type.
#include <nrg_behaviors/nrg_behaviors.hpp>
#include <nrg_utility_behaviors/make_shared.hpp>


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // One shared ROS2 node, passed by SharedPtr to every behavior that needs ROS access
  // (clients, subscriptions, TF, etc.). Behaviors create their clients on this node and
  // poll futures non-blockingly via wait_for(0ms); the jthread below does the actual
  // executor work that resolves those futures and fires subscription callbacks.
  auto node = std::make_shared<rclcpp::Node>("demo_bt");

  // Per-run reconstruction output directory, supplied by the launch (built from
  // the same bag_prefix + timestamp as the rosbag, so reconstructions and the bag
  // share a run label). Seeded onto the tree blackboard below so SaveModelToFile
  // in NBVOnTarget writes each target's mesh under this run's folder.
  node->declare_parameter("reconstruction_dir", std::string("data/reconstructions/default"));
  const std::string reconstruction_dir = node->get_parameter("reconstruction_dir").as_string();

  // Two-thread architecture:
  //   - Main thread (this one): blocks below in tree.tickWhileRunning(), ticking BT nodes
  //     and polling futures with wait_for(0ms). It never spins the executor itself.
  //   - jthread (below): continuously runs executor->spin_some() to dispatch all incoming
  //     ROS2 work — service responses, action feedback/results, subscription messages,
  //     timer callbacks, TF updates — for the shared node.
  //
  // The BT thread fires async requests (outgoing) directly from its tick; the jthread
  // executor handles the responses (incoming) and fulfills the corresponding futures.
  //
  // std::jthread (C++20) gives us RAII: on destruction it signals stop_token and joins,
  // so no manual cleanup is needed when main exits.
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  std::jthread spin_thread([&executor](std::stop_token stop_token) {
    while (!stop_token.stop_requested()) {
      executor->spin_some();  // process all currently-ready callbacks, then loop
    }
  });

  // Handles creation of Behavior Tree and registration of Behaviors
  BT::BehaviorTreeFactory factory;

  // Register custom behaviors. Behaviors that need ROS access take the shared node as
  // a 3rd constructor arg (forwarded by the factory). Behaviors that are pure C++ logic
  // (no ROS) don't need it. The shared node lets every behavior plug into the same
  // executor (the jthread above) without each one owning its own ROS2 node.
  factory.registerNodeType<DetectAndSortQueue>("DetectAndSortQueue", node);
  factory.registerNodeType<isQueueNotEmpty>("isQueueNotEmpty");
  factory.registerNodeType<SampleViewPosesHelix>("SampleViewPosesHelix", node);
  factory.registerNodeType<SampleViewPosesCone>("SampleViewPosesCone", node);
  factory.registerNodeType<DubinsClient>("DubinsClient", node);
  factory.registerNodeType<PathLengthClient>("PathLengthClient", node);
  factory.registerNodeType<MarkTargetComplete>("MarkTargetComplete");
  factory.registerNodeType<GetNextSearchPose>("GetNextSearchPose", node);
  factory.registerNodeType<isWithinTimeLimit>("isWithinTimeLimit", node);

  // Sample NBV behaviors - each owns an internal node + uses spin_until_future_complete.
  // Different threading model from our shared-node behaviors, but coexists fine.
  factory.registerNodeType<nbv_behaviors::SetROIExtents>("SetROIExtents");
  factory.registerNodeType<nbv_behaviors::ActivatePolicy>("ActivatePolicy");
  factory.registerNodeType<nbv_behaviors::SetViews>("SetViews");
  factory.registerNodeType<nbv_behaviors::CheckScoreSaturation>("CheckScoreSaturation");
  factory.registerNodeType<nbv_behaviors::GetBestView>("GetBestView");
  factory.registerNodeType<nbv_behaviors::GetBestViewWithCost>("GetBestViewWithCost");
  factory.registerNodeType<nbv_behaviors::CalculatePlanningPose>("CalculatePlanningPose");
  factory.registerNodeType<nbv_behaviors::ConcludePolicy>("ConcludePolicy");
  factory.registerNodeType<nbv_behaviors::SaveModelToFile>("SaveModelToFile");

  // nrg_behaviors - one call registers PublishTransform, RepeatWhile, etc.
  // Pass our shared node so those behaviors use the same executor (jthread above).
  nrg_behaviors::Config nrg_config;
  nrg_config.ros_node = node;
  nrg_behaviors::registerBehaviors(factory, nrg_config);

  // Templated nrg behaviors need explicit type registration.
  // MakeShared<PoseStamped> wraps current_pose into a SharedPtr for PublishTransform.
  factory.registerNodeType<nrg_utility_behaviors::MakeShared<geometry_msgs::msg::PoseStamped>>("MakeShared");

  // Register BT XML files. Register both main_tree.xml (root) and nbv_on_target.xml
  // (subtree referenced by main via <SubTree ID="NBVOnTarget">).
  std::string share_path = ament_index_cpp::get_package_share_directory(
    "demo_behaviors");
  factory.registerBehaviorTreeFromFile(share_path + "/behavior_trees/nbv_on_target.xml");
  factory.registerBehaviorTreeFromFile(share_path + "/behavior_trees/bayesian_search.xml");
  factory.registerBehaviorTreeFromFile(share_path + "/behavior_trees/main_tree.xml");

  // ---- Write Behaviors into TreeNodes Model ---- //
  std::string model_str = BT::writeTreeNodesModelXML(factory);

  // Create path to TreeNodesModel file
  std::filesystem::path package_root = std::filesystem::path(__FILE__).parent_path().parent_path();
  std::filesystem::path models_path = package_root / "config" / "models.xml";

  // Write XML to file
  std::ofstream models_file(models_path);
  if (models_file.is_open()) {
    models_file << model_str;
    models_file.close();
    RCLCPP_INFO(node->get_logger(), "Wrote TreeNodesModel to: %s", models_path.c_str());
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to write TreeNodesModel to: %s", models_path.c_str());
  }

  // ---- Create and run Behavior Tree ---- //
  // MainTree is the root; it has one NBVOnTarget subtree that gets re-ticked
  // each loop iteration with a different target (via blackboard autoremap).
  auto tree = factory.createTree("MainTree");

  // Seed the per-run reconstruction directory onto the root blackboard; the
  // NBVOnTarget subtree (_autoremap="true") reads it as {reconstruction_dir}.
  tree.rootBlackboard()->set("reconstruction_dir", reconstruction_dir);

  BT::Groot2Publisher publisher(tree); // Publish to Groot2 to view BT on Port 1667

  // tickWhileRunning() blocks the main thread, repeatedly ticking the tree until it
  // returns SUCCESS or FAILURE. While this runs:
  //   - main thread: tick BT, peek futures (wait_for(0ms)), set/get blackboard entries
  //   - jthread: spin executor, deliver responses, fire subscription callbacks
  // The two threads coordinate through the shared node and async futures.
  tree.tickWhileRunning();

  // Cancel the executor explicitly (cooperative shutdown). The jthread's destructor
  // will then signal stop_token and join automatically (RAII), so the spin loop exits
  // cleanly before main returns.
  executor->cancel();
  rclcpp::shutdown();
  return 0;
}
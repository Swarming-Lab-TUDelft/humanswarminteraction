/* TU Delft Swarm Lab - Human Swarm Interaction Project
 * Description: Track and filter a single keypoint linearly.
 * Author: Alexander James Becoy
 * Date: 2024-10-28
 */

#include <keypoints_tracking/single_keypoint_tracker_node.hpp>

using namespace HumanSwarmInteraction;

SingleKeypointTrackerNode::SingleKeypointTrackerNode()
: Node("single_keypoint_tracker_node")
{
  configureParameters();
  initializeKeyPointsSubscriber();
  initializeKeyPointsPublisher();
  RCLCPP_INFO(get_logger(), "Single Keypoint Tracker Node initialized.");
}

SingleKeypointTrackerNode::~SingleKeypointTrackerNode()
{
  RCLCPP_WARN(get_logger(), "Single Keypoint Tracker Node destroyed.");
}

void SingleKeypointTrackerNode::keypointsCallback(
  const human_swarm_interaction_interfaces::msg::PoseKeypointsStamped::SharedPtr msg)
{
}

void SingleKeypointTrackerNode::publishKeypoints() const
{
}

void SingleKeypointTrackerNode::initializeKeyPointsSubscriber()
{
  this->declare_parameter("input_keypoints_topic_name", "all_keypoints");
  this->declare_parameter("input_keypoints_topic_queue_size", 10);

  std::string input_keypoints_topic_name = this->get_parameter("input_keypoints_topic_name").as_string();
  int input_keypoints_topic_queue_size = this->get_parameter("input_keypoints_topic_queue_size").as_int();
  RCLCPP_INFO(get_logger(), "Subscribing to topic: %s with queue size: %d", 
    input_keypoints_topic_name.c_str(), input_keypoints_topic_queue_size);

  m_keypoints_sub = this->create_subscription<human_swarm_interaction_interfaces::msg::PoseKeypointsStamped>(
    input_keypoints_topic_name, input_keypoints_topic_queue_size, 
    std::bind(&SingleKeypointTrackerNode::keypointsCallback, this, std::placeholders::_1));
}

void SingleKeypointTrackerNode::initializeKeyPointsPublisher()
{
  this->declare_parameter("output_keypoints_topic_name", "filtered_keypoints");
  this->declare_parameter("output_keypoints_topic_queue_size", 10);

  std::string output_keypoints_topic_name = this->get_parameter("output_keypoints_topic_name").as_string();
  int output_keypoints_topic_queue_size = this->get_parameter("output_keypoints_topic_queue_size").as_int();
  RCLCPP_INFO(get_logger(), "Publishing to topic: %s with queue size: %d", 
    output_keypoints_topic_name.c_str(), output_keypoints_topic_queue_size);

  m_keypoints_pub = this->create_publisher<human_swarm_interaction_interfaces::msg::PoseKeypointsStamped>(
    output_keypoints_topic_name, output_keypoints_topic_queue_size);
}

void SingleKeypointTrackerNode::configureParameters()
{
  this->declare_parameter("keypoint_name", "right_wrist");
  m_keypoint_name = this->get_parameter("keypoint_name").as_string();
  RCLCPP_INFO(get_logger(), "Selected keypoint name: %s", m_keypoint_name.c_str());
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SingleKeypointTrackerNode>());
  rclcpp::shutdown();
  return 0;
}

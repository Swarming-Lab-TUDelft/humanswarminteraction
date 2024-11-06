/* TU Delft Swarm Lab - Human Swarm Interaction Project
 * Description: Track and filter multiple keypoints linearly.
 * Author: Alexander James Becoy
 * Date: 2024-11-06
 */

#include <keypoints_tracking/multiple_keypoints_tracker_node.hpp>

using namespace HumanSwarmInteraction;

MultipleKeypointsTrackerNode::MultipleKeypointsTrackerNode()
: Node("multiple_keypoints_tracking_node")
{
  configureParameters();
  initializeKeyPointsPublisher();
  initializeKeyPointsSubscriber();
  initializeTimer();
  RCLCPP_INFO(get_logger(), "Multiple Keypoints Tracker Node initialized.");
}

MultipleKeypointsTrackerNode::~MultipleKeypointsTrackerNode()
{
  RCLCPP_WARN(get_logger(), "Multiple Keypoints Tracker Node destroyed.");
}

void MultipleKeypointsTrackerNode::timerCallback()
{
  // TODO: Implement timer callback
}

void MultipleKeypointsTrackerNode::keypointsCallback(
  const human_swarm_interaction_interfaces::msg::PoseKeypointsStamped::SharedPtr msg)
{
  // TODO: Implement keypoints callback
}

void MultipleKeypointsTrackerNode::publishKeypoints() const
{
  // TODO: Implement publish keypoints
}

void MultipleKeypointsTrackerNode::initializeTimer()
{
  // Initialize timer
  this->declare_parameter("timer_period", 0.01);
  double timer_period = this->get_parameter("timer_period").as_double();

  m_timer = this->create_wall_timer(std::chrono::milliseconds((int)(timer_period * 1000)), 
    std::bind(&MultipleKeypointsTrackerNode::timerCallback, this));
  RCLCPP_INFO(get_logger(), "Timer initialized with period: %f", timer_period);
}

void MultipleKeypointsTrackerNode::initializeKeyPointsSubscriber()
{
  this->declare_parameter("input_keypoints_topic_name", "all_keypoints");
  this->declare_parameter("input_keypoints_topic_queue_size", 10);

  std::string input_keypoints_topic_name = this->get_parameter("input_keypoints_topic_name").as_string();
  int input_keypoints_topic_queue_size = this->get_parameter("input_keypoints_topic_queue_size").as_int();
  RCLCPP_INFO(get_logger(), "Subscribing to topic: %s with queue size: %d", 
    input_keypoints_topic_name.c_str(), input_keypoints_topic_queue_size);

  m_keypoints_sub = this->create_subscription<human_swarm_interaction_interfaces::msg::PoseKeypointsStamped>(
    input_keypoints_topic_name, input_keypoints_topic_queue_size, 
    std::bind(&MultipleKeypointsTrackerNode::keypointsCallback, this, std::placeholders::_1));
}

void MultipleKeypointsTrackerNode::initializeKeyPointsPublisher()
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

void MultipleKeypointsTrackerNode::configureParameters()
{
  // Parameterize the keypoint names to track
  this->declare_parameter("keypoint_names", std::vector<std::string>{"right_wrist", "left_wrist"});
  m_keypoint_names = this->get_parameter("keypoint_names").as_string_array();
  RCLCPP_INFO(get_logger(), "Selected keypoint names: ");
  for (const auto& keypoint_name : m_keypoint_names) {
    RCLCPP_INFO(get_logger(), keypoint_name.c_str());
  }

  // Parameterize the tracking window dimensions
  this->declare_parameter("tracking_window.width", 0.1);
  this->declare_parameter("tracking_window.height", 0.1);
  m_tracking_window_width = this->get_parameter("tracking_window.width").as_double();
  m_tracking_window_height = this->get_parameter("tracking_window.height").as_double();
  RCLCPP_INFO(get_logger(), "Tracking window width: %f, height: %f", 
    m_tracking_window_width, m_tracking_window_height);

  // Parameterize the tracking window scaling to confidence
  this->declare_parameter("tracking_window_scaling_to_confidence.x", 1.0);
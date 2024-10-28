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
  initializeLinearKalmanFilter();
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
  m_last_msg_header = &(msg->header);

  // try {
  //   double x, y;
  //   for (const auto& keypoint : msg->keypoints) {
  //     if (keypoint.name == m_keypoint_name) {
  //       x = keypoint.x;
  //       y = keypoint.y;
  //       break;
  //     }
  //   }
  // }
}

void SingleKeypointTrackerNode::publishKeypoints(double x, double y) const
{
  // Create PoseKeypointsStamped message and update header to match the last received message.
  human_swarm_interaction_interfaces::msg::PoseKeypointsStamped msg;
  msg.header = *m_last_msg_header;

  // Update the keypoint with the filtered 2D position.
  human_swarm_interaction_interfaces::msg::PoseKeypoint keypoint;
  keypoint.name = m_keypoint_name;
  keypoint.x = x;
  keypoint.y = y;
  msg.keypoints.push_back(keypoint);

  m_keypoints_pub->publish(msg);
}

void SingleKeypointTrackerNode::initializeLinearKalmanFilter()
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
  // Parameterize the keypoint name to track
  this->declare_parameter("keypoint_name", "right_wrist");
  m_keypoint_name = this->get_parameter("keypoint_name").as_string();
  RCLCPP_INFO(get_logger(), "Selected keypoint name: %s", m_keypoint_name.c_str());

  // Parameterize the tracking window dimensions
  this->declare_parameter("tracking_window_width", 0.1);
  this->declare_parameter("tracking_window_height", 0.1);
  m_tracking_window_width = this->get_parameter("tracking_window_width").as_double();
  m_tracking_window_height = this->get_parameter("tracking_window_height").as_double();
  RCLCPP_INFO(get_logger(), "Tracking window width: %f, height: %f", 
    m_tracking_window_width, m_tracking_window_height);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SingleKeypointTrackerNode>());
  rclcpp::shutdown();
  return 0;
}

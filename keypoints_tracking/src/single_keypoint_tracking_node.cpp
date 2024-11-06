/* TU Delft Swarm Lab - Human Swarm Interaction Project
 * Description: Track and filter a single keypoint linearly.
 * Author: Alexander James Becoy
 * Date: 2024-10-28
 */

#include <keypoints_tracking/single_keypoint_tracker_node.hpp>

using namespace HumanSwarmInteraction;

SingleKeypointTrackerNode::SingleKeypointTrackerNode()
: Node("single_keypoint_tracking_node")
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

void SingleKeypointTrackerNode::timerCallback()
{
  // auto centroid = m_kalman_filter->getCentroidCoordinate();
  // publishKeypoints(centroid[0], centroid[1]);

  // Publish moving average window
  if (m_x_moving_average_window.size() >= static_cast<size_t>(m_moving_average_window_size)) {
    double x_sum = 0.0;
    double y_sum = 0.0;
    for (int i = 0; i < m_moving_average_window_size; i++) {
      x_sum += m_x_moving_average_window[i];
      y_sum += m_y_moving_average_window[i];
    }

    publishKeypoints(x_sum / static_cast<double>(m_moving_average_window_size), 
      y_sum / static_cast<double>(m_moving_average_window_size));
  }
}

void SingleKeypointTrackerNode::keypointsCallback(
  const human_swarm_interaction_interfaces::msg::PoseKeypointsStamped::SharedPtr msg)
{
  try {
    for (const auto& keypoint : msg->keypoints) {
      if (keypoint.name == m_keypoint_name) {
        if (m_initialized) {
          m_kalman_filter->predict();
          m_kalman_filter->update(keypoint.x, keypoint.y);
        } else {
          m_kalman_filter->setInitialState(keypoint.x, keypoint.y, 0.0, 0.0);
          m_initialized = true;
        }

        // Update moving average window
        auto centroid = m_kalman_filter->getCentroidCoordinate();
        m_x_moving_average_window.push_back(centroid[0]);
        m_y_moving_average_window.push_back(centroid[1]);

        // Remove the oldest element if the window size is exceeded
        if (m_x_moving_average_window.size() > static_cast<size_t>(m_moving_average_window_size)) {
          m_x_moving_average_window.erase(m_x_moving_average_window.begin());
          m_y_moving_average_window.erase(m_y_moving_average_window.begin());
        }

        break;
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
      "Error in keypoint tracking: %s", e.what());
  }
}

void SingleKeypointTrackerNode::publishKeypoints(double x, double y) const
{
  // Create PoseKeypointsStamped message and update header to match the 
  // last received message.
  human_swarm_interaction_interfaces::msg::PoseKeypointsStamped msg;
  msg.header.frame_id = "camera_frame";
  msg.header.stamp = this->now();

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
  this->declare_parameter("initial_state", std::vector<double>());
  this->declare_parameter("covariances", std::vector<double>());
  this->declare_parameter("process_noises", std::vector<double>());
  this->declare_parameter("observation_noises", std::vector<double>());

  std::vector<double> initial_state = this->get_parameter("initial_state").as_double_array();
  std::vector<double> covariances = this->get_parameter("covariances").as_double_array();
  std::vector<double> process_noises = this->get_parameter("process_noises").as_double_array();
  std::vector<double> observation_noises = this->get_parameter("observation_noises").as_double_array();

  // Check if the parameters are of the correct size
  if (initial_state.size() != STATE_SIZE) {
    throw std::invalid_argument("Initial state must have size 8. Current size: " 
      + std::to_string(initial_state.size()));
  }

  if (covariances.size() != STATE_SIZE) {
    throw std::invalid_argument("Covariances must have size 8. Current size: " 
      + std::to_string(covariances.size()));
  }

  if (process_noises.size() != STATE_SIZE) {
    throw std::invalid_argument("Process noises must have size 8. Current size: " 
      + std::to_string(process_noises.size()));
  }

  if (observation_noises.size() != MEASUREMENT_SIZE) {
    throw std::invalid_argument("Observation noises must have size 4. Current size: " 
      + std::to_string(observation_noises.size()));
  }

  // Initialize timer
  this->declare_parameter("timer_period", 0.01);
  double timer_period = this->get_parameter("timer_period").as_double();

  m_timer = this->create_wall_timer(std::chrono::milliseconds((int)(timer_period * 1000)), 
    std::bind(&SingleKeypointTrackerNode::timerCallback, this));
  RCLCPP_INFO(get_logger(), "Timer initialized with period: %f", timer_period);

  // Initialize the Kalman filter
  m_kalman_filter = std::make_unique<LinearKalmanFilter>(initial_state, covariances, process_noises, observation_noises, timer_period);
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

  // Parameterize the moving average window size
  this->declare_parameter("moving_average_window_size", 5);
  m_moving_average_window_size = this->get_parameter("moving_average_window_size").as_int();
  RCLCPP_INFO(get_logger(), "Moving average window size: %d", m_moving_average_window_size);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SingleKeypointTrackerNode>());
  rclcpp::shutdown();
  return 0;
}

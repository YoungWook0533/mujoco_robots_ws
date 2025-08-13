/*!
 * @file     controller_ros.h
 * @author   Giuseppe Rizzi
 * @date     08.09.2020
 * @version  1.0
 * @brief    description
 */
#pragma once
#include <mppi/core/solver.h>
#include <mppi_ros/threading/WorkerManager.hpp>
#include "mppi_ros/msg/data.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <rclcpp/rclcpp.hpp>
#include <atomic>

using namespace mppi;
namespace mppi_ros {

class ControllerRos {
 public:
  ControllerRos() = delete;
  ControllerRos(std::shared_ptr<rclcpp::Node> node);
  virtual ~ControllerRos();

  virtual bool set_controller(std::shared_ptr<Solver>& controller) = 0;
  virtual bool update_reference() { return true; }
  virtual bool init_ros() { return true; }
  virtual void publish_ros() {}

  bool init();
  bool start();
  void stop();

  inline std::shared_ptr<Solver>& get_controller() { return controller_; }
  std::shared_ptr<rclcpp::Node> get_node() { return node_; }

  void set_observation(const observation_t& x, const double& t);
  bool update_policy();
  void get_input(const observation_t& x, input_t& u, const double& t);
  void get_input_state(const observation_t& x, observation_t& x_nom, input_t& u, const double& t);
  bool publish_ros_default();

  // Thread-safe helpers for derived classes
  bool get_optimal_rollout(mppi::Rollout& r);
  inline bool is_policy_ready() const { return policy_ready_.load(std::memory_order_acquire); }

 private:
  void init_default_ros();
  bool init_default_params();

  bool update_policy_thread(const mppi::threading::WorkerEvent& event);
  bool update_reference_thread(const mppi::threading::WorkerEvent& event);
  bool publish_ros_thread(const mppi::threading::WorkerEvent& event);

  void publish_stage_cost();
  void publish_rollout_cost();
  void publish_input();

 private:
  bool started_ = false;
  bool initialized_ = false;
  bool observation_set_ = false;

  // Serialize access to controller_ across threads/callbacks
  std::mutex controller_mutex_;

  mppi::threading::WorkerManager worker_manager_;

  mppi::cost_ptr cost_;
  mppi::dynamics_ptr dynamics_;
  mppi::config_t config_;

  std::shared_ptr<mppi::Solver> controller_ = nullptr;

  double policy_update_rate_ = 0.0;
  double reference_update_rate_ = 0.0;

  bool publish_ros_ = false;
  double ros_publish_rate_ = 0.0;

  // New: gate publishing until the first successful policy update completes
  std::atomic_bool policy_ready_{false};

 public:
  std::shared_ptr<rclcpp::Node> node_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cost_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr min_rollout_cost_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr max_rollout_cost_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr input_publisher_;
  rclcpp::Publisher<mppi_ros::msg::Data>::SharedPtr data_publisher_;

  std_msgs::msg::Float64 stage_cost_;
  std_msgs::msg::Float64 min_rollout_cost_;
  std_msgs::msg::Float64 max_rollout_cost_;

  std::shared_mutex input_mutex_;
  mppi::input_t input_ = mppi::input_t::Zero(1);
  std_msgs::msg::Float32MultiArray input_ros_;

  // logging
  mppi_ros::msg::Data data_ros_;
};

}  // namespace mppi_ros

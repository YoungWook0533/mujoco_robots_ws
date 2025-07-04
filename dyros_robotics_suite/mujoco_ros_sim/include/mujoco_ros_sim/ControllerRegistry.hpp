#pragma once
#include <unordered_map>
#include <functional>
#include <memory>
#include <string>
#include "mujoco_ros_sim/JointDict.hpp"
#include <rclcpp/rclcpp.hpp>

class ControllerInterface;
using JointDict = mujoco_ros_sim::JointDict;
using ControllerFactory =
        std::function<std::unique_ptr<ControllerInterface>(double, JointDict)>;

class ControllerRegistry
{
public:
  static ControllerRegistry& instance();
  void add(const std::string& name, ControllerFactory f)
  { map_[name] = std::move(f); }
  const auto& map() const { return map_; }
private:
  std::unordered_map<std::string, ControllerFactory> map_;
};

inline void ensure_rclcpp()
{
  if (!rclcpp::ok()) 
  {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }
}


#define REGISTER_MJ_CONTROLLER(CLS, NAME)                                       \
  namespace                                                                     \
  {                                                                             \
    std::unique_ptr<ControllerInterface> factory_##CLS(double dt, JointDict jd) \
    {                                                                           \
      ensure_rclcpp();                                                          \
      auto node = rclcpp::Node::make_shared(std::string(NAME));                 \
      return std::make_unique<CLS>(node, dt, std::move(jd));                    \
    }                                                                           \
    const bool registered_##CLS = [](){                                         \
      ControllerRegistry::instance().add(NAME, factory_##CLS);                  \
      return true;                                                              \
    }();                                                                        \
  }
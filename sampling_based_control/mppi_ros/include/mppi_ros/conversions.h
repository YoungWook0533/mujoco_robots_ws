//
// Created by giuseppe on 05.02.21.
//

#pragma once

#include <mppi/core/config.h>
#include <mppi/core/data.h>
#include <mppi/core/rollout.h>
#include <mppi/core/typedefs.h>

#include "mppi_ros/msg/array.hpp"
#include "mppi_ros/msg/config.hpp"
#include "mppi_ros/msg/data.hpp"
#include "mppi_ros/msg/rollout.hpp"

namespace mppi_ros {

void to_msg(const mppi::config_t& config, mppi_ros::msg::Config& config_ros);
void to_msg(const mppi::Rollout& rollout, mppi_ros::msg::Rollout& rollout_ros, const bool input_only=false, const size_t max_length=1000);
void to_msg(const mppi::data_t& data, mppi_ros::msg::Data& data_ros);

}  // namespace mppi_ros
// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#define FMT_HEADER_ONLY
#include <fmt/format.h>

#include <string>
#include <vector>

#include "autoware_state_monitor/autoware_state_monitor_node.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

void AutowareStateMonitorNode::setupDiagnosticUpdater()
{
  updater_.setHardwareID("autoware_state_monitor");

  this->declare_parameter("module_names");
  std::vector<std::string> module_names = this->get_parameter("module_names").as_string_array();

  // TF
  updater_.add(
    "localization_tf_status",
    std::bind(
      &AutowareStateMonitorNode::checkTFStatus, this, std::placeholders::_1,
      "localization"));
}

void AutowareStateMonitorNode::checkTFStatus(
  diagnostic_updater::DiagnosticStatusWrapper & stat, const std::string & module_name)
{
  int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;

  const auto & tf_stats = state_input_.tf_stats;

  // OK
  for (const auto & tf_config : tf_stats.ok_list) {
    if (tf_config.module != module_name) {
      continue;
    }

    const auto name = fmt::format("{}2{}", tf_config.from, tf_config.to);
    stat.add(fmt::format("{} status", name), "OK");
  }

  // Check tf received
  for (const auto & tf_config : tf_stats.non_received_list) {
    if (tf_config.module != module_name) {
      continue;
    }

    const auto name = fmt::format("{}2{}", tf_config.from, tf_config.to);
    stat.add(fmt::format("{} status", name), "Not Received");

    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }

  // Check tf timeout
  for (const auto & tf_config_pair : tf_stats.timeout_list) {
    const auto & tf_config = tf_config_pair.first;
    const auto & last_received_time = tf_config_pair.second;

    if (tf_config.module != module_name) {
      continue;
    }

    const auto name = fmt::format("{}2{}", tf_config.from, tf_config.to);
    stat.add(fmt::format("{} status", name), "Timeout");
    stat.addf(fmt::format("{} timeout", name), "%.2f [s]", tf_config.timeout);
    stat.addf(fmt::format("{} checked_time", name), "%.2f [s]", tf_stats.checked_time.seconds());
    stat.addf(fmt::format("{} last_received_time", name), "%.2f [s]", last_received_time.seconds());

    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }

  // Create message
  std::string msg;
  if (level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
    msg = "OK";
  } else if (level == diagnostic_msgs::msg::DiagnosticStatus::WARN) {
    msg = "Warn";
  } else if (level == diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
    msg = "Error";
  }

  stat.summary(level, msg);
}

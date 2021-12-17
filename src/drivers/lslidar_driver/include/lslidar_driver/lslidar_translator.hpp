// Copyright 2018-2020 the Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

/// \copyright Copyright 2017-2018 the Autoware Foundation
/// \file
/// \brief This file defines a driver for Lslidar LiDARs

#ifndef LSLIDAR_DRIVER__LSLIDAR_TRANSLATOR_HPP_
#define LSLIDAR_DRIVER__LSLIDAR_TRANSLATOR_HPP_

#include <lslidar_driver/visibility_control.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <lslidar_driver/common.hpp>
#include <lslidar_driver/ls16_data.hpp>
#include <lslidar_driver/ls32_data.hpp>
#include <lslidar_driver/ch128_data.hpp>
#include <lslidar_driver/ch64w_data.hpp>
#include <vector>

namespace autoware
{
namespace drivers
{
namespace lslidar_driver
{
/// \brief This class handles converting packets from a lslidar lidar into cartesian points.
/// \tparam SensorData Data class representing a specific sensor model.
template<typename SensorData>
class LSLIDAR_DRIVER_PUBLIC LslidarTranslator
{
public:
  /// \brief Stores basic configuration information, does some simple validity checking
  static constexpr uint16_t POINT_BLOCK_CAPACITY = 512U;

  class Config
  {
public:
    /// \brief Constructor
    /// \param[in] rpm rotation speed of the lslidar, determines how many points per scan
    explicit Config(const float32_t rpm)
    : m_rpm(rpm)
    {
    }

    float32_t get_rpm() const
    {
      return m_rpm;
    }

private:
    float32_t m_rpm;
  };

  /// \brief default constructor
  /// \param[in] config config struct with rpm, transform, radial and angle pruning params
  /// \throw std::runtime_error if pruning parameters are inconsistent
  explicit LslidarTranslator(const Config & config)
  : m_sensor_data(config.get_rpm())
  {
  }
	
  void unpack_dev(const std::vector<uint8_t> & buffer, std::size_t & m_cloud_size) 
  {
  	m_sensor_data.unpack_dev(buffer, m_cloud_size);
  }
	
  /// \brief Convert a packet into a block of cartesian points
  /// \param[in] pkt A packet from a LS16 HiRes sensor for conversion
  /// \param[out] output Gets filled with cartesian points and any additional flags
  void convert(const std::vector<uint8_t> & buffer, std::vector<autoware::common::types::PointXYZIF> & output)
  {
    output.clear();
	m_sensor_data.unpack(buffer, output);
  }

private:
  SensorData m_sensor_data;
  
};  // class Driver
using Ls16Translator = LslidarTranslator<LS16Data>;
using Ls32Translator = LslidarTranslator<LS32Data>;
using Ch128Translator = LslidarTranslator<CH128Data>;
using Ch64wTranslator = LslidarTranslator<CH64wData>;

}  // namespace lslidar_driver
}  // namespace drivers
}  // namespace autoware

#endif  // LSLIDAR_DRIVER__LSLIDAR_TRANSLATOR_HPP_

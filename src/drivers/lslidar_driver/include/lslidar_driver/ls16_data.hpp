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

/// \copyright Copyright 2018-2020 the Autoware Foundation
/// \file
/// \brief This file defines a driver for Lslidar LiDARs

#ifndef LSLIDAR_DRIVER__LS16_DATA_HPP_
#define LSLIDAR_DRIVER__LS16_DATA_HPP_

#include <lslidar_driver/visibility_control.hpp>
#include <lslidar_driver/common.hpp>
#include <cmath>
#include <utility>

namespace autoware
{
/// \brief Libraries, ROS nodes, and other functionality relating to
///         sensor drivers or actuation.
namespace drivers
{
/// \brief Classes, types, and definitions specifically relating to
///        Lslidar LiDARs. In it's current incarnation, we consider Lslidar to be synonymous
///        with LiDARs. In the future, this namespace will diverge to LiDAR and Lslidar for
///        general LiDAR point cloud functionality, and specific driver functionality for
///        velodne LiDARs respectively.
namespace lslidar_driver
{

/// Class implementing LS16 specific computation and caches
class LSLIDAR_DRIVER_PUBLIC LS16Data
{
public:
	static constexpr double DEG_TO_RAD = 0.017453292;
	static constexpr double RAD_TO_DEG = 57.29577951;
	static constexpr int SIZE_BLOCK = 100;
    static constexpr int RAW_SCAN_SIZE = 3;
    static constexpr int SCANS_PER_BLOCK = 32;
    static constexpr int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);  // 96

    static constexpr float ROTATION_RESOLUTION = 0.01f;   /**< degrees 旋转角分辨率*/
    static constexpr uint16_t ROTATION_MAX_UNITS = 36000; /**< hundredths of degrees */

    static constexpr float DISTANCE_MAX = 200.0f;            /**< meters */
    static constexpr float DISTANCE_MIN = 0.2f;              /**< meters */
    static constexpr float DISTANCE_RESOLUTION = 0.01f;      /**< meters */
    static constexpr float DISTANCE_RESOLUTION_NEW = 0.005f; /**< meters */
    static constexpr float DISTANCE_MAX_UNITS = (DISTANCE_MAX / DISTANCE_RESOLUTION + 1.0f);
/** @todo make this work for both big and little-endian machines */
    static constexpr uint16_t UPPER_BANK = 0xeeff;  //
    static constexpr uint16_t LOWER_BANK = 0xddff;

/** Special Defines for LSC16 support **/
    static constexpr int LSC16_FIRINGS_PER_BLOCK = 2;
    static constexpr int LSC16_SCANS_PER_FIRING = 16;
    static constexpr float LSC16_BLOCK_TDURATION = 100.0f;  // [µs]
    static constexpr float LSC16_DSR_TOFFSET = 3.125f;        // [µs]
    static constexpr float LSC16_FIRING_TOFFSET = 50.0f;    // [µs]
	static constexpr int TEMPERATURE_MIN = 31;
	static constexpr int PACKET_SIZE = 1206;
    static constexpr int BLOCKS_PER_PACKET = 12;
    static constexpr int PACKET_STATUS_SIZE = 4;
    static constexpr int SCANS_PER_PACKET = (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);
	
	typedef struct raw_block {
        uint16_t header;  ///< UPPER_BANK or LOWER_BANK
        uint8_t rotation_1;
        uint8_t rotation_2;  /// combine rotation1 and rotation2 together to get 0-35999, divide by 100 to get degrees
        uint8_t data[BLOCK_DATA_SIZE];  // 96
    } raw_block_t;
	
	union two_bytes {
        uint16_t uint;
        uint8_t bytes[2];
    };
	
// Pre-compute the sine and cosine for the altitude angles.
    double scan_altitude_original_2[16] = {
            -0.2617993877991494, 0.017453292519943295,
            -0.22689280275926285, 0.05235987755982989,
            -0.19198621771937624, 0.08726646259971647,
            -0.15707963267948966, 0.12217304763960307,
            -0.12217304763960307, 0.15707963267948966,
            -0.08726646259971647, 0.19198621771937624,
            -0.05235987755982989, 0.22689280275926285,
            -0.017453292519943295, 0.2617993877991494
    };
    double scan_altitude_original_1[16] = {
            -0.17453292519943295,0.01160643952576229,
            -0.15123277968530863,0.03490658503988659,
            -0.12793263417118436,0.05811946409141117,
            -0.10471975511965978,0.08141960960553547,
            -0.08141960960553547,0.10471975511965978,
            -0.05811946409141117,0.12793263417118436,
            -0.03490658503988659,0.15123277968530863,
            -0.01160643952576229,0.17453292519943295
    };

    double scan_altitude[16];
	
	typedef struct raw_packet {
        raw_block_t blocks[BLOCKS_PER_PACKET];
        uint16_t revolution;
        uint8_t status[PACKET_STATUS_SIZE];
    } raw_packet_t;

    union vertical_point {
        uint8_t uint[2];
        uint16_t value;
    };
	
	explicit LS16Data(const float32_t rpm);

	void unpack(const std::vector<uint8_t> & buffer, std::vector<autoware::common::types::PointXYZIF> & output);
  
	void unpack_dev(const std::vector<uint8_t> & buffer, std::size_t & m_cloud_size);
	
	double R1_;
	double R2_;
	float distance_unit_;
	int return_mode_;
	int degree_mode_;
	bool config_vert_;
	bool config_vert_angle;
	bool config_vert_file_;
	double vert_angle;
	double cos_scan_altitude_caliration[LSC16_SCANS_PER_FIRING];
	double sin_scan_altitude_caliration[LSC16_SCANS_PER_FIRING];
	float sin_azimuth_table[ROTATION_MAX_UNITS];
    float cos_azimuth_table[ROTATION_MAX_UNITS];
	
private:
	LSLIDAR_DRIVER_LOCAL void init_azimuth_table();
    float32_t m_rpm;
};
}  // namespace lslidar_driver
}  // namespace drivers
}  // namespace autoware

#endif  // LSLIDAR_DRIVER__LS16_DATA_HPP_

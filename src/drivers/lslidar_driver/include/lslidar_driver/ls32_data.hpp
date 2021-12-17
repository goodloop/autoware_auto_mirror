// Copyright 2020 the Autoware Foundation
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

/// \copyright Copyright 2020 the Autoware Foundation
/// \file
/// \brief This file defines a driver for Lslidar LiDARs

#ifndef LSLIDAR_DRIVER__LS32_DATA_HPP_
#define LSLIDAR_DRIVER__LS32_DATA_HPP_

#include <lslidar_driver/visibility_control.hpp>
#include <lslidar_driver/common.hpp>
#include <cmath>
#include <cstring>
#include <utility>

namespace autoware
{
namespace drivers
{
namespace lslidar_driver
{
	
class LSLIDAR_DRIVER_PUBLIC LS32Data
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

/** Special Defines for LSC32 support **/
	static constexpr int LSC32_FIRINGS_PER_BLOCK = 1;
	static constexpr int LSC32_SCANS_PER_FIRING = 32;
	static constexpr float LSC32_BLOCK_TDURATION = 100.0f;  // [µs]
	static constexpr float LSC32_DSR_TOFFSET = 3.125f;        // [µs]
	static constexpr float LSC32_FIRING_TOFFSET = 100.0f;    // [µs]
	
	static constexpr int TEMPERATURE_MIN = 31;
	static constexpr int PACKET_SIZE = 1206;
    static constexpr int BLOCKS_PER_PACKET = 12;
    static constexpr int PACKET_STATUS_SIZE = 4;
    static constexpr int SCANS_PER_PACKET = (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);
	
	static constexpr float LSC32_AZIMUTH_TOFFSET = 12.98f;  /**< meters */
	static constexpr float LSC32_DISTANCE_TOFFSET = 4.94f;  /**< meters */

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
	double scan_altitude_original_A[32] = {  //1
	  -16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1,
	  0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15
	};

	double scan_altitude_original_A3[32] = {  //1
	  -16,0,-15,1,-14,2,-13,3,-12,4,-11,5,-10,6,-9,7,-8,8,-7,
	  9,-6,10,-5,11,-4,12,-3,13,-2,14,-1,15
	};

	const double scan_altitude_original_C[32] = {  //0.33
	  -18,-15,-12,-10,-8,-7,-6,-5,-4,-3.33,-2.66,-3,-2.33,-2,-1.33,-1.66,
	  -1,-0.66,0,-0.33,0.33,0.66,1.33,1,1.66,2,3,4,6,8,11,14
	};

	double scan_altitude_original_C3[32] = {  //0.33
	  -18,-1,-15,-0.66,-12,-0.33,-10,0,-8,0.33,-7,0.66,-6,1,-5,1.33,-4,1.66,
	  -3.33,2,-3,3,-2.66,4,-2.33,6,-2,8,-1.66,11,-1.33,14
	};

	 double scan_altitude[32] = {
			-0.2792526803190927, -0.2617993877991494,
			-0.24434609527920614, -0.22689280275926285,
			-0.20943951023931953, -0.19198621771937624,
			-0.17453292519943295, -0.15707963267948966,
			-0.13962634015954636, -0.12217304763960307,
			-0.10471975511965977, -0.08726646259971647,
			-0.06981317007977318, -0.05235987755982988,
			-0.03490658503988659, -0.017453292519943295,
			0                   			 , 0.017453292519943295,
			0.03490658503988659, 0.05235987755982988,
			0.06981317007977318, 0.08726646259971647,
			0.10471975511965977, 0.12217304763960307,
			0.13962634015954636, 0.15707963267948966,
			0.17453292519943295, 0.19198621771937624,
			0.20943951023931953, 0.22689280275926285,
			0.24434609527920614, 0.2617993877991494
	};

	 double scan_altitude2[32] = {
			-0.3141592653589793,-0.20943951023931956,
			-0.13962634015954636,-0.10471975511965978,
			-0.06981317007977318,-0.05235987755982989,
			-0.04066617157146788,-0.023212879051524585,
			-0.017453292519943295,-0.005759586531581287,
			0.005759586531581287,0.017453292519943295,
			0.02897246558310587,0.05235987755982989,
			0.10471975511965978,0.19198621771937624,
			-0.2617993877991494,-0.17453292519943295,
			-0.12217304763960307,-0.08726646259971647,
			-0.05811946409141117,-0.04642575810304917,
			-0.03490658503988659,-0.02897246558310587,
			-0.011519173063162575,0.0,
			0.011519173063162575,0.023212879051524585,
			0.03490658503988659,0.06981317007977318,
			0.13962634015954636,0.24434609527920614
	};

	typedef struct raw_packet {
        raw_block_t blocks[BLOCKS_PER_PACKET];
        uint16_t revolution;
        uint8_t status[PACKET_STATUS_SIZE];
    } raw_packet_t;
	
	explicit LS32Data(const float32_t rpm);

	void unpack(const std::vector<uint8_t> & buffer, std::vector<autoware::common::types::PointXYZIF> & output);
  
	void unpack_dev(const std::vector<uint8_t> & buffer, std::size_t & m_cloud_size);
	
	int adjust_angle;
	int adjust_angle_two;
	int adjust_angle_three;
	int adjust_angle_four;

	double scan_altitude_A[32];
	double scan_altitude_C[32];
  
	double R1_;
	double R2_;
	float distance_unit_;
	int return_mode_;
	int degree_mode_;
	bool config_vert_;
	bool config_vert_angle;
	double cos_scan_altitude_caliration[LSC32_SCANS_PER_FIRING];
	double sin_scan_altitude_caliration[LSC32_SCANS_PER_FIRING];
	float sin_azimuth_table[ROTATION_MAX_UNITS];
    float cos_azimuth_table[ROTATION_MAX_UNITS];
	
private:
    float32_t m_rpm;
	
};

}  // namespace lslidar_driver
}  // namespace drivers
}  // namespace autoware

#endif  // LSLIDAR_DRIVER__LS32_DATA_HPP_

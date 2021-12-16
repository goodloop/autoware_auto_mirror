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

/// \copyright Copyright 2018-2020 the Autoware Foundation
/// \file
/// \brief This file defines a driver for Lslidar LiDARs

#ifndef LSLIDAR_DRIVER__CH128_DATA_HPP_
#define LSLIDAR_DRIVER__CH128_DATA_HPP_

#include <lslidar_driver/visibility_control.hpp>
#include <lslidar_driver/common.hpp>
#include <cmath>
#include <utility>
#include "rclcpp/rclcpp.hpp"

namespace autoware
{
namespace drivers
{
namespace lslidar_driver
{
class LSLIDAR_DRIVER_PUBLIC CH128Data
{
public:
	static constexpr double DEG_TO_RAD = 0.017453292;
	static constexpr double RAD_TO_DEG = 57.29577951;
	static constexpr int SIZE_BLOCK = 100;
    static constexpr int RAW_SCAN_SIZE = 3;
    static constexpr int SCANS_PER_BLOCK = 32;
    static constexpr int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);  // 96

    static constexpr double DISTANCE_MAX = 200.0000390625;            /**< meters */
    static constexpr double DISTANCE_RESOLUTION = 0.0000390625;      /**< meters */
    static constexpr double DISTANCE_MAX_UNITS = DISTANCE_MAX / DISTANCE_RESOLUTION;

	static constexpr int PACKET_SIZE = 1206;
	static constexpr int POINTS_PER_PACKET  = 171;

	static constexpr double scan_laser_altitude[32] = {
        -0.29670597283903605,-0.2792526803190927,
        -0.2617993877991494,-0.24434609527920614,
        -0.22689280275926285,-0.20943951023931956,
        -0.19198621771937624,-0.17453292519943295,
        -0.15707963267948966,-0.13962634015954636,
        -0.12217304763960307,-0.10471975511965978,
        -0.08726646259971647,-0.06981317007977318,
        -0.05235987755982989,-0.03490658503988659,
        -0.017453292519943295,0.0,
         0.017453292519943295,0.03490658503988659,
         0.05235987755982989,0.06981317007977318,
         0.08726646259971647,0.10471975511965978,
         0.12217304763960307,0.13962634015954636,
         0.15707963267948966,0.17453292519943295,
         0.19198621771937624,0.20943951023931956,
         0.22689280275926285,0.24434609527920614,
	};
	
	static constexpr double sin_scan_laser_altitude[32] = {
		std::sin(scan_laser_altitude[0]), std::sin(scan_laser_altitude[1]),
		std::sin(scan_laser_altitude[2]), std::sin(scan_laser_altitude[3]),
		std::sin(scan_laser_altitude[4]), std::sin(scan_laser_altitude[5]),
		std::sin(scan_laser_altitude[6]), std::sin(scan_laser_altitude[7]),
		std::sin(scan_laser_altitude[8]), std::sin(scan_laser_altitude[9]),
		std::sin(scan_laser_altitude[10]), std::sin(scan_laser_altitude[11]),
		std::sin(scan_laser_altitude[12]), std::sin(scan_laser_altitude[13]),
		std::sin(scan_laser_altitude[14]), std::sin(scan_laser_altitude[15]),
		std::sin(scan_laser_altitude[16]), std::sin(scan_laser_altitude[17]),
		std::sin(scan_laser_altitude[18]), std::sin(scan_laser_altitude[19]),
		std::sin(scan_laser_altitude[20]), std::sin(scan_laser_altitude[21]),
		std::sin(scan_laser_altitude[22]), std::sin(scan_laser_altitude[23]),
		std::sin(scan_laser_altitude[24]), std::sin(scan_laser_altitude[25]),
		std::sin(scan_laser_altitude[26]), std::sin(scan_laser_altitude[27]),
		std::sin(scan_laser_altitude[28]), std::sin(scan_laser_altitude[29]),
		std::sin(scan_laser_altitude[30]), std::sin(scan_laser_altitude[31]),
	};
	
	double big_angle[32]={-17,-16,-15,-14,-13,-12,-11,-10, -9,-8,-7,-6,-5,-4.125,-4,-3.125,
					  -3,-2.125,-2,-1.125,-1,-0.125,0,0.875,1,1.875,2,3,4,5,6,7};
	
	
	static constexpr double scan_mirror_altitude[4] = {
			0.0,0.004363323129985824,
			0.008726646259971648,0.013089969389957472,
	};
	
	static constexpr double sin_scan_mirror_altitude[4] = {
		std::sin(scan_mirror_altitude[0]), std::sin(scan_mirror_altitude[1]),
		std::sin(scan_mirror_altitude[2]), std::sin(scan_mirror_altitude[3]),
	};

	typedef struct{
		double distance;
		double intensity;
	}point_struct;
	
	union TwoBytes {
        int16_t value;
        uint8_t  bytes[2];
    };
	
	union ThreeBytes {
        uint32_t value;
        uint8_t  bytes[3];
    };

    struct Point {
        uint8_t vertical_line;        //0-127 
        uint8_t azimuth_1;
        uint8_t azimuth_2;      ///< 1500-16500, divide by 100 to get degrees
        uint8_t distance_1;
        uint8_t distance_2;
        uint8_t distance_3;
        uint8_t intensity;
    };
    
    struct RawPacket {
        Point points[POINTS_PER_PACKET];
        uint32_t time_stamp;
        uint8_t factory[2];
    };

	struct Firing {
        int vertical_line;
        double azimuth;
        double distance;
        int intensity;
    };
	
	explicit CH128Data(const float32_t rpm);

	void unpack(const std::vector<uint8_t> & buffer, std::vector<autoware::common::types::PointXYZIF> & output);
  
	void unpack_dev(const std::vector<uint8_t> & buffer, std::size_t & m_cloud_size);
	
	bool publish_point_cloud;
    bool publish_channel;
	bool is_first_sweep;
    double last_azimuth;
	double prism_angle[4];
	uint64_t time_last;
    int channel_num;
    int layer_num;
    Firing firings[POINTS_PER_PACKET];
	
private:
    float32_t m_rpm;

};

}  // namespace lslidar_driver
}  // namespace drivers
}  // namespace autoware

#endif  // LSLIDAR_DRIVER__CH128_DATA_HPP_

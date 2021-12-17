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

#ifndef LSLIDAR_DRIVER__CH64W_DATA_HPP_
#define LSLIDAR_DRIVER__CH64W_DATA_HPP_

#include <lslidar_driver/visibility_control.hpp>
#include <lslidar_driver/common.hpp>
#include <cmath>
#include <utility>

namespace autoware
{
namespace drivers
{
namespace lslidar_driver
{
class LSLIDAR_DRIVER_PUBLIC CH64wData
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

	typedef struct{
		double distance;
		double intensity;
	}point_struct;
	
	union TwoBytes {
        uint16_t value;
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
	
	explicit CH64wData(const float32_t rpm);

	void unpack(const std::vector<uint8_t> & buffer, std::vector<autoware::common::types::PointXYZIF> & output);
  
	void unpack_dev(const std::vector<uint8_t> & buffer, std::size_t & m_cloud_size);
	
	bool publish_point_cloud;
    bool publish_channel;
	bool is_first_sweep;
    double last_azimuth;
	uint64_t time_last;
    int channel_num;
    int layer_num;
    Firing firings[POINTS_PER_PACKET];
	double prismAngle[4];
	double Theat1_s[128];
	double Theat2_s[128];
	double Theat1_c[128];
	double Theat2_c[128];
	
private:
    float32_t m_rpm;

};

}  // namespace lslidar_driver
}  // namespace drivers
}  // namespace autoware

#endif  // LSLIDAR_DRIVER__CH64W_DATA_HPP_

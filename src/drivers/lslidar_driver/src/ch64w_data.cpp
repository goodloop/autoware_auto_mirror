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

#include <lslidar_driver/ch64w_data.hpp>
#include <utility>
#include <angles/angles.h>

namespace autoware
{
namespace drivers
{
namespace lslidar_driver
{

CH64wData::CH64wData(const float32_t rpm)
{
	m_rpm = rpm;
}

void CH64wData::unpack_dev(const std::vector<uint8_t> & buffer, std::size_t & m_cloud_size) 
{
	const uint8_t *data = &buffer[0];
	
	if (data[0] != 0xa5 || data[1] != 0xff || data[2] != 0x00 || data[3] != 0x5a) {
            return;
    }
	
	int rpm = data[8]*256 + data[9];
	
	if(rpm < 450)
		rpm = 300;
	else if(rpm < 750)
		rpm = 600;
	else
		rpm = 1200;
	
	m_cloud_size = static_cast<std::size_t >(11450*171*60/rpm);
}

void CH64wData::unpack(const std::vector<uint8_t> & buffer, std::vector<autoware::common::types::PointXYZIF> & output)
{
	const RawPacket *packet = reinterpret_cast<const RawPacket *>(&buffer[0]);

	for (size_t point_idx = 0; point_idx < POINTS_PER_PACKET; point_idx++) 
	{
		firings[point_idx].vertical_line = packet->points[point_idx].vertical_line;
		TwoBytes point_amuzith;
		point_amuzith.bytes[0] = packet->points[point_idx].azimuth_2;
		point_amuzith.bytes[1] = packet->points[point_idx].azimuth_1;
		firings[point_idx].azimuth = static_cast<double>(point_amuzith.value) * 0.01;
		ThreeBytes point_distance;
		point_distance.bytes[0] = packet->points[point_idx].distance_3;
		point_distance.bytes[1] = packet->points[point_idx].distance_2;
		point_distance.bytes[2] = packet->points[point_idx].distance_1;
		firings[point_idx].distance = static_cast<double>(point_distance.value) * DISTANCE_RESOLUTION;
		firings[point_idx].intensity = packet->points[point_idx].intensity;
	}
	
	for (int i = 0; i < 4; i++)
	{
		prismAngle[i] = i * 0.35;
	}
	for (int i = 0; i < 128; i++)
	{
		if (i / 4 % 2 == 0)
		{
			Theat1_s[i] = sin((-25 + floor(i / 8) * 2.5) * DEG_TO_RAD);
			Theat2_s[i] = sin((prismAngle[i % 4]) * DEG_TO_RAD);
			Theat1_c[i] = cos((-25 + floor(i /8) * 2.5) * DEG_TO_RAD);
			Theat2_c[i] = cos((prismAngle[i % 4]) * DEG_TO_RAD);
		}
		else
		{
			Theat1_s[i] = sin((-24 + floor(i / 8) * 2.5) * DEG_TO_RAD);
			Theat2_s[i] = sin((prismAngle[i % 4]) * DEG_TO_RAD);
			Theat1_c[i] = cos((-24 + floor(i /8) * 2.5) * DEG_TO_RAD);
			Theat2_c[i] = cos((prismAngle[i % 4]) * DEG_TO_RAD);
		}
	}
		
	double cos_xita;
	
	for (size_t point_idx = 0; point_idx < POINTS_PER_PACKET; point_idx++) {
		if (firings[point_idx].vertical_line >128) continue;

		int ID = firings[point_idx].vertical_line;
		PointXYZIF pt;
		
		if (ID / 4 % 2 == 0)
			cos_xita = cos((firings[point_idx].azimuth / 2.0 + 22.5) * DEG_TO_RAD);
		else
			cos_xita = cos((-firings[point_idx].azimuth / 2.0 +112.5) * DEG_TO_RAD);
			
		double _R_ = Theat2_c[ID] * Theat1_c[ID] * cos_xita - Theat2_s[ID] * Theat1_s[ID];
		
		double sin_theat = Theat1_s[ID] + 2*_R_ * Theat2_s[ID]; 
		double cos_theat = sqrt(1 - pow(sin_theat, 2)); 
		double cos_H_xita = (2 * _R_ * Theat2_c[ID] * cos_xita - Theat1_c[ID] / cos_theat);
		double sin_H_xita = sqrt(1 - pow(cos_H_xita, 2)); 
		
		double cos_xita_F;
		double sin_xita_F;
		if (ID / 4 % 2 == 0)
		{
			cos_xita_F = (cos_H_xita + sin_H_xita) * sqrt(0.5);
			sin_xita_F = sqrt(1 - pow(cos_xita_F, 2)); 
		}
		else
		{
			cos_xita_F = (cos_H_xita + sin_H_xita) * (-sqrt(0.5));
			sin_xita_F = sqrt(1 - pow(cos_xita_F, 2)); 
		}	

		
		/*if (ID / 4 % 2 == 0)
		{
			sin_theat_left = Theat1_s[ID] - 2 * pow(Theat2_s[ID], 2) * Theat1_s[ID]
							+ 2 * cos((firings[point_idx].azimuth / 2.0 + 22.5) * DEG_TO_RAD)* Theat2_s[ID] * Theat2_c[ID] * Theat1_c[ID];
			cos_theat_left = sqrt(1 - pow(sin_theat_left, 2));
			
			pt.x = static_cast<float>(firings[point_idx].distance * cos_theat_left * cos(firings[point_idx].azimuth * DEG_TO_RAD));
			pt.y = static_cast<float>(firings[point_idx].distance * cos_theat_left * sin(firings[point_idx].azimuth * DEG_TO_RAD));
			pt.z = static_cast<float>(firings[point_idx].distance * sin_theat_left);
		}else {*/
		//	sin_theat_right = Theat1_s[ID] - 2 * pow(Theat2_s[ID], 2) * Theat1_s[ID]
		//					+ 2 * cos((-firings[point_idx].azimuth / 2.0 + 112.5) * DEG_TO_RAD)* Theat2_s[ID] * Theat2_c[ID] * Theat1_c[ID];
		//	cos_theat_right = sqrt(1 - pow(sin_theat_right, 2));
			
			pt.x = static_cast<float>(firings[point_idx].distance * cos_theat * cos_xita_F);
			pt.y = static_cast<float>(firings[point_idx].distance * cos_theat * sin_xita_F);
			pt.z = static_cast<float>(firings[point_idx].distance * sin_theat);
		//}

		pt.intensity = static_cast<float>(firings[point_idx].intensity);
		pt.id = static_cast<uint16_t>(firings[point_idx].vertical_line);
		output.push_back(pt);
	}
}


}  // namespace lslidar_driver
}  // namespace drivers
}  // namespace autoware

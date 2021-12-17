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

#include <lslidar_driver/ch128_data.hpp>
#include <utility>
#include <angles/angles.h>

namespace autoware
{
namespace drivers
{
namespace lslidar_driver
{

CH128Data::CH128Data(const float32_t rpm)
{
	m_rpm = rpm;
}

void CH128Data::unpack_dev(const std::vector<uint8_t> & buffer, std::size_t & m_cloud_size) 
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

	m_cloud_size = static_cast<std::size_t >(6791*171*60/rpm);

	TwoBytes angle_1, angle_2, angle_3, angle_4;
	angle_1.bytes[0] = data[243];
	angle_1.bytes[1] = data[242];
	prism_angle[0] = angle_1.value * 0.01;

	angle_2.bytes[0] = data[245];
	angle_2.bytes[1] = data[244];
	prism_angle[1] = angle_2.value * 0.01;

	angle_3.bytes[0] = data[247];
	angle_3.bytes[1] = data[246];
	prism_angle[2] = angle_3.value * 0.01;

	angle_4.bytes[0] = data[249];
	angle_4.bytes[1] = data[248];
	prism_angle[3] = angle_4.value * 0.01;
}

void CH128Data::unpack(const std::vector<uint8_t> & buffer, std::vector<autoware::common::types::PointXYZIF> & output)
{
	const RawPacket *packet = reinterpret_cast<const RawPacket *>(&buffer[0]);

	for (size_t point_idx = 0; point_idx < POINTS_PER_PACKET; point_idx++) 
	{
		firings[point_idx].vertical_line = packet->points[point_idx].vertical_line;
		TwoBytes point_amuzith;
		point_amuzith.bytes[0] = packet->points[point_idx].azimuth_2;
		point_amuzith.bytes[1] = packet->points[point_idx].azimuth_1;
		firings[point_idx].azimuth = static_cast<double>(point_amuzith.value) * 0.01 * DEG_TO_RAD;
		ThreeBytes point_distance;
		point_distance.bytes[0] = packet->points[point_idx].distance_3;
		point_distance.bytes[1] = packet->points[point_idx].distance_2;
		point_distance.bytes[2] = packet->points[point_idx].distance_1;
		firings[point_idx].distance = static_cast<double>(point_distance.value) * DISTANCE_RESOLUTION;
		firings[point_idx].intensity = packet->points[point_idx].intensity;
	}
	
	double z_sin_altitude = 0.0;
	double z_cos_altitude = 0.0;
	double sinTheta_1[128] = {0};
	double sinTheta_2[128] = {0};
	double cosTheta_1[128] = {0};
	double cosTheta_2[128] = {0};
	
	for (int i = 0; i < 128; i++) 
	{
		sinTheta_1[i] = sin(big_angle[i / 4] * M_PI / 180);
		cosTheta_1[i] = cos(big_angle[i / 4] * M_PI / 180);
	   if(fabs(prism_angle[0]) < 1e-6 && fabs(prism_angle[1]) < 1e-6 && fabs(prism_angle[2]) < 1e-6 && fabs(prism_angle[3]) < 1e-6){
		   sinTheta_2[i] = sin((i % 4) * (-0.17) * M_PI / 180);
		   cosTheta_2[i] = cos((i % 4) * (-0.17) * M_PI / 180);
	   }else{
		   sinTheta_2[i] = sin(prism_angle[i % 4]  * M_PI / 180);
		   cosTheta_2[i] = cos(prism_angle[i % 4]  * M_PI / 180);
	   }
	}
	
	for (size_t point_idx = 0; point_idx < POINTS_PER_PACKET; point_idx++) {

		double _R_ = cosTheta_2[firings[point_idx].vertical_line] * cosTheta_1[firings[point_idx].vertical_line] * cos(firings[point_idx].azimuth/2.0) 
			- sinTheta_2[firings[point_idx].vertical_line] * sinTheta_1[firings[point_idx].vertical_line];

		z_sin_altitude = sinTheta_1[firings[point_idx].vertical_line] + 2 * _R_* sinTheta_2[firings[point_idx].vertical_line];
		z_cos_altitude = sqrt(1 - pow(z_sin_altitude, 2));

		PointXYZIF pt;
		pt.x = static_cast<float>(firings[point_idx].distance * z_cos_altitude * cos(firings[point_idx].azimuth));
        pt.y = static_cast<float>(firings[point_idx].distance * z_cos_altitude * sin(firings[point_idx].azimuth));
        pt.z = static_cast<float>(firings[point_idx].distance * z_sin_altitude);
		pt.intensity = static_cast<float>(firings[point_idx].intensity);
		pt.id = static_cast<uint16_t>(firings[point_idx].vertical_line);
		output.push_back(pt);
	}
}


}  // namespace lslidar_driver
}  // namespace drivers
}  // namespace autoware

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

#include <lslidar_driver/ls16_data.hpp>
#include <utility>
#include <angles/angles.h>

namespace autoware
{
namespace drivers
{
namespace lslidar_driver
{
LS16Data::LS16Data(const float32_t rpm)
{
	m_rpm = rpm;
    config_vert_angle = false;
	R1_ = 0.04376;  
	R2_ = 0.010875;
	degree_mode_ =  2;
	distance_unit_ = 0.25;
	return_mode_ = 1;
}

void LS16Data::unpack_dev(const std::vector<uint8_t> & buffer, std::size_t & m_cloud_size) 
{
	const uint8_t *data = &buffer[0];

	if (data[0] != 0xa5 || data[1] != 0xff || data[2] != 0x00 || data[3] != 0x5a) {
            return;
    }
	int rpm = data[8]*256 + data[9];
	
	if(data[185] == 0 || data[185] == 1)
		return_mode_ = data[185] + 1;
	
	if(rpm < 450)
		rpm = 300;
	else if(rpm < 750)
		rpm = 600;
	else
		rpm = 1200;
	
	m_cloud_size = static_cast<std::size_t >(31990*600*return_mode_/rpm);
	
	for (int i = 0; i < 16; i++) {
		uint8_t data1 = data[245 + 2 * i];
		uint8_t data2 = data[245 + 2 * i + 1];
		int vert_angle = data1 * 256 + data2;
		if (vert_angle > 32767) {
			vert_angle = vert_angle - 65535;
		}
		double accuracy = 100;
		scan_altitude[i] = static_cast<double>(vert_angle) / accuracy * DEG_TO_RAD;

		if (2 == degree_mode_) {
			if (scan_altitude[i] != 0) {
				if (fabs(scan_altitude_original_2[i] - scan_altitude[i]) * RAD_TO_DEG > 1.5) {
					scan_altitude[i] = scan_altitude_original_2[i];
				}
			} else {
				scan_altitude[i] = scan_altitude_original_2[i];
			}
		} else if (1 == degree_mode_) {
			if (scan_altitude[i] != 0) {
				if (fabs(scan_altitude_original_1[i] - scan_altitude[i]) * RAD_TO_DEG > 1.5) {
					scan_altitude[i] = scan_altitude_original_1[i];
				}
			} else {
				scan_altitude[i] = scan_altitude_original_1[i];
			}
		}
		config_vert_angle = true;
	}
	
	init_azimuth_table();
}

void LS16Data::unpack(const std::vector<uint8_t> & buffer, std::vector<autoware::common::types::PointXYZIF> & output)
{
	float azimuth;  
	float intensity;
	float azimuth_diff;
	float azimuth_corrected_f;
	int azimuth_corrected;
	
	if (config_vert_angle) {
		for (int i = 0; i < LSC16_SCANS_PER_FIRING; i++) {
			cos_scan_altitude_caliration[i] = std::cos(scan_altitude[i]);
			sin_scan_altitude_caliration[i] = std::sin(scan_altitude[i]);
		}
		config_vert_angle = false;
	}

	const raw_packet_t *raw = reinterpret_cast<const raw_packet_t *>(&buffer[0]);
	
	for (int block = 0; block < BLOCKS_PER_PACKET; block++)  
	{
		if (UPPER_BANK != raw->blocks[block].header) {
			break;
		}
		azimuth = static_cast<float>(256 * raw->blocks[block].rotation_2 + raw->blocks[block].rotation_1);

		if (2 == return_mode_) {

			if (block < (BLOCKS_PER_PACKET - 2))  
			{
				int azi1, azi2;
				azi1 = 256 * raw->blocks[block + 2].rotation_2 + raw->blocks[block + 2].rotation_1;
				azi2 = 256 * raw->blocks[block].rotation_2 + raw->blocks[block].rotation_1;
				azimuth_diff = static_cast<float>((36000 + azi1 - azi2) % 36000);
			} else {
				int azi1, azi2;
				azi1 = 256 * raw->blocks[block].rotation_2 + raw->blocks[block].rotation_1;
				azi2 = 256 * raw->blocks[block - 2].rotation_2 + raw->blocks[block - 2].rotation_1;
				azimuth_diff = static_cast<float>((36000 + azi1 - azi2) % 36000);
			}
		} 
		else 
		{
			if (block < (BLOCKS_PER_PACKET - 1))  
			{
				int azi1, azi2;
				azi1 = 256 * raw->blocks[block + 1].rotation_2 + raw->blocks[block + 1].rotation_1;
				azi2 = 256 * raw->blocks[block].rotation_2 + raw->blocks[block].rotation_1;
				azimuth_diff = static_cast<float>((36000 + azi1 - azi2) % 36000);
			} else {
				int azi1, azi2;
				azi1 = 256 * raw->blocks[block].rotation_2 + raw->blocks[block].rotation_1;
				azi2 = 256 * raw->blocks[block - 1].rotation_2 + raw->blocks[block - 1].rotation_1;
				azimuth_diff = static_cast<float>((36000 + azi1 - azi2) % 36000);
			}
		}

		float cos_azimuth;
		float sin_azimuth;
		for (int firing = 0, k = 0; firing < LSC16_FIRINGS_PER_BLOCK; firing++)  // 2
		{
			for (uint16_t dsr = 0; dsr < LSC16_SCANS_PER_FIRING; dsr++, k += RAW_SCAN_SIZE)  // 16   3
			{
				azimuth_corrected_f = azimuth + azimuth_diff / static_cast<float>((LSC16_SCANS_PER_FIRING * 2) *
												(LSC16_SCANS_PER_FIRING * firing + dsr));

				azimuth_corrected = static_cast<int>(round(azimuth_corrected_f)) % 36000;  // convert to integral value...

				cos_azimuth = cos_azimuth_table[azimuth_corrected];
				sin_azimuth = sin_azimuth_table[azimuth_corrected];

				//distance
				union two_bytes tmp;
				tmp.bytes[0] = raw->blocks[block].data[k];
				tmp.bytes[1] = raw->blocks[block].data[k + 1];
				float distance = static_cast<float>(tmp.uint);

				// read intensity
				intensity = raw->blocks[block].data[k + 2];
				float distance2 = distance * DISTANCE_RESOLUTION * distance_unit_;

				//The offset calibration
				float arg_horiz = azimuth_corrected_f * ROTATION_RESOLUTION;
				arg_horiz = arg_horiz > 360 ? (arg_horiz - 360) : arg_horiz;
				float arg_horiz_orginal = (static_cast<float>(14.67) - arg_horiz) * static_cast<float>(M_PI) / 180.f;
				float distance_offset = static_cast<float>(0.00426);
				
				PointXYZIF pt;
				pt.x = distance2 * static_cast<float>(cos_scan_altitude_caliration[dsr]) * cos_azimuth + static_cast<float>(R1_ * cos(arg_horiz_orginal));
				pt.y = -distance2 * static_cast<float>(cos_scan_altitude_caliration[dsr]) * sin_azimuth -  static_cast<float>(R1_ * sin(arg_horiz_orginal));
				pt.z = distance2 * static_cast<float>(sin_scan_altitude_caliration[dsr]) + distance_offset;
				pt.intensity = intensity;
				pt.id = static_cast<uint16_t>((dsr % 2) * 8 + dsr / 2);
				output.push_back(pt);
			}
		}
	}
}

void LS16Data::init_azimuth_table()
{
		if (2 == degree_mode_) {
            //Vertical Angle Calibration for device package
            for (int i = 0; i < LSC16_SCANS_PER_FIRING; i++) {
                cos_scan_altitude_caliration[i] = std::cos(scan_altitude_original_2[i]);
                sin_scan_altitude_caliration[i] = std::sin(scan_altitude_original_2[i]);
                scan_altitude[i] = scan_altitude_original_2[i];
            }
        } else if (1 == degree_mode_) {
            for (int i = 0; i < LSC16_SCANS_PER_FIRING; i++) {
                cos_scan_altitude_caliration[i] = std::cos(scan_altitude_original_1[i]);
                sin_scan_altitude_caliration[i] = std::sin(scan_altitude_original_1[i]);
                scan_altitude[i] = scan_altitude_original_1[i];
            }
        }
		
		for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
            float rotation = static_cast<float>(angles::from_degrees(ROTATION_RESOLUTION * rot_index));
            cos_azimuth_table[rot_index] = cosf(rotation);
            sin_azimuth_table[rot_index] = sinf(rotation);
        }
}

}  // namespace lslidar_driver
}  // namespace drivers
}  // namespace autoware

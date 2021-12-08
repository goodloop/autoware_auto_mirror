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

#include <lslidar_driver/ls32_data.hpp>
#include <utility>
#include <angles/angles.h>
#include <iostream>

namespace autoware
{
namespace drivers
{
namespace lslidar_driver
{

LS32Data::LS32Data(const float32_t rpm)
{
	m_rpm = rpm;
    config_vert_angle = false;
	R1_ = 0.04638;  
	R2_ = 0.010875;
	distance_unit_ = 0.25f;
	return_mode_ = 1;
	degree_mode_ = 2;
}

void LS32Data::unpack_dev(const std::vector<uint8_t> & buffer, std::size_t & m_cloud_size) 
{
	const uint8_t *data = &buffer[0];
	int startpos = 0;
	double scan_altitude_original_degree33[32];
	double scan_altitude_original_degree1[32];
  
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
	
	  startpos = 245;
	   //Horizontal correction Angle
	  adjust_angle = (data[186]*256 + data[187]);         //Angle correction A1
	  if(adjust_angle > 32767){
		adjust_angle = adjust_angle - 65535;
	  }

	  adjust_angle_two = (data[190]*256 + data[191]);     //Angle correction A2
	  if(adjust_angle_two > 32767){
		adjust_angle_two = adjust_angle_two - 65535;
	  }

	  adjust_angle_three = (data[188]*256 + data[189]);   //Angle correction A3
	  if(adjust_angle_three > 32767){
		adjust_angle_three = adjust_angle_three - 65535;
	  }

	  adjust_angle_four = (data[192]*256 + data[193]);    //Angle correction A4
	  if(adjust_angle_four > 32767){
		adjust_angle_four = adjust_angle_four - 65535;
	  }
	  memcpy(scan_altitude_original_degree1,scan_altitude_original_A3,32*8);
	  memcpy(scan_altitude_original_degree33,scan_altitude_original_C3,32*8);
	  
	if(data[185] == 0 || data[185] == 1)
	  {
		  return_mode_ = data[185]+1;
		  if(data[1195] == 0x21 ) 
			 degree_mode_ = 2;
		  else
			 degree_mode_ = 1;
		  
		  distance_unit_ = 0.4f;
		
		  for(int i = 0; i < LSC32_SCANS_PER_FIRING; i++)
		  {
			//均匀1度校准两列
			if(1 == degree_mode_){
			  cos_scan_altitude_caliration[i] = std::cos(angles::from_degrees(scan_altitude_original_A3[i]));
			  sin_scan_altitude_caliration[i] = std::sin(angles::from_degrees(scan_altitude_original_A3[i]));
			  scan_altitude_A[i] = scan_altitude_original_A3[i];
			}

			//0.33度校准四列
			if(2 == degree_mode_){
			  cos_scan_altitude_caliration[i] = std::cos(angles::from_degrees(scan_altitude_original_C3[i]));
			  sin_scan_altitude_caliration[i] = std::sin(angles::from_degrees(scan_altitude_original_C3[i]));
			  scan_altitude_C[i] = scan_altitude_original_C3[i];
			}
		  }
	  }
	  
	  if(1 == degree_mode_)
		m_cloud_size = static_cast<std::size_t >(1693*354*60*return_mode_*return_mode_/rpm);
	  else
		m_cloud_size = static_cast<std::size_t >(1700*354*60*return_mode_*return_mode_/rpm);
						
	 if(config_vert_)
	 {
		for(int i = 0; i < LSC32_SCANS_PER_FIRING; i++){
		  uint8_t data1 = data[startpos + 2*i];
		  uint8_t data2 = data[startpos + 2*i+1];
		  int vert_angle = data1*256 + data2;
		  if(vert_angle > 32767){
			vert_angle = vert_angle - 65535;
		  }

	  
		  //均匀1度校准两列
		  if(1 == degree_mode_){
			scan_altitude_A[i] = static_cast<double>(vert_angle) *  static_cast<double>(ROTATION_RESOLUTION);
			if(fabs(scan_altitude_original_degree1[i] - scan_altitude_A[i]) > 1.5){
			  scan_altitude_A[i] = scan_altitude_original_degree1[i];
			}
			config_vert_angle = true;
		  }

		  //0.33度校准四列
		  if(2 == degree_mode_){
			scan_altitude_C[i] = static_cast<double>(vert_angle) * static_cast<double>(ROTATION_RESOLUTION);
			if(fabs(scan_altitude_original_degree33[i] - scan_altitude_C[i]) > 1.5){
			  scan_altitude_C[i] = scan_altitude_original_degree33[i];
			}
			config_vert_angle = true;
		  }
    }
  }
}

void LS32Data::unpack(const std::vector<uint8_t> & buffer, std::vector<autoware::common::types::PointXYZIF> & output)
{
	float azimuth;  // 0.01 dgree
	float intensity;
	float azimuth_diff;
	float azimuth_corrected_f;
	float azimuth_corrected_offset_f;
	if(config_vert_angle)
	{
		for(int i = 0; i < LSC32_SCANS_PER_FIRING; i++)
		{
		  //均匀1度校准两列
		  if(1 == degree_mode_){
			cos_scan_altitude_caliration[i] = std::cos(angles::from_degrees(scan_altitude_A[i]));
			sin_scan_altitude_caliration[i] = std::sin(angles::from_degrees(scan_altitude_A[i]));
		  }

		  //0.33度校准四列
		  if(2 == degree_mode_){
			cos_scan_altitude_caliration[i] = std::cos(angles::from_degrees(scan_altitude_C[i]));
			sin_scan_altitude_caliration[i] = std::sin(angles::from_degrees(scan_altitude_C[i]));
		  }
		  config_vert_angle = false;
		}
    }
	
	int ring;
	double vert1[32][2] = { 0.0 };
	double vert2[32][2] = { 0.0 };
	int vert_num[32] = { 0 };
	for(int k=0; k<LSC32_SCANS_PER_FIRING; k++)
	{
		vert1[k][0] = sin_scan_altitude_caliration[k];
		vert1[k][1] = static_cast<double>(k);
	}		
	
	for (int i=0; i<LSC32_SCANS_PER_FIRING-1; i++) 
	for (int j=0; j<LSC32_SCANS_PER_FIRING-1-i; j++) {
		if (vert1[j][0] > vert1[j+1][0]) {
			vert2[0][0] = vert1[j][0];
			vert2[0][1] = vert1[j][1];
			vert1[j][0] = vert1[j+1][0];
			vert1[j][1] = vert1[j+1][1];
			vert1[j+1][0] = vert2[0][0];
			vert1[j+1][1] = vert2[0][1];
		}
	}
	
	for(int m=0; m<LSC32_SCANS_PER_FIRING; m++)
		vert_num[m] = static_cast<int>(vert1[m][1]);

	const raw_packet_t *raw = reinterpret_cast<const raw_packet_t *>(&buffer[0]);
	
	for (int block = 0; block < BLOCKS_PER_PACKET; block++)  // 1 packet:12 data blocks
	{
		if (UPPER_BANK != raw->blocks[block].header) {
			break;
		}
		azimuth = static_cast<float>(256 * raw->blocks[block].rotation_2 + raw->blocks[block].rotation_1);

		if (2 == return_mode_) {

			if (block < (BLOCKS_PER_PACKET - 2))  // 12
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
			if (block < (BLOCKS_PER_PACKET - 1))  // 12
			{
				int azi1, azi2;
				azi1 = 256 * raw->blocks[block + 1].rotation_2 + raw->blocks[block + 1].rotation_1;
				azi2 = 256 * raw->blocks[block].rotation_2 + raw->blocks[block].rotation_1;
				azimuth_diff = static_cast<float>((36000 + azi1 - azi2) % 36000);
			} else {
				//block 12
				int azi1, azi2;
				azi1 = 256 * raw->blocks[block].rotation_2 + raw->blocks[block].rotation_1;
				azi2 = 256 * raw->blocks[block - 1].rotation_2 + raw->blocks[block - 1].rotation_1;
				azimuth_diff = static_cast<float>((36000 + azi1 - azi2) % 36000);
			}
		}

		for (int firing = 0, k = 0; firing < LSC32_FIRINGS_PER_BLOCK; firing++)  // 2
		{
			for (uint16_t dsr = 0; dsr < LSC32_SCANS_PER_FIRING; dsr++, k += RAW_SCAN_SIZE)  // 16   3
			{
				azimuth_corrected_f = azimuth + azimuth_diff / static_cast<float>(LSC32_SCANS_PER_FIRING * dsr);
				
				azimuth_corrected_f = azimuth_corrected_f < 0.0f ? azimuth_corrected_f + 36000.0f : azimuth_corrected_f;
				azimuth_corrected_f = azimuth_corrected_f >36000.0f ? azimuth_corrected_f - 36000.0f : azimuth_corrected_f;

				//distance
				union two_bytes tmp;
				tmp.bytes[0] = raw->blocks[block].data[k];
				tmp.bytes[1] = raw->blocks[block].data[k + 1];
				float distance = static_cast<float>(tmp.uint);

				// read intensity
				intensity = raw->blocks[block].data[k + 2];
				float distance2 = distance * DISTANCE_RESOLUTION * distance_unit_;
				
				for(int n=0; n<LSC32_SCANS_PER_FIRING; n++)
				{
					if(vert_num[n] == dsr)
					{
						ring = n;
						break;
					}
				}
				
				
            if(1 == degree_mode_)
				{
				  double adjust_diff = adjust_angle_two - adjust_angle;
				  if(adjust_diff > 300 && adjust_diff < 460){
						//v2.7 calibrtation
						if(1 >= dsr % 4){
						  azimuth_corrected_f += static_cast<float>(adjust_angle_two);
						}else{
						  azimuth_corrected_f += static_cast<float>(adjust_angle);
						}
				  }else{
					//v2.6 calibrtation
					if(0 == dsr % 2){
					  azimuth_corrected_f += static_cast<float>(adjust_angle);
					}else{
					  azimuth_corrected_f -= static_cast<float>(adjust_angle);
					}
				  }
				}
				
				if(2 == degree_mode_)
				{
					//v3.0 calibrtation
					if(0 == dsr || 1 == dsr || 4 == dsr || 8 == dsr || 9 == dsr || 12 == dsr || 16 == dsr || 17 == dsr || 21 == dsr  || 24 == dsr  || 25 == dsr  || 29 == dsr)
					{
						azimuth_corrected_f += static_cast<float>(adjust_angle_four);   //A4
					}

					if(2 == dsr || 3 == dsr || 6 == dsr || 10 == dsr  || 11 == dsr  || 14 == dsr  || 18 == dsr  || 19 == dsr  || 23 == dsr  || 26 == dsr  || 27 == dsr  || 31 == dsr)
					{
						azimuth_corrected_f += static_cast<float>(adjust_angle_three);  //A3
					}

					if(5== dsr || 13 == dsr || 20 == dsr || 28 == dsr)
					{
						azimuth_corrected_f += static_cast<float>(adjust_angle_two);    //A2
					}

					if(7 == dsr || 15 == dsr || 22 == dsr || 30 == dsr)
					{
						azimuth_corrected_f += static_cast<float>(adjust_angle);        //A1
					}
				}   
			
			
				float rotation_azimuth = static_cast<float>(angles::from_degrees(ROTATION_RESOLUTION * azimuth_corrected_f));
				azimuth_corrected_offset_f = azimuth_corrected_f*ROTATION_RESOLUTION - LSC32_AZIMUTH_TOFFSET;
				float rotation_azimuth_offset = static_cast<float>(angles::from_degrees(azimuth_corrected_offset_f));

				PointXYZIF pt;
				pt.x = distance2 * static_cast<float>(cos_scan_altitude_caliration[dsr]) * cosf(rotation_azimuth) + (LSC32_DISTANCE_TOFFSET * cosf(rotation_azimuth_offset)) * DISTANCE_RESOLUTION;
				pt.y = -(distance2 * static_cast<float>(cos_scan_altitude_caliration[dsr]) * sinf(rotation_azimuth) -  (LSC32_DISTANCE_TOFFSET * sinf(rotation_azimuth_offset)) * DISTANCE_RESOLUTION);
				pt.z = distance2 * static_cast<float>(sin_scan_altitude_caliration[dsr]);
				pt.intensity = intensity;
				pt.id = static_cast<uint16_t>(ring);
				output.push_back(pt);
			}
		}
	}
}

}  // namespace lslidar_driver
}  // namespace drivers
}  // namespace autoware

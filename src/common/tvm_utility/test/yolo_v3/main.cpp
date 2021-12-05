// Copyright 2021 Arm Limited and Contributors.
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

#include <autoware_auto_perception_msgs/msg/bounding_box.hpp>

#include <opencv2/opencv.hpp>
#include <algorithm>
#include <string>
#include <utility>
#include <vector>
#include <map>

#include "gtest/gtest.h"
#include "tvm_utility/model_zoo.hpp"
#include "tvm_utility/pipeline.hpp"

using model_zoo::perception::camera_obstacle_detection::yolo_v3::tensorflow_fp32_coco::config;
// using BoundingBox = autoware_auto_perception_msgs::msg::BoundingBox;

// Name of file containing the human readable names of the classes. One class
// on each line.
static constexpr const char * LABEL_FILENAME = "./yolo_v3_artifacts/labels.txt";

// Name of file containing the anchor values for the network. Each line is one
// anchor. each anchor has 2 comma separated floating point values.
static constexpr const char * ANCHOR_FILENAME = "./yolo_v3_artifacts/anchors.csv";

// Filename of the image on which to run the inference
static constexpr const char * IMAGE_FILENAME = "./yolo_v3_artifacts/test_image_0.jpg";

namespace tvm_utility
{
namespace yolo_v3
{

class PreProcessorYoloV3
  : public tvm_utility::pipeline::PreProcessor<std::string>
{
public:
  explicit PreProcessorYoloV3(tvm_utility::pipeline::InferenceEngineTVMConfig config)
  : network_input_width(config.network_inputs[0].second[1]),
    network_input_height(config.network_inputs[0].second[2]),
    network_input_depth(config.network_inputs[0].second[3]),
    network_datatype_bytes(config.tvm_dtype_bits / 8)
  {
    // Allocate input variable
    std::vector<int64_t> shape_x{1, network_input_width, network_input_height, network_input_depth};
    tvm_utility::pipeline::TVMArrayContainer x{shape_x,
      config.tvm_dtype_code,
      config.tvm_dtype_bits,
      config.tvm_dtype_lanes,
      config.tvm_device_type,
      config.tvm_device_id};

    output = x;
  }

  // The cv::Mat can't be used as an input because it throws an exception when
  // passed as a constant reference
  tvm_utility::pipeline::TVMArrayContainerVector
  schedule(const std::string & input)
  {
    // Read input image
    auto image = cv::imread(input, cv::IMREAD_COLOR);
    if (!image.data) {
      throw std::runtime_error("File " + input + " not found");
    }

    // Compute the ratio for resizing and size for padding
    double scale_x = static_cast<double>(image.size().width) / network_input_width;
    double scale_y = static_cast<double>(image.size().height) / network_input_height;
    double scale = std::max(scale_x, scale_y);

    // Perform padding
    if (scale != 1) {
      cv::resize(image, image, cv::Size(), 1.0f / scale, 1.0f / scale);
    }

    size_t w_pad = network_input_width - image.size().width;
    size_t h_pad = network_input_height - image.size().height;

    if (w_pad || h_pad) {
      cv::copyMakeBorder(
        image, image, h_pad / 2, (h_pad - h_pad / 2),
        w_pad / 2, (w_pad - w_pad / 2), cv::BORDER_CONSTANT,
        cv::Scalar(0, 0, 0));
    }

    // Convert pixel values from int8 to float32. convert pixel value range from 0 - 255 to 0 - 1.
    cv::Mat3f image_3f{};
    image.convertTo(image_3f, CV_32FC3, 1 / 255.0f);

    // cv library uses BGR as a default color format, the network expects the data in RGB format
    cv::cvtColor(image_3f, image_3f, cv::COLOR_BGR2RGB);

    TVMArrayCopyFromBytes(
      output.getArray(), image_3f.data,
      network_input_width * network_input_height *
      network_input_depth * network_datatype_bytes);

    return {output};
  }

private:
  int64_t network_input_width;
  int64_t network_input_height;
  int64_t network_input_depth;
  int64_t network_datatype_bytes;
  tvm_utility::pipeline::TVMArrayContainer output;
};

class PostProcessorYoloV3 : public tvm_utility::pipeline::PostProcessor<std::vector<float>>
{
public:
  PostProcessorYoloV3(
    tvm_utility::pipeline::InferenceEngineTVMConfig config)
  : network_input_width(config.network_inputs[0].second[1]),
    network_input_height(config.network_inputs[0].second[2]),
    network_output_width(config.network_outputs[0].second[1]),
    network_output_height(config.network_outputs[0].second[2]),
    network_output_depth(config.network_outputs[0].second[3])
  {
    // Parse human readable names for the classes
    std::ifstream label_file{LABEL_FILENAME};
    if (!label_file.good()) {
      std::string label_filename = LABEL_FILENAME;
      throw std::runtime_error("unable to open label file:" + label_filename);
    }
    std::string line{};
    while (std::getline(label_file, line)) {
      labels.push_back(line);
    }

    // Get anchor values for this network from the anchor file
    std::ifstream anchor_file{ANCHOR_FILENAME};
    if (!anchor_file.good()) {
      std::string anchor_filename = ANCHOR_FILENAME;
      throw std::runtime_error("unable to open anchor file:" + anchor_filename);
    }
    std::string first{};
    std::string second{};
    while (std::getline(anchor_file, line)) {
      std::stringstream line_stream(line);
      std::getline(line_stream, first, ',');
      std::getline(line_stream, second, ',');
      anchors.push_back(std::make_pair(std::atof(first.c_str()), std::atof(second.c_str())));
    }
  }

  std::vector<float>
  schedule(const tvm_utility::pipeline::TVMArrayContainerVector & input)
  {
    int scale = 0;
    // Vector used to check if the result is accurate,
    // this is also the output of this (schedule) function
    std::vector<float> scores_above_threshold{};
    // Loop to run through the multiple outputs generated for different scale
    for (auto & y : input) {
      int64_t shape_y[] = {1, config.network_outputs[scale].second[1],
        config.network_outputs[scale].second[2],
        network_output_depth};
      auto l_h = shape_y[1];           // layer height
      auto l_w = shape_y[2];           // layer width
      auto n_classes = labels.size();  // total number of classes
      auto n_anchors = anchors.size()/config.network_outputs.size();    // total number of anchors
      auto n_coords = 4;               // number of coordinates in a single anchor box
      auto nudetections = n_classes * n_anchors * l_w * l_h;

      // assert data is stored row-majored in y and the dtype is float
      assert(y.getArray()->strides == nullptr);
      assert(y.getArray()->dtype.bits == sizeof(float) * 8);

      // get a pointer to the output data
      float * data_ptr = reinterpret_cast<float *>(reinterpret_cast<uint8_t *>(y.getArray()->data) +
        y.getArray()->byte_offset);

      // utility function to return data from y given index
      auto get_output_data = [data_ptr, shape_y, n_classes, n_anchors,
          n_coords](auto row_i, auto col_j, auto anchor_k,
          auto offset)
        {
          auto box_index = (row_i * shape_y[2] + col_j) * shape_y[3];
          auto index = box_index + anchor_k * (n_classes + n_coords + 1);
          return data_ptr[index + offset];
        };

      // sigmoid function
      auto sigmoid = [](float x)
        {return static_cast<float>(1.0 / (1.0 + std::exp(-x)));};

      // Parse results into detections. Loop over each detection cell in the model
      // output
      for (size_t i = 0; i < l_w; i++) {
        for (size_t j = 0; j < l_h; j++) {
          for (size_t anchor_k = scale * n_anchors; anchor_k < (scale + 1) * n_anchors;
            anchor_k++)
          {
            float anchor_w = anchors[anchor_k].first;
            float anchor_h = anchors[anchor_k].second;

            // Compute property indices
            auto box_x = get_output_data(i, j, anchor_k, 0);
            auto box_y = get_output_data(i, j, anchor_k, 1);
            auto box_w = get_output_data(i, j, anchor_k, 2);
            auto box_h = get_output_data(i, j, anchor_k, 3);
            auto box_p = get_output_data(i, j, anchor_k, 4);

            // Transform log-space predicted coordinates to absolute space + offset
            // Transform bounding box position from offset to absolute (ratio)
            auto x_coord = (sigmoid(box_x) + j) / l_w;
            auto y_coord = (sigmoid(box_y) + i) / l_h;

            // Transform bounding box height and width from log to absolute space
            auto w = anchor_w * exp(box_w) / network_input_width;
            auto h = anchor_h * exp(box_h) / network_input_height;

            // Decode the confidence of detection in this anchor box
            auto p_0 = sigmoid(box_p);

            // find maximum probability of all classes
            float max_p = 0.0f;
            int max_ind = -1;
            for (int i_class = 0; i_class < n_classes; i_class++) {
              auto class_p = get_output_data(i, j, anchor_k, 5 + i_class);
              if (max_p < class_p) {
                max_p = class_p;
                max_ind = i_class;
              }
            }

            // decode and copy class probabilities
            std::vector<float> class_probabilities{};
            float p_total = 0;
            for (size_t i_class = 0; i_class < n_classes; i_class++) {
              auto class_p = get_output_data(i, j, anchor_k, 5 + i_class);
              class_probabilities.push_back(std::exp(class_p - max_p));
              p_total += class_probabilities[i_class];
            }

            // Find the most likely score
            auto max_score = class_probabilities[max_ind] * p_0 / p_total;

            if (max_score > 0.5) {
              if (bbox_map.count(max_ind) == 0) {
                bbox_map[max_ind] = std::vector<BoundingBox>{};
              }
              bbox_map[max_ind].push_back(
                BoundingBox(
                  x_coord - w / 2,
                  y_coord - h / 2,
                  x_coord + w / 2,
                  y_coord + h / 2,
                  max_score));
            }
          }
        }
      }
      scale++;
    }
    do_nms();
    for (const auto & entry : bbox_map) {
      for (const auto & bbox : entry.second) {
        if (bbox.conf == 0) {
          continue;
        }
        scores_above_threshold.push_back(bbox.conf);
      }
    }
    return scores_above_threshold;
  }

private:
  int64_t network_input_width;
  int64_t network_input_height;
  int64_t network_output_width;
  int64_t network_output_height;
  int64_t network_output_depth;
  std::vector<std::string> labels{};
  std::vector<std::pair<float, float>> anchors{};

  /// Struct for storing detections in image with confidence scores
  struct BoundingBox
  {
    float xmin;
    float ymin;
    float xmax;
    float ymax;
    float conf;

    BoundingBox(float x_min, float y_min, float x_max, float y_max, float conf_)
    : xmin(x_min), ymin(y_min), xmax(x_max), ymax(y_max), conf(conf_) {}
  };

  /// Map for storing the class and detections in the image
  std::map<int, std::vector<BoundingBox>> bbox_map{};

  /// \brief gets the amount of overlap between the two bounding boxes
  /// \return returns the overlap amount

  float interval_overlap(std::pair<float, float> interval_a, std::pair<float, float> interval_b)
  {
    float x1 = interval_a.first;
    float x2 = interval_a.second;
    float x3 = interval_b.first;
    float x4 = interval_b.second;

    if (x3 < x1) {
      if (x4 < x1) {
        return 0.0;
      } else {
        return std::min(x2, x4) - x1;
      }
    } else {
      if (x2 < x3) {
        return 0.0;
      } else {
        return std::min(x2, x4) - x3;
      }
    }
  }

  /// \brief gets the IoU between the two bounding boxes
  /// \return returns the IoU
  float bbox_iou(
    const BoundingBox & box1,
    const BoundingBox & box2)
  {
    float intersect_w = interval_overlap(
      std::make_pair(box1.xmin, box1.xmax),
      std::make_pair(box2.xmin, box2.xmax));
    float intersect_h = interval_overlap(
      std::make_pair(box1.ymin, box1.ymax),
      std::make_pair(box2.ymin, box2.ymax));

    float intersect = intersect_w * intersect_h;

    float w1 = box1.xmax - box1.xmin;
    float h1 = box1.ymax - box1.ymin;
    float w2 = box2.xmax - box2.xmin;
    float h2 = box2.ymax - box2.ymin;

    float union_bbox = w1 * h1 + w2 * h2 - intersect;
    if (union_bbox == 0) {
      return 1.0;
    }
    return intersect / union_bbox;
  }

  /// \brief removes the duplicate detections with lower confidence scores for the same object
  void do_nms()
  {
    for (auto & entry : bbox_map) {
      sort(
        entry.second.begin(), entry.second.end(), [](const BoundingBox & left_bbox,
        const BoundingBox & right_bbox)
        {return left_bbox.conf > right_bbox.conf;});
    }

    for (auto & entry : bbox_map) {
      auto & bboxes = entry.second;
      for (int i = 0; i < bboxes.size(); i++) {
        if (bboxes[i].conf == 0) {
          continue;
        }

        for (int j = i + 1; j < bboxes.size(); j++) {
          if (bbox_iou(bboxes[i], bboxes[j]) >= 0.45) {
            bboxes[j].conf = 0;
          }
        }
      }
    }
  }
};

TEST(PipelineExamplesYoloV3, SimplePipelineYoloV3) {
  // Instantiate the pipeline
  using PrePT = PreProcessorYoloV3;
  using IET = tvm_utility::pipeline::InferenceEngineTVM;
  using PostPT = PostProcessorYoloV3;

  PrePT PreP{config};
  IET IE{config};
  PostPT PostP{config};

  tvm_utility::pipeline::Pipeline<PrePT, IET, PostPT> pipeline(PreP, IE, PostP);

  // Push data input the pipeline and get the output
  auto output = pipeline.schedule(IMAGE_FILENAME);

  // Define reference vector containing expected values, expressed as hexadecimal integers
  std::vector<int32_t> int_output{0x3f7ebc1f, 0x3f79feeb, 0x3f7fd391};

  std::vector<float> expected_output(int_output.size());

  // A memcpy means that the floats in expected_output have a well-defined binary value
  for (int i = 0; i < int_output.size(); i++) {
    memcpy(&expected_output[i], &int_output[i], sizeof(expected_output[i]));
  }

  // Test: check if the generated output is equal to the reference
  EXPECT_EQ(expected_output.size(), output.size()) << "Unexpected output size";
  for (auto i = 0; i < output.size(); ++i) {
    EXPECT_NEAR(expected_output[i], output[i], 0.0001) << "at index: " << i;
  }
}

}  // namespace yolo_v3
}  // namespace tvm_utility

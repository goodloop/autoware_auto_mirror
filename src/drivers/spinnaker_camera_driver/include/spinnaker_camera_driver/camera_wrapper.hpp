// Copyright 2020 Apex.AI, Inc.
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

#ifndef SPINNAKER_CAMERA_DRIVER__CAMERA_WRAPPER_HPP_
#define SPINNAKER_CAMERA_DRIVER__CAMERA_WRAPPER_HPP_

#ifndef DOXYGEN_SKIP
#include <Spinnaker.h>
#endif

#include <spinnaker_camera_driver/camera_settings.hpp>
#include <spinnaker_camera_driver/visibility_control.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <memory>
#include <string>
#include <functional>

namespace autoware
{
namespace drivers
{
namespace camera
{
namespace spinnaker
{

/// A wrapper around the Spinnaker camera.
///
/// It handles correct creation and destruction of the camera along with handling
/// subscriptions to images generated by the camera.
class SPINNAKER_CAMERA_PUBLIC CameraWrapper : public Spinnaker::ImageEventHandler
{
public:
  /// A typedef for the callback function used to return an image message to the user.
  using ImageCallbackFunction = std::function<void (
        std::uint32_t,
        std::unique_ptr<sensor_msgs::msg::Image>)>;

  /// Construct a camera that wraps the spinnaker camera pointer.
  explicit CameraWrapper(
    std::uint32_t camera_index,
    const Spinnaker::CameraPtr & camera,
    const CameraSettings & camera_settings);

  /// Construct a camera that wraps the spinnaker camera pointer.
  explicit CameraWrapper(
    std::uint32_t camera_index,
    const Spinnaker::CameraPtr & camera);

  /// Properly destroys the camera.
  ~CameraWrapper() override;

  // We don't expect this class to be coppied over, so forbid copying.
  CameraWrapper(const CameraWrapper &) = delete;
  CameraWrapper & operator=(const CameraWrapper &) = delete;

  // It is still ok to move this camera.
  CameraWrapper(CameraWrapper &&) = default;
  CameraWrapper & operator=(CameraWrapper &&) = default;

  /// This gets triggered when a camera received an image.
  /// It forwards this event to the parent bridge class.
  void OnImageEvent(Spinnaker::ImagePtr image) override;

  /// Configure a Spinnaker camera.
  void configure_camera(const CameraSettings & camera_settings);

  /// Retreive latest available image.
  std::unique_ptr<sensor_msgs::msg::Image> retreive_image() const;

  /// Start capturing on all cameras.
  void start_capturing();

  /// Stop capturing on all cameras.
  void stop_capturing();

  /// Set the callback function called upon image arrival from the SDK.
  void set_on_image_callback(ImageCallbackFunction callback);

private:
  /// Convert Spinnaker image to image message.
  static std::unique_ptr<sensor_msgs::msg::Image> convert_to_image_msg(
    const Spinnaker::ImagePtr & image, const std::string & frame_id);

  /// Convert a configuration string to Spinnaker PixelFormat enum.
  static Spinnaker::PixelFormatEnums convert_to_pixel_format_enum(const std::string & pixel_format);
  /// Convert Spinnaker PixelFormat enum to string.
  static std::string convert_to_pixel_format_string(Spinnaker::PixelFormatEnums pixel_format);

  /// Index of the current camera.
  std::uint32_t m_camera_index{};
  /// A handle to the Spinnaker camera pointer.
  Spinnaker::CameraPtr m_camera{};
  /// A frame id of this camera.
  std::string m_frame_id{};
  /// An indicator that the camera is configured.
  bool m_is_camera_configured{};
  /// An indicator that the camera is acquiring images.
  bool m_camera_is_capturing{};

  /// A callback that the user can set to receive an image message when a new image is available.
  ImageCallbackFunction m_on_image_callback{};
};

}  //  namespace spinnaker
}  //  namespace camera
}  //  namespace drivers
}  //  namespace autoware

#endif  // SPINNAKER_CAMERA_DRIVER__CAMERA_WRAPPER_HPP_

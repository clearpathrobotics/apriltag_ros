/**
 * Copyright (c) 2017, California Institute of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the California Institute of
 * Technology.
 *
 ** continuous_detector.h ******************************************************
 *
 * Wrapper class of TagDetector class which calls TagDetector::detectTags on
 * each newly arrived image published by a camera.
 *
 * $Revision: 1.0 $
 * $Date: 2017/12/17 13:25:52 $
 * $Author: dmalyuta $
 *
 * Originator:        Danylo Malyuta, JPL
 ******************************************************************************/

#ifndef APRILTAG_ROS_CONTINUOUS_DETECTOR_H
#define APRILTAG_ROS_CONTINUOUS_DETECTOR_H

#include "apriltag_ros/common_functions.h"
#include "image_geometry/pinhole_camera_model.h"

#include <cpr_tf2_ros/camera_frame_transformer.h>
#include <tf/transform_listener.h> // TODO (npalmar): remove this once validPoseSet gets moved out
#include <tf2/LinearMath/Transform.h>

#include <memory>
#include <mutex>

#include <nodelet/nodelet.h>
#include <ros/service_server.h>
#include <std_srvs/Empty.h>

namespace apriltag_ros
{

class ContinuousDetector: public nodelet::Nodelet
{
 public:
  ContinuousDetector() = default;
  ~ContinuousDetector() = default;

  void onInit();

  void imageCallback(const sensor_msgs::ImageConstPtr& image_rect,
                     const sensor_msgs::CameraInfoConstPtr& camera_info);

  void refreshTagParameters();

 private:
  image_geometry::PinholeCameraModel camera_model_;
  cpr_tf2_ros::CameraFrameTransformer camera_frame_transformer_;
  tf2::Transform tag_pose_;
  std_msgs::Header pose_base_frame_header_;
  tf::TransformListener tf_listener_;

  double fov_size_scalar_{ 1.0 }; // scales the size of the image to produce a larger or smaller FOV size (acts as an FOV 'buffer')
  // squared version of min and max detection distances (set these limits widely to not conflict with navigation)
  double min_detection_dist2_{ 0.0 }; // Minimum squared euclidean distance in 3d for the detector to run
  double max_detection_dist2_{ 10.0 }; // Maximum squared euclidean distance in 3d for the detector to run

  ros::Subscriber target_pose_sub_; // subscribes to a topic with the target pose in base frame (in 3d)
  bool in_fov_{ false }; // whether or not the target is in the camera's fov
  bool in_detection_range_{ false }; // whether or not the target is within the config detection limits
  double fov_pixel_buffer_width_;
  double fov_pixel_buffer_height_;

  std::mutex detection_mutex_;
  std::shared_ptr<TagDetector> tag_detector_;
  bool draw_tag_detections_image_;
  cv_bridge::CvImagePtr cv_image_;

  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraSubscriber camera_image_subscriber_;
  image_transport::Publisher tag_detections_image_publisher_;
  ros::Publisher tag_detections_publisher_;

  ros::ServiceServer refresh_params_service_;
  bool refreshParamsCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  // publishes an empty detection
  void publishEmptyDetection(const std_msgs::Header& header);

  // checks if a tag is in the FOV of the camera given the tag pose in optical frame
  bool isTagInFOV(const tf2::Transform& tag_pose) const;

  // checks if a tag is within the minimum and maximum detection range
  bool isTagInDetectionRange(const tf2::Transform& tag_pose) const;

  // get the target pose in the base frame
  void targetPoseCallback(const geometry_msgs::PoseStamped& pose_msg);

  // TODO (npalmar): fix this when it gets moved to cpr_tf2_ros
  bool validPoseSet(const std_msgs::Header& header, tf2::Transform& target_pose,
                                         const bool& base_to_optical);
};

} // namespace apriltag_ros

#endif // APRILTAG_ROS_CONTINUOUS_DETECTOR_H

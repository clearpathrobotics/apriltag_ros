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
 */

#include "apriltag_ros/continuous_detector.h"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(apriltag_ros::ContinuousDetector, nodelet::Nodelet);

namespace apriltag_ros
{
void ContinuousDetector::onInit ()
{
  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& pnh = getPrivateNodeHandle();

  pnh.param("fov_size_scaler", fov_size_scaler_, fov_size_scaler_);
  pnh.param("min_detection_dist", min_detection_dist_, min_detection_dist_);
  pnh.param("max_detection_dist", max_detection_dist_, max_detection_dist_);
  pnh.param("detection_timeout", detection_timeout_, detection_timeout_);

  tag_detector_ = std::shared_ptr<TagDetector>(new TagDetector(pnh));
  draw_tag_detections_image_ = getAprilTagOption<bool>(pnh, 
      "publish_tag_detections_image", false);
  it_ = std::shared_ptr<image_transport::ImageTransport>(
      new image_transport::ImageTransport(nh));

  std::string transport_hint;
  pnh.param<std::string>("transport_hint", transport_hint, "raw");

  int queue_size;
  pnh.param<int>("queue_size", queue_size, 1);
  camera_image_subscriber_ =
      it_->subscribeCamera("image_rect", queue_size,
                          &ContinuousDetector::imageCallback, this,
                          image_transport::TransportHints(transport_hint));
  tag_detections_publisher_ =
      nh.advertise<AprilTagDetectionArray>("tag_detections", 1);
  if (draw_tag_detections_image_)
  {
    tag_detections_image_publisher_ = it_->advertise("tag_detections_image", 1);
  }

  refresh_params_service_ =
      pnh.advertiseService("refresh_tag_params", 
                          &ContinuousDetector::refreshParamsCallback, this);
  
  target_pose_sub_ = nh.subscribe("/target/pose_base_frame", 
                        1, &ContinuousDetector::targetPoseCallback, this);
}

void ContinuousDetector::refreshTagParameters()
{
  // Resetting the tag detector will cause a new param server lookup
  // So if the parameters have changed (by someone/something), 
  // they will be updated dynamically
  std::scoped_lock<std::mutex> lock(detection_mutex_);
  ros::NodeHandle& pnh = getPrivateNodeHandle();
  tag_detector_.reset(new TagDetector(pnh));
}

bool ContinuousDetector::refreshParamsCallback(std_srvs::Empty::Request& req,
                                               std_srvs::Empty::Response& res)
{
  refreshTagParameters();
  return true;
}

void ContinuousDetector::publishEmptyDetection(const std_msgs::Header& header)
{
  AprilTagDetectionArray tag_detection_array;
  tag_detection_array.header = header;
  tag_detections_publisher_.publish(tag_detection_array);
}

cv::Point3f toCvPoint3f(const tf::Vector3& position)
{
  return cv::Point3f(position.getX(), position.getY(), position.getZ());
}

bool ContinuousDetector::isTagInFOV(const tf::Transform& tag_pose) const
{
  if (!camera_model_.initialized())
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "Camera not initialized, skipping FOV check");
    return false;
  }

  if (tag_pose.getOrigin().getZ() <= 0)
  {
    // handles Z = 0 (shouldn't happen) or Z < 0 (behind the camera)
    return false;
  }

  // convert the 3d pose in the image frame to a 2d pixel frame
  const auto image_pt = camera_model_.project3dToPixel(toCvPoint3f(tag_pose.getOrigin()));
  ROS_DEBUG_STREAM_THROTTLE(1.0, 
    "Frame " << camera_model_.cameraInfo().header.frame_id 
      << " has target at pixel coordinate x: " << image_pt.x << ", y: " << image_pt.y);

  // check if the image is within the target fov
  const auto camera_resolution = camera_model_.reducedResolution();
  return (fov_pixel_buffer_width_ <= image_pt.x 
    && image_pt.x < (camera_resolution.width - fov_pixel_buffer_width_) 
    && fov_pixel_buffer_height_ <= image_pt.y 
    && image_pt.y < (camera_resolution.height - fov_pixel_buffer_height_));
}

bool ContinuousDetector::isTagInDetectionRange(const tf::Transform& tag_pose) const
{
  // distance is the z component in the optical frame
  const auto curr_dist = tag_pose.getOrigin().getZ();
  ROS_DEBUG_STREAM_THROTTLE(1.0, 
    "Frame " << camera_model_.cameraInfo().header.frame_id << " has dist to targ: " << curr_dist
      << ", however min is " << min_detection_dist_ << " and max dist is " << max_detection_dist_);
  return min_detection_dist_ <= curr_dist && curr_dist <= max_detection_dist_;
}

void ContinuousDetector::targetPoseCallback(const geometry_msgs::PoseStamped& pose_msg)
{
  tf::poseMsgToTF(pose_msg.pose, tag_pose_);
  robot_pose_header_ = pose_msg.header;
}

bool ContinuousDetector::toOptical(const std_msgs::Header& tag_pose_header, 
  const std::string& optical_frame, tf::Transform& tag_pose)
{
  tf::StampedTransform base_to_optical_tf;
  try
  {
    tf_listener_.lookupTransform(tag_pose_header.frame_id, 
      optical_frame, tag_pose_header.stamp, base_to_optical_tf);
  }
  catch (const tf::TransformException& ex)
  {
    ROS_ERROR_STREAM("Could not get " << tag_pose_header.frame_id 
      << " to " << optical_frame << " transform: " << ex.what()); 
    return false;
  }
  
  tag_pose = base_to_optical_tf.inverse() * tag_pose;
  return true;
}

void ContinuousDetector::imageCallback (
    const sensor_msgs::ImageConstPtr& image_rect,
    const sensor_msgs::CameraInfoConstPtr& camera_info)
{
  std::scoped_lock<std::mutex> lock(detection_mutex_);
  // Lazy updates:
  // When there are no subscribers _and_ when tf is not published,
  // skip detection.
  if (tag_detections_publisher_.getNumSubscribers() == 0 &&
      tag_detections_image_publisher_.getNumSubscribers() == 0 &&
      !tag_detector_->get_publish_tf())
  {
    // ROS_INFO_STREAM("No subscribers and no tf publishing, skip processing.");
    return;
  }

  // Check for an empty image and publish an empty detection message
  if (image_rect->data.empty())
  {
    publishEmptyDetection(image_rect->header);
    return;
  }

  // Transform the tag pose from robot's base frame to optical frame
  if (!toOptical(robot_pose_header_, image_rect->header.frame_id, tag_pose_))
  {
    publishEmptyDetection(image_rect->header);
    return;
  }

  // Set the camera info
  camera_model_.fromCameraInfo(camera_info);
  // compute the pixel values for the FOV pixel buffer (ie. resizing the FOV)
  const auto camera_resolution = camera_model_.reducedResolution();
  const auto fov_scale = 0.5 * (1.0 - fov_size_scaler_);
  fov_pixel_buffer_height_ = camera_resolution.height * fov_scale;
  fov_pixel_buffer_width_ = camera_resolution.width * fov_scale;

  if (!isTagInFOV(tag_pose_) || !isTagInDetectionRange(tag_pose_))
  {
    // only publish an empty detection if the timeout is exceeded
    if ((image_rect->header.stamp - last_valid_detection_).toSec() > detection_timeout_)
    {
      publishEmptyDetection(image_rect->header);
      return;
    }
  }
  else
  {
    last_valid_detection_ = image_rect->header.stamp;
  }

  // Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run
  // AprilTag 2 on the iamge
  try
  {
    cv_image_ = cv_bridge::toCvCopy(image_rect, image_rect->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Publish detected tags in the image by AprilTag 2
  tag_detections_publisher_.publish(
      tag_detector_->detectTags(cv_image_,camera_info));

  // Publish the camera image overlaid by outlines of the detected tags and
  // their payload values
  if (draw_tag_detections_image_)
  {
    tag_detector_->drawDetections(cv_image_);
    tag_detections_image_publisher_.publish(cv_image_->toImageMsg());
  }
}

} // namespace apriltag_ros

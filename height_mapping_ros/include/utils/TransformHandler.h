/*
 * TransformHandler.h
 *
 *  Created on: Dec 19, 2022
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef ROS_UTILS_TRANSFORM_HANDLER_H
#define ROS_UTILS_TRANSFORM_HANDLER_H

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace utils
{
class TransformHandler
{
public:
  /// @brief
  /// @param source_frame The frame where the data originated
  /// @param target_frame The frame to which the data should be transformed
  /// @param time Queried time for transform lookup
  /// @param timeout Time for waiting lookupTransform
  /// @return Pair of bool and transform_stamped. True if succeed to get transform. False otherwise
  std::pair<bool, geometry_msgs::TransformStamped> getTransform(const std::string& source_frame,
                                                                const std::string& target_frame, const ros::Time& time,
                                                                const ros::Duration& timeout)
  {
    try
    {
      auto transform_stamped = tf_buffer_.lookupTransform(target_frame, source_frame, time, timeout);
      return { true, transform_stamped };
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_STREAM_THROTTLE(1, "Failed to look up transform from " << source_frame << " to "
                                                                       << target_frame);  // 0.2us
      return { false, {} };
    }
  }

  /// @brief
  /// @param source_frame The frame where the data originated
  /// @param target_frame The frame to which the data should be transformed
  /// @return Pair of bool and transform_stamped. True if succeed to get transform. False otherwise
  std::pair<bool, geometry_msgs::TransformStamped> getTransform(const std::string& source_frame,
                                                                const std::string& target_frame)
  {
    return getTransform(source_frame, target_frame, ros::Time(0), ros::Duration(0.1));
  }

  /// @brief
  /// @param source_frame The frame where the data originated
  /// @param target_frame The frame to which the data should be transformed
  /// @param time Queried time for transform lookup
  /// @param transform_stamped Transform (translation, rotation) from source to target
  /// @return True if succeed to get transform. False otherwise
  std::pair<bool, geometry_msgs::TransformStamped> getTransform(const std::string& source_frame,
                                                                const std::string& target_frame, const ros::Time& time)
  {
    return getTransform(source_frame, target_frame, time, ros::Duration(0.1));
  }

  void sendTransform(const geometry_msgs::Transform& transform, const std::string& target_frame,
                     const std::string& source_frame, const ros::Time& time)
  {
    geometry_msgs::TransformStamped tf;

    tf.header.stamp = time;
    tf.header.frame_id = target_frame;
    tf.child_frame_id = source_frame;
    tf.transform = transform;

    tf_broadcaster_.sendTransform(tf);
  }

  void sendTransform(const geometry_msgs::TransformStamped& transform_stamped)
  {
    tf_broadcaster_.sendTransform(transform_stamped);
  }

  void sendStaticTransform(const geometry_msgs::Transform& transform, const std::string& target_frame,
                           const std::string& source_frame)
  {
    geometry_msgs::TransformStamped static_tf;

    static_tf.header.stamp = ros::Time::now();
    static_tf.header.frame_id = target_frame;
    static_tf.child_frame_id = source_frame;
    static_tf.transform = transform;

    static_tf_broadcaster_.sendTransform(static_tf);
  }

  void sendStaticTransform(const geometry_msgs::TransformStamped& transform_stamped)
  {
    static_tf_broadcaster_.sendTransform(transform_stamped);
  }

private:
  tf2_ros::Buffer tf_buffer_{};
  tf2_ros::TransformListener tf_listener_{ tf_buffer_ };
  tf2_ros::TransformBroadcaster tf_broadcaster_{};
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_{};
};
// class TransformHandler
}  // namespace utils
#endif  // ROS_UTILS_TRANSFORM_HANDLER_H
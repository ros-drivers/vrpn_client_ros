
/**
*
*  \author     Paul Bovbel <pbovbel@clearpathrobotics.com>
*  \copyright  Copyright (c) 2015, Clearpath Robotics, Inc.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Clearpath Robotics, Inc. nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Please send comments, questions, or patches to code@clearpathrobotics.com
*
*/

#include "vrpn_client_ros/vrpn_client_ros.h"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"

#include "rclcpp/logging.hpp"

#include <vector>
#include <unordered_set>
#include <algorithm>
#include <chrono>

namespace
{
  std::unordered_set<std::string> name_blacklist_({"VRPN Control"});
}


namespace vrpn_client_ros
{
  using namespace std::chrono_literals;

  /**
   * check Ros Names as defined here: http://wiki.ros.org/Names
   */
  bool isInvalidFirstCharInName(const char c)
  {
    return ! ( isalpha(c) || c == '/' || c == '~' );
  }

  bool isInvalidSubsequentCharInName(const char c)
  {
    return ! ( isalnum(c) || c == '/' || c == '_' );
  }

  VrpnTrackerRos::VrpnTrackerRos(std::string tracker_name, ConnectionPtr connection, rclcpp::Node::SharedPtr nh)
  {
    tracker_remote_ = std::make_shared<vrpn_Tracker_Remote>(tracker_name.c_str(), connection.get());

    std::string clean_name = tracker_name;

    
    if (clean_name.size() > 0)
    {
      int start_subsequent = 1;
      if (isInvalidFirstCharInName(clean_name[0])) 
      {
        clean_name = clean_name.substr(1);
        start_subsequent = 0;
      }

      clean_name.erase( std::remove_if( clean_name.begin() + start_subsequent, clean_name.end(), isInvalidSubsequentCharInName ), clean_name.end() );
    }

    init(clean_name, nh, false);
  }

  VrpnTrackerRos::VrpnTrackerRos(std::string tracker_name, std::string host, rclcpp::Node::SharedPtr nh)
  {
    std::string tracker_address;
    tracker_address = tracker_name + "@" + host;
    tracker_remote_ = std::make_shared<vrpn_Tracker_Remote>(tracker_address.c_str());
    init(tracker_name, nh, true);
  }

  void VrpnTrackerRos::init(std::string tracker_name, rclcpp::Node::SharedPtr nh, bool create_mainloop_timer)
  {
    RCLCPP_INFO_STREAM(nh->get_logger(), "Creating new tracker " << tracker_name);

    tracker_remote_->register_change_handler(this, &VrpnTrackerRos::handle_pose);
    tracker_remote_->register_change_handler(this, &VrpnTrackerRos::handle_twist);
    tracker_remote_->register_change_handler(this, &VrpnTrackerRos::handle_accel);
    tracker_remote_->shutup = true;

    rclcpp::expand_topic_or_service_name(
            tracker_name,
            nh->get_name(),
            nh->get_namespace()
    );  // will throw an error if invalid

    this->tracker_name = tracker_name;

    output_nh_ = nh->create_sub_node(tracker_name);

   
    std::string frame_id;
    nh->get_parameter("frame_id", frame_id);
    nh->get_parameter("use_server_time", use_server_time_);
    nh->get_parameter("broadcast_tf", broadcast_tf_);
    nh->get_parameter("calculate_twist_and_accel", calculate_twist_and_accel_);
    
    previous_message_arrived_ = false;

    pose_msg_.header.frame_id = frame_id;

    if (create_mainloop_timer)
    {
      double update_frequency;
      nh->get_parameter("update_frequency", update_frequency);
      mainloop_timer = nh->create_wall_timer(1s/update_frequency, std::bind(&VrpnTrackerRos::mainloop, this));
    }
  }

  VrpnTrackerRos::~VrpnTrackerRos()
  {
    RCLCPP_INFO_STREAM(output_nh_->get_logger(), "Destroying tracker " << tracker_name);
    tracker_remote_->unregister_change_handler(this, &VrpnTrackerRos::handle_pose);
    tracker_remote_->unregister_change_handler(this, &VrpnTrackerRos::handle_twist);
    tracker_remote_->unregister_change_handler(this, &VrpnTrackerRos::handle_accel);
  }

  void VrpnTrackerRos::mainloop()
  {
    tracker_remote_->mainloop();
    mainloop_executed_ = false;
  }

  void VRPN_CALLBACK VrpnTrackerRos::handle_pose(void *userData, const vrpn_TRACKERCB tracker_pose)
  {
    VrpnTrackerRos *tracker = static_cast<VrpnTrackerRos *>(userData);

    if(tracker->mainloop_executed_) {
        RCLCPP_WARN_ONCE(
            tracker->output_nh_->get_logger(), 
            "VRPN update executed multiple times for single mainloop run. Try to adjust your VRPN server settings."
        );
        return;
    }
    tracker->mainloop_executed_ = true;

    rclcpp::Node::SharedPtr nh = tracker->output_nh_;
    
    static tf2_ros::TransformBroadcaster tf_broadcaster(nh);
    
    if (!tracker->pose_pub_)
    {
      tracker->pose_pub_ = nh->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 1);
    }

    if (tracker->use_server_time_)
    {
      tracker->pose_msg_.header.stamp.sec = tracker_pose.msg_time.tv_sec;
      tracker->pose_msg_.header.stamp.nanosec = tracker_pose.msg_time.tv_usec * 1000;
    }
    else
    {
      tracker->pose_msg_.header.stamp = nh->now();
    }

    tracker->pose_msg_.pose.position.x = tracker_pose.pos[0];
    tracker->pose_msg_.pose.position.y = tracker_pose.pos[1];
    tracker->pose_msg_.pose.position.z = tracker_pose.pos[2];

    tracker->pose_msg_.pose.orientation.x = tracker_pose.quat[0];
    tracker->pose_msg_.pose.orientation.y = tracker_pose.quat[1];
    tracker->pose_msg_.pose.orientation.z = tracker_pose.quat[2];
    tracker->pose_msg_.pose.orientation.w = tracker_pose.quat[3];

    tracker->pose_pub_->publish(tracker->pose_msg_);
    
    if (tracker->broadcast_tf_)
    {
      

      if (tracker->use_server_time_)
      {
        tracker->transform_stamped_.header.stamp.sec = tracker_pose.msg_time.tv_sec;
        tracker->transform_stamped_.header.stamp.nanosec = tracker_pose.msg_time.tv_usec * 1000;
      }
      else
      {
        tracker->transform_stamped_.header.stamp = rclcpp::Clock().now();
      }


      tracker->transform_stamped_.child_frame_id = tracker->tracker_name;
      

      tracker->transform_stamped_.header.frame_id = tracker->pose_msg_.header.frame_id;
      tracker->transform_stamped_.transform.translation.x = tracker_pose.pos[0];
      tracker->transform_stamped_.transform.translation.y = tracker_pose.pos[1];
      tracker->transform_stamped_.transform.translation.z = tracker_pose.pos[2];

      tracker->transform_stamped_.transform.rotation.x = tracker_pose.quat[0];
      tracker->transform_stamped_.transform.rotation.y = tracker_pose.quat[1];
      tracker->transform_stamped_.transform.rotation.z = tracker_pose.quat[2];
      tracker->transform_stamped_.transform.rotation.w = tracker_pose.quat[3];

      tf_broadcaster.sendTransform(tracker->transform_stamped_);
    }


    if (tracker->calculate_twist_and_accel_ && tracker->previous_message_arrived_)
    {
      if (!tracker->twist_pub_)
      {
        tracker->twist_pub_ = nh->create_publisher<geometry_msgs::msg::TwistStamped>("twist", 1);
      }


      int64_t dt_nsec = (tracker->pose_msg_.header.stamp.sec*1e-9+tracker->pose_msg_.header.stamp.nanosec) - 
                    (tracker->previous_pose_msg_.header.stamp.sec*1e-9+tracker->previous_pose_msg_.header.stamp.nanosec);

      tracker->twist_msg_.twist.linear.x = (tracker->pose_msg_.pose.position.x - tracker->previous_pose_msg_.pose.position.x)*1e9/dt_nsec;
      tracker->twist_msg_.twist.linear.y = (tracker->pose_msg_.pose.position.y - tracker->previous_pose_msg_.pose.position.y)*1e9/dt_nsec;
      tracker->twist_msg_.twist.linear.z = (tracker->pose_msg_.pose.position.z - tracker->previous_pose_msg_.pose.position.z)*1e9/dt_nsec;

      tf2::Quaternion quat( tracker->pose_msg_.pose.orientation.x, 
                            tracker->pose_msg_.pose.orientation.y, 
                            tracker->pose_msg_.pose.orientation.z, 
                            tracker->pose_msg_.pose.orientation.w);


      auto twist_quat = (quat*tracker->last_quat.inverse())/dt_nsec;
      tracker->twist_msg_.header = tracker->pose_msg_.header;

      double roll, pitch, yaw;
      tf2::Matrix3x3 rot_mat(twist_quat);
      rot_mat.getRPY(roll, pitch, yaw);

      tracker->twist_msg_.twist.angular.x = roll;
      tracker->twist_msg_.twist.angular.y = pitch;
      tracker->twist_msg_.twist.angular.z = yaw;
      

      tracker->twist_pub_->publish(tracker->twist_msg_);

      if (!tracker->accel_pub_)
      {
        tracker->accel_pub_ = nh->create_publisher<geometry_msgs::msg::AccelStamped>("accel", 1);
      }

      tracker->accel_msg_.accel.linear.x = (tracker->twist_msg_.twist.linear.x - tracker->previous_twist_msg_.twist.linear.x)*1e9/dt_nsec;
      tracker->accel_msg_.accel.linear.y = (tracker->twist_msg_.twist.linear.y - tracker->previous_twist_msg_.twist.linear.y)*1e9/dt_nsec;
      tracker->accel_msg_.accel.linear.z = (tracker->twist_msg_.twist.linear.z - tracker->previous_twist_msg_.twist.linear.z)*1e9/dt_nsec;

      tracker->accel_msg_.accel.angular.x = (tracker->twist_msg_.twist.angular.x - tracker->previous_twist_msg_.twist.angular.x)*1e9/dt_nsec;
      tracker->accel_msg_.accel.angular.y = (tracker->twist_msg_.twist.angular.y - tracker->previous_twist_msg_.twist.angular.y)*1e9/dt_nsec;
      tracker->accel_msg_.accel.angular.z = (tracker->twist_msg_.twist.angular.z - tracker->previous_twist_msg_.twist.angular.z)*1e9/dt_nsec;


      tracker->accel_msg_.header = tracker->pose_msg_.header;

      tracker->accel_pub_->publish(tracker->accel_msg_);

      tracker->last_quat = quat;
    }

    tracker->previous_message_arrived_ = true;
    tracker->previous_pose_msg_ = tracker->pose_msg_;
    tracker->previous_twist_msg_ = tracker->twist_msg_;
  }

  void VRPN_CALLBACK VrpnTrackerRos::handle_twist(void *userData, const vrpn_TRACKERVELCB tracker_twist)
  {
    VrpnTrackerRos *tracker = static_cast<VrpnTrackerRos *>(userData);

    
    rclcpp::Node::SharedPtr nh = tracker->output_nh_;
    

    if (!tracker->twist_pub_)
    {
      tracker->twist_pub_ = nh->create_publisher<geometry_msgs::msg::TwistStamped>("twist", 1);
    }

    if (tracker->use_server_time_)
    {
      tracker->twist_msg_.header.stamp.sec = tracker_twist.msg_time.tv_sec;
      tracker->twist_msg_.header.stamp.nanosec = tracker_twist.msg_time.tv_usec * 1000;
    }
    else
    {
      tracker->twist_msg_.header.stamp = nh->now();
    }

    tracker->twist_msg_.twist.linear.x = tracker_twist.vel[0];
    tracker->twist_msg_.twist.linear.y = tracker_twist.vel[1];
    tracker->twist_msg_.twist.linear.z = tracker_twist.vel[2];

    double roll, pitch, yaw;
    tf2::Matrix3x3 rot_mat(
        tf2::Quaternion(tracker_twist.vel_quat[0], tracker_twist.vel_quat[1], tracker_twist.vel_quat[2],
                        tracker_twist.vel_quat[3]));
    rot_mat.getRPY(roll, pitch, yaw);

    tracker->twist_msg_.twist.angular.x = roll;
    tracker->twist_msg_.twist.angular.y = pitch;
    tracker->twist_msg_.twist.angular.z = yaw;

    tracker->twist_pub_->publish(tracker->twist_msg_);
    
  }

  void VRPN_CALLBACK VrpnTrackerRos::handle_accel(void *userData, const vrpn_TRACKERACCCB tracker_accel)
  {
    VrpnTrackerRos *tracker = static_cast<VrpnTrackerRos *>(userData);

    rclcpp::Node::SharedPtr nh = tracker->output_nh_;


    if (!tracker->accel_pub_)
    {
      tracker->accel_pub_ = nh->create_publisher<geometry_msgs::msg::AccelStamped>("accel", 1);
    }


    if (tracker->use_server_time_)
    {
      tracker->accel_msg_.header.stamp.sec = tracker_accel.msg_time.tv_sec;
      tracker->accel_msg_.header.stamp.nanosec = tracker_accel.msg_time.tv_usec * 1000;
    }
    else
    {
      tracker->accel_msg_.header.stamp = nh->now();
    }

    tracker->accel_msg_.accel.linear.x = tracker_accel.acc[0];
    tracker->accel_msg_.accel.linear.y = tracker_accel.acc[1];
    tracker->accel_msg_.accel.linear.z = tracker_accel.acc[2];

    double roll, pitch, yaw;
    tf2::Matrix3x3 rot_mat(
        tf2::Quaternion(tracker_accel.acc_quat[0], tracker_accel.acc_quat[1], tracker_accel.acc_quat[2],
                        tracker_accel.acc_quat[3]));
    rot_mat.getRPY(roll, pitch, yaw);

    tracker->accel_msg_.accel.angular.x = roll;
    tracker->accel_msg_.accel.angular.y = pitch;
    tracker->accel_msg_.accel.angular.z = yaw;

    tracker->accel_pub_->publish(tracker->accel_msg_);
  
  }

  VrpnClientRos::VrpnClientRos(rclcpp::Node::SharedPtr nh, rclcpp::Node::SharedPtr private_nh)
  {
    output_nh_ = private_nh;

    nh->declare_parameter("server", "localhost");
    nh->declare_parameter("port", 3883);
    nh->declare_parameter("update_frequency", 100.0);
    nh->declare_parameter("frame_id", "world");
    nh->declare_parameter("use_server_time", false);
    nh->declare_parameter("broadcast_tf", true);
    nh->declare_parameter("refresh_tracker_frequency", 5.0);
    nh->declare_parameter("calculate_twist_and_accel", true);

    std::vector<std::string> param_tracker_names;
    nh->declare_parameter("trackers", param_tracker_names);


    host_ = getHostStringFromParams(private_nh);

    RCLCPP_INFO_STREAM(output_nh_->get_logger(), "Connecting to VRPN server at " << host_);
    connection_ = std::shared_ptr<vrpn_Connection>(vrpn_get_connection_by_name(host_.c_str()));
    RCLCPP_INFO(output_nh_->get_logger(), "Connection established");

    double update_frequency;
    private_nh->get_parameter("update_frequency", update_frequency);


    mainloop_timer = nh->create_wall_timer(1s/update_frequency, std::bind(&VrpnClientRos::mainloop, this));

    double refresh_tracker_frequency;
    private_nh->get_parameter("refresh_tracker_frequency", refresh_tracker_frequency);

    if (refresh_tracker_frequency > 0.0)
    {
      refresh_tracker_timer = nh->create_wall_timer(1s/refresh_tracker_frequency, 
              std::bind(&VrpnClientRos::updateTrackers, this));
    }

    std::vector<std::string> param_tracker_names_;
    if (private_nh->get_parameter("trackers", param_tracker_names_))
    {
      for (std::vector<std::string>::iterator it = param_tracker_names_.begin();
           it != param_tracker_names_.end(); ++it)
      {
        trackers_.insert(std::make_pair(*it, std::make_shared<VrpnTrackerRos>(*it, connection_, output_nh_)));
      }
    }
  }

  std::string VrpnClientRos::getHostStringFromParams(rclcpp::Node::SharedPtr host_nh)
  {
    std::stringstream host_stream;
    std::string server;
    int port;


    host_nh->get_parameter("server", server);
    host_stream << server;

    if (host_nh->get_parameter("port", port))
    {
      host_stream << ":" << port;
    }
    return host_stream.str();
  }

  void VrpnClientRos::mainloop()
  {
    connection_->mainloop();
    if (!connection_->doing_okay())
    {
      RCLCPP_WARN(output_nh_->get_logger(), "VRPN connection is not 'doing okay'");
    }
    for (TrackerMap::iterator it = trackers_.begin(); it != trackers_.end(); ++it)
    {
      it->second->mainloop();
    }
  }

  void VrpnClientRos::updateTrackers()
  {
    int i = 0;
    while (connection_->sender_name(i) != NULL)
    {
      if (trackers_.count(connection_->sender_name(i)) == 0 && name_blacklist_.count(connection_->sender_name(i)) == 0)
      {
        RCLCPP_INFO_STREAM(output_nh_->get_logger(), "Found new sender: " << connection_->sender_name(i));
        trackers_.insert(std::make_pair(connection_->sender_name(i),
                                        std::make_shared<VrpnTrackerRos>(connection_->sender_name(i), connection_,
                                                                           output_nh_)));
      }
      i++;
    }
  }
}  // namespace vrpn_client_ros
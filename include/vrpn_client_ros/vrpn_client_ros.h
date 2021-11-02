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

#ifndef VRPN_CLIENT_ROS_VRPN_CLIENT_ROS_H
#define VRPN_CLIENT_ROS_VRPN_CLIENT_ROS_H

#include "vrpn_client_ros/vrpn_client_ros.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.h"
#include "geometry_msgs/msg/accel_stamped.h"
#include "geometry_msgs/msg/transform_stamped.h"

#include <vrpn_Tracker.h>
#include <vrpn_Connection.h>
#include <map>
#include <string>
#include <unordered_map>

namespace vrpn_client_ros
{

  typedef std::shared_ptr<vrpn_Connection> ConnectionPtr;
  typedef std::shared_ptr<vrpn_Tracker_Remote> TrackerRemotePtr;

  class VrpnTrackerRos
  {
  public:

    typedef std::shared_ptr<VrpnTrackerRos> Ptr;
    /**
     * Create and initialize VrpnTrackerRos using an existing underlying VRPN connection object. The underlying
     * connection object is responsible for calling the tracker's mainloop.
     */
    VrpnTrackerRos(std::string tracker_name, ConnectionPtr connection, rclcpp::Node::SharedPtr nh);

    /**
     * Create and initialize VrpnTrackerRos, creating a new connection to tracker_name@host. This constructor will
     * register timer callbacks on nh to call mainloop.
     */
    VrpnTrackerRos(std::string tracker_name, std::string host, rclcpp::Node::SharedPtr nh);

    ~VrpnTrackerRos();

    /**
     * Call mainloop of underlying vrpn_Tracker_Remote
     */
    void mainloop();

  private:
    TrackerRemotePtr tracker_remote_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    //std::vector<ros::Publisher> pose_pubs_, twist_pubs_, accel_pubs_;
    rclcpp::Node::SharedPtr output_nh_;
    bool use_server_time_, broadcast_tf_, mainloop_executed_;
    std::string tracker_name;

    rclcpp::TimerBase::SharedPtr mainloop_timer;

    geometry_msgs::msg::PoseStamped pose_msg_;
    // geometry_msgs::TwistStamped twist_msg_;
    // geometry_msgs::AccelStamped accel_msg_;
    // geometry_msgs::TransformStamped transform_stamped_;

    void init(std::string tracker_name, rclcpp::Node::SharedPtr nh, bool create_mainloop_timer);

    static void VRPN_CALLBACK handle_pose(void *userData, const vrpn_TRACKERCB tracker_pose);

    static void VRPN_CALLBACK handle_twist(void *userData, const vrpn_TRACKERVELCB tracker_twist);

    static void VRPN_CALLBACK handle_accel(void *userData, const vrpn_TRACKERACCCB tracker_accel);
  };

  class VrpnClientRos
  {
  public:
    typedef std::shared_ptr<VrpnClientRos> Ptr;
    typedef std::unordered_map<std::string, VrpnTrackerRos::Ptr> TrackerMap;

    /**
     * Create and initialize VrpnClientRos object in the private_nh namespace.
     */
    VrpnClientRos(rclcpp::Node::SharedPtr nh, rclcpp::Node::SharedPtr private_nh);

    static std::string getHostStringFromParams(rclcpp::Node::SharedPtr host_nh);

    /**
     * Call mainloop of underlying VRPN connection and all registered VrpnTrackerRemote objects.
     */
    void mainloop();

    /**
     * Examine vrpn_Connection's senders and create new trackers as necessary.
     */
    void updateTrackers();

  private:
    std::string host_;
    rclcpp::Node::SharedPtr output_nh_;

    /**
     * Underlying VRPN connection object
     */
    ConnectionPtr connection_;

    /**
     * Map of registered trackers, accessible by name
     */
    TrackerMap trackers_;


    rclcpp::TimerBase::SharedPtr refresh_tracker_timer, mainloop_timer;
  };
}  // namespace vrpn_client_ros

#endif  // VRPN_CLIENT_ROS_VRPN_CLIENT_ROS_H
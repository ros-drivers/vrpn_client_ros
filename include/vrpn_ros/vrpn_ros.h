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

#ifndef VRPN_ROS_VRPN_ROS_H
#define VRPN_ROS_VRPN_ROS_H

#include "vrpn_ros/vrpn_ros.h"

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/AccelStamped.h"
#include "geometry_msgs/TransformStamped.h"

#include <vrpn_Tracker.h>
#include <vrpn_Connection.h>
#include <map>
#include <string>

namespace vrpn_ros
{

  typedef boost::shared_ptr<vrpn_Connection> ConnectionPtr;
  typedef boost::shared_ptr<vrpn_Tracker_Remote> TrackerRemotePtr;

  class VrpnTrackerRos
  {
  public:
    /**
     * Create and initialize VrpnTrackerRos using an existing underlying VRPN connection object.
     */
    VrpnTrackerRos(std::string tracker_name, ConnectionPtr connection, ros::NodeHandle nh);

    /**
     * Create and initialize VrpnTrackerRos, creating a new connection to tracker_name@host.
     */
    VrpnTrackerRos(std::string tracker_name, std::string host, ros::NodeHandle nh);

    ~VrpnTrackerRos();

    void init(std::string tracker_name, ros::NodeHandle nh, bool create_mainloop_timer);

    /**
     * Call mainloop of underlying vrpn_Tracker_Remote
     */
    void mainloop();

  private:
    TrackerRemotePtr tracker_remote_;
    ros::Publisher pose_pub, twist_pub, accel_pub;
    ros::NodeHandle output_nh_;
    bool use_server_time_, broadcast_tf_;

    ros::Timer mainloop_timer;

    geometry_msgs::PoseStamped pose_msg_;
    geometry_msgs::TwistStamped twist_msg_;
    geometry_msgs::AccelStamped accel_msg_;
    geometry_msgs::TransformStamped transform_stamped_;

    static void VRPN_CALLBACK handle_pose(void *userData, const vrpn_TRACKERCB tracker_pose);

    static void VRPN_CALLBACK handle_twist(void *userData, const vrpn_TRACKERVELCB tracker_twist);

    static void VRPN_CALLBACK handle_accel(void *userData, const vrpn_TRACKERACCCB tracker_accel);
  };

  class VrpnClientRos
  {
  public:
    /**
     * Create and initialize VrpnClientRos object in the private_nh namespace.
     */
    VrpnClientRos(ros::NodeHandle nh, ros::NodeHandle private_nh);

    static std::string getHostStringFromParams(ros::NodeHandle host_nh);

    /**
     * Call mainloop of underlying VRPN connection and all registered VrpnTrackerRemote objects.
     */
    void mainloop();

    /**
     * Examine vrpn_Connection and create or remove tracker objects as necessary
     */
    void updateTrackers();

  private:
    std::string host_, frame_id_;
    bool use_server_time_, broadcast_tf_;
    ros::NodeHandle output_nh_;

    /**
     * Underlying VRPN connection object
     */
    ConnectionPtr connection_;

    /**
     * Map of registered trackers, accessible by name
     */
    std::map<std::string, VrpnTrackerRos> trackers_;

    ros::Timer refresh_tracker_timer_, mainloop_timer;
  };
}  // namespace vrpn_ros

#endif  // VRPN_ROS_VRPN_ROS_H

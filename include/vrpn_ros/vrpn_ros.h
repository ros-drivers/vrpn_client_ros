//
// Created by pbovbel on 12/06/15.
//

#ifndef PROJECT_VRPN_ROS_H
#define PROJECT_VRPN_ROS_H


#include "vrpn_ros/vrpn_ros.h"
#include <vrpn_Tracker.h>
#include <vrpn_Connection.h>
#include <vrpn_Types.h>
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>

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
    VrpnTrackerRos(std::string tracker_name, ConnectionPtr connection, ros::NodeHandle nh, std::string frame_id,
                   bool use_server_time, bool broadcast_tf);

    /**
     * Create and initialize VrpnTrackerRos, creating a new connection to tracker_name@host.
     */
    VrpnTrackerRos(std::string tracker_name, std::string host, ros::NodeHandle nh, std::string frame_id,
                   bool use_server_time, bool broadcast_tf);

    ~VrpnTrackerRos();

    void spin();

  private:

    TrackerRemotePtr tracker_remote_;
    ros::Publisher pose_pub, twist_pub, accel_pub;
    ros::NodeHandle nh_;
    bool use_server_time_, broadcast_tf_;

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

    /**
     * Spin mainloop of underlying VRPN connection and all registered VrpnTrackerRemote objects.
     */
    void spin();

    /**
     * Examine vrpn_Connection and create or remove tracker objects as necessary
     */
    void updateTrackers();
  };
}

#endif //PROJECT_VRPN_ROS_H

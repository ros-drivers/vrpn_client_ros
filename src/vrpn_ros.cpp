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

    VrpnTrackerRos(std::string tracker_name, ConnectionPtr connection, ros::NodeHandle nh, std::string frame_id,
                   bool use_server_time, bool broadcast_tf)
        : nh_(nh, tracker_name), use_server_time_(use_server_time), broadcast_tf_(broadcast_tf)
    {
//      nh_ = ros::NodeHandle(nh, tracker_name);
//      frame_id_ = frame_id;
//      use_server_time_ = use_server_time;
//      broadcast_tf_ = broadcast_tf;

      pose_msg.header.frame_id = twist_msg.header.frame_id = accel_msg.header.frame_id = transform_stamped.header.frame_id = frame_id;
      transform_stamped.child_frame_id = tracker_name;

      tracker_remote_ = boost::make_shared<vrpn_Tracker_Remote>(tracker_name.c_str(), connection.get());
      tracker_remote_->register_change_handler(this, &VrpnTrackerRos::handle_pose);
      tracker_remote_->register_change_handler(this, &VrpnTrackerRos::handle_twist);
      tracker_remote_->register_change_handler(this, &VrpnTrackerRos::handle_accel);
    }

    ~VrpnTrackerRos()
    {
      tracker_remote_->unregister_change_handler(this, &VrpnTrackerRos::handle_pose);
      tracker_remote_->unregister_change_handler(this, &VrpnTrackerRos::handle_twist);
      tracker_remote_->unregister_change_handler(this, &VrpnTrackerRos::handle_accel);
    }

    void mainloop()
    {
      tracker_remote_->mainloop();
    }

  private:

    TrackerRemotePtr tracker_remote_;
    ros::Publisher pose_pub, twist_pub, accel_pub;
    ros::NodeHandle nh_;
    bool use_server_time_, broadcast_tf_;

    geometry_msgs::PoseStamped pose_msg;
    geometry_msgs::TwistStamped twist_msg;
    geometry_msgs::AccelStamped accel_msg;
    geometry_msgs::TransformStamped transform_stamped;

    static void VRPN_CALLBACK handle_pose(void *userData, const vrpn_TRACKERCB tracker_pose)
    {

      VrpnTrackerRos *tracker = (VrpnTrackerRos *) userData;
      if (tracker->pose_pub.getTopic().empty())
      {
        tracker->pose_pub = tracker->nh_.advertise<geometry_msgs::PoseStamped>("pose", 1);
      }

      if (tracker->pose_pub.getNumSubscribers() > 0)
      {
        if (tracker->use_server_time_)
        {
          tracker->pose_msg.header.stamp.sec = tracker_pose.msg_time.tv_sec;
          tracker->pose_msg.header.stamp.nsec = tracker_pose.msg_time.tv_usec * 1000;
        }
        else
        {
          tracker->pose_msg.header.stamp = ros::Time::now();
        }

        tracker->pose_msg.pose.position.x = tracker_pose.pos[0];
        tracker->pose_msg.pose.position.y = tracker_pose.pos[1];
        tracker->pose_msg.pose.position.z = tracker_pose.pos[2];

        tracker->pose_msg.pose.orientation.x = tracker_pose.quat[0];
        tracker->pose_msg.pose.orientation.y = tracker_pose.quat[1];
        tracker->pose_msg.pose.orientation.z = tracker_pose.quat[2];
        tracker->pose_msg.pose.orientation.w = tracker_pose.quat[3];

        tracker->pose_pub.publish(tracker->pose_msg);
      }

      if (tracker->broadcast_tf_)
      {

        static tf2_ros::TransformBroadcaster tf_broadcaster;
        tracker->transform_stamped.transform.translation.x = tracker_pose.pos[0];
        tracker->transform_stamped.transform.translation.y = tracker_pose.pos[1];
        tracker->transform_stamped.transform.translation.z = tracker_pose.pos[2];

        tracker->transform_stamped.transform.rotation.x = tracker_pose.quat[0];
        tracker->transform_stamped.transform.rotation.y = tracker_pose.quat[1];
        tracker->transform_stamped.transform.rotation.z = tracker_pose.quat[2];
        tracker->transform_stamped.transform.rotation.w = tracker_pose.quat[3];

        tf_broadcaster.sendTransform(tracker->transform_stamped);

      }

    }

    static void VRPN_CALLBACK handle_twist(void *userData, const vrpn_TRACKERVELCB tracker_twist)
    {
      VrpnTrackerRos *tracker = (VrpnTrackerRos *) userData;
      if (tracker->twist_pub.getTopic().empty())
      {
        tracker->twist_pub = tracker->nh_.advertise<geometry_msgs::TwistStamped>("twist", 1);
      }

      if (tracker->twist_pub.getNumSubscribers() > 0)
      {

        if (tracker->use_server_time_)
        {
          tracker->twist_msg.header.stamp.sec = tracker_twist.msg_time.tv_sec;
          tracker->twist_msg.header.stamp.nsec = tracker_twist.msg_time.tv_usec * 1000;
        }
        else
        {
          tracker->twist_msg.header.stamp = ros::Time::now();
        }

        tracker->twist_msg.twist.linear.x = tracker_twist.vel[0];
        tracker->twist_msg.twist.linear.y = tracker_twist.vel[1];
        tracker->twist_msg.twist.linear.z = tracker_twist.vel[2];

        double roll, pitch, yaw;
        tf2::Matrix3x3 rot_mat(
            tf2::Quaternion(tracker_twist.vel_quat[0], tracker_twist.vel_quat[1], tracker_twist.vel_quat[2],
                            tracker_twist.vel_quat[3]));
        rot_mat.getRPY(roll, pitch, yaw);

        tracker->twist_msg.twist.angular.x = roll;
        tracker->twist_msg.twist.angular.y = pitch;
        tracker->twist_msg.twist.angular.z = yaw;

        tracker->twist_pub.publish(tracker->twist_msg);
      }
    }

    static void VRPN_CALLBACK handle_accel(void *userData, const vrpn_TRACKERACCCB tracker_accel)
    {
      VrpnTrackerRos *tracker = (VrpnTrackerRos *) userData;
      if (tracker->accel_pub.getTopic().empty())
      {
        tracker->accel_pub = tracker->nh_.advertise<geometry_msgs::AccelStamped>("accel", 1);
      }

      if (tracker->accel_pub.getNumSubscribers() > 0)
      {
        if (tracker->use_server_time_)
        {
          tracker->accel_msg.header.stamp.sec = tracker_accel.msg_time.tv_sec;
          tracker->accel_msg.header.stamp.nsec = tracker_accel.msg_time.tv_usec * 1000;
        }
        else
        {
          tracker->accel_msg.header.stamp = ros::Time::now();
        }

        tracker->accel_msg.accel.linear.x = tracker_accel.acc[0];
        tracker->accel_msg.accel.linear.y = tracker_accel.acc[1];
        tracker->accel_msg.accel.linear.z = tracker_accel.acc[2];

        double roll, pitch, yaw;
        tf2::Matrix3x3 rot_mat(
            tf2::Quaternion(tracker_accel.acc_quat[0], tracker_accel.acc_quat[1], tracker_accel.acc_quat[2],
                            tracker_accel.acc_quat[3]));
        rot_mat.getRPY(roll, pitch, yaw);

        tracker->accel_msg.accel.angular.x = roll;
        tracker->accel_msg.accel.angular.y = pitch;
        tracker->accel_msg.accel.angular.z = yaw;

        tracker->accel_pub.publish(tracker->accel_msg);

      }
    }

  };

  class VrpnClientRos
  {

  public:


    typedef std::map<std::string, VrpnTrackerRos> TrackerMap;


    VrpnClientRos(ros::NodeHandle nh, ros::NodeHandle private_nh)
    {

      output_nh_ = private_nh;

      private_nh.param<std::string>("frame_id", frame_id_, "world");
      private_nh.param<bool>("use_server_time", use_server_time_, false);
      private_nh.param<bool>("broadcast_tf", broadcast_tf_, false);

      std::stringstream address_stream;
      std::string server;
      int port;

      private_nh.param<std::string>("server", server, "localhost");
      address_stream << server;

      if (private_nh.getParam("port", port))
      {
        address_stream << ":" << port;
      }
      address_ = address_stream.str();

      connection_ = boost::shared_ptr<vrpn_Connection>(vrpn_get_connection_by_name(address_.c_str()));

      double update_frequency;
      private_nh.param<double>("update_frequency", update_frequency, 100.0);
      mainloop_timer = nh.createTimer(ros::Duration(1 / update_frequency), boost::bind(&VrpnClientRos::mainloop, this));

      double refresh_tracker_frequency;
      private_nh.param<double>("refresh_tracker_frequency", refresh_tracker_frequency, 0.0);

      if (refresh_tracker_frequency > 0.0)
      {
        refresh_tracker_timer_ = nh.createTimer(ros::Duration(1 / refresh_tracker_frequency),
                                                boost::bind(&VrpnClientRos::updateTrackers, this));
      }

      std::vector<std::string> param_tracker_names_;
      if (nh.getParam("trackers", param_tracker_names_))
      {
        for (std::vector<std::string>::iterator it = param_tracker_names_.begin();
             it != param_tracker_names_.end(); ++it)
        {
          trackers_.insert(std::make_pair(*it, VrpnTrackerRos(*it, connection_, output_nh_, frame_id_, use_server_time_,
                                                              broadcast_tf_)));
        }
      }
    }

  private:

    std::string address_, frame_id_;
    TrackerMap trackers_;
    ConnectionPtr connection_;
    ros::NodeHandle output_nh_;
    bool use_server_time_, broadcast_tf_;

    ros::Timer refresh_tracker_timer_, mainloop_timer;

    void mainloop()
    {
      connection_->mainloop();
      for (TrackerMap::iterator it = trackers_.begin(); it != trackers_.end(); ++it)
      {
        it->second.mainloop();
      }
    }

    void updateTrackers()
    {

      std::vector<std::string> tracker_names, sender_names, to_create, to_remove;

      int i = 0;
      while (connection_->sender_name(i) != NULL)
      {
        sender_names.push_back(std::string(connection_->sender_name(i++)));
      }

      for (TrackerMap::iterator it = trackers_.begin(); it != trackers_.end(); ++it)
      {
        tracker_names.push_back(it->first);
      }

      std::set_difference(sender_names.begin(), sender_names.end(), tracker_names.begin(), tracker_names.end(),
                          std::inserter(to_create, to_create.begin()));
      std::set_difference(tracker_names.begin(), tracker_names.end(), sender_names.begin(), sender_names.end(),
                          std::inserter(to_remove, to_remove.begin()));

      for (std::vector<std::string>::iterator it = to_create.begin(); it != to_create.end(); ++it)
      {
        trackers_.insert(std::make_pair(*it, VrpnTrackerRos(*it, connection_,
                                                            output_nh_, frame_id_, use_server_time_, broadcast_tf_)));
      }

      for (std::vector<std::string>::iterator it = to_remove.begin(); it != to_remove.end(); ++it)
      {
        trackers_.erase(*it);
      }

    }


  };

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vrpn_ros_node");

  ros::NodeHandle nh, private_nh("~");

  vrpn_ros::VrpnClientRos vrpn(nh, private_nh);

  ros::spin();
  return 0;
}


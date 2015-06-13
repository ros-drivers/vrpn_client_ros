//
// Created by pbovbel on 12/06/15.
//

#include "vrpn_ros/vrpn_ros.h"

#include <vrpn_Tracker.h>
#include <vrpn_Connection.h>
#include <vrpn_Types.h>
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>

ros::Time last_time;

int position_m_id, velocity_m_id, accel_m_id;
std::string frame_id;


//
//void VRPN_CALLBACK handle_tracker(void* userData, const vrpn_TRACKERCB t )
//{
//
//  ros::Time this_time = ros::Time::now();
//
//  ros::Duration elapsed = this_time - last_time;
//  last_time = this_time;
//
////  ROS_INFO_STREAM("Tracker " << t.sensor << "' : " << t.pos[0] << "," <<  t.pos[1] << "," << t.pos[2]);
//  ROS_INFO_STREAM("F " << 1/elapsed.toSec());
//
//}
//
//void VRPN_CALLBACK handle_tracker_vel(void* userData, const vrpn_TRACKERVELCB t )
//{
//  std::cout << "Tracker '" << t.sensor << "' : " << t.vel[0] << "," <<  t.vel[1] << "," << t.vel[2] << std::endl;
//}

typedef boost::shared_ptr<vrpn_Connection> ConnectionPtr;
//typedef boost::shared_ptr<vrpn_Tracker_Remote> TrackerPtr;

boost::shared_ptr<vrpn_Connection> connection_;
//std::vector<TrackerPtr> trackers_;

struct TrackerRemoteExposer : vrpn_Tracker_Remote
{
  using vrpn_Tracker_Remote::handle_change_message;
  using vrpn_Tracker_Remote::handle_vel_change_message;
  using vrpn_Tracker_Remote::handle_acc_change_message;
};

struct ConnectionExposer : vrpn_Connection
{
  using vrpn_Connection::message_type_is_registered;
};

int handle_all(void *userdata, vrpn_HANDLERPARAM message)
{

  position_m_id = ((connection_.get())->*(&ConnectionExposer::message_type_is_registered))(
      "vrpn_Tracker Pos_Quat");
  velocity_m_id = ((connection_.get())->*(&ConnectionExposer::message_type_is_registered))(
      "vrpn_Tracker Velocity");
  accel_m_id = ((connection_.get())->*(&ConnectionExposer::message_type_is_registered))(
      "vrpn_Tracker Acceleration");

  ROS_INFO_STREAM("Got " << connection_->message_type_name(message.type) << " from " <<
                  connection_->sender_name(message.sender));

  std_msgs::Header header;
  header.frame_id = frame_id;

//  if(true/*TODO*/){
//  header.stamp = ros::Time(message.msg_time.tv_sec, message.msg_time.tv_usec*1000);
//  }else{
    header.stamp = ros::Time::now();
//  }

  ROS_INFO_STREAM(message.type << " " << position_m_id << " " << velocity_m_id);

  if (message.type == position_m_id)
  {
    geometry_msgs::PoseStamped pose;
    vrpn_int32 sensor, padding;

    vrpn_unbuffer(&message.buffer, &sensor);
    vrpn_unbuffer(&message.buffer, &padding);

    vrpn_unbuffer(&message.buffer, &pose.pose.position.x);
    vrpn_unbuffer(&message.buffer, &pose.pose.position.y);
    vrpn_unbuffer(&message.buffer, &pose.pose.position.z);
    vrpn_unbuffer(&message.buffer, &pose.pose.orientation.w);
    vrpn_unbuffer(&message.buffer, &pose.pose.orientation.x);
    vrpn_unbuffer(&message.buffer, &pose.pose.orientation.y);
    vrpn_unbuffer(&message.buffer, &pose.pose.orientation.z);


    pose.header = header;
    ROS_INFO_STREAM(pose);

  }
  else if (message.type == velocity_m_id)
  {

  }
  else if (message.type == accel_m_id)
  {

  }

  return 0;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "vrpn_ros_node");

  ros::NodeHandle nh, private_nh("~");

//  VrpnRos(nh, private_nh);

  connection_ = boost::shared_ptr<vrpn_Connection>(vrpn_get_connection_by_name("10.27.15.72"));

//  int got_connection_type = ((connection_.get())->*(&ConnectionMessageIdExposer::message_type_is_registered))(vrpn_got_connection);
//  connection_->register_handler(got_connection_type, got_connection_handler, NULL, vrpn_ANY_SENDER);

//  int dropped_connection_type = ((connection_.get())->*(&ConnectionMessageIdExposer::message_type_is_registered))(vrpn_got_connection);
//  connection_->register_handler(dropped_connection_type, dropped_connection_handler, NULL, vrpn_ANY_SENDER);

  connection_->mainloop();

  position_m_id = ((connection_.get())->*(&ConnectionExposer::message_type_is_registered))(
      "vrpn_Tracker Pos_Quat");
  velocity_m_id = ((connection_.get())->*(&ConnectionExposer::message_type_is_registered))(
      "vrpn_Tracker Velocity");
  accel_m_id = ((connection_.get())->*(&ConnectionExposer::message_type_is_registered))(
      "vrpn_Tracker Acceleration");

  connection_->register_handler(vrpn_ANY_TYPE, handle_all, NULL, vrpn_ANY_SENDER);

  ros::Rate r(100);
  while (ros::ok())
  {
    connection_->mainloop();

//    for(int i = 0; i < 10; i ++)
//    {
//      ROS_INFO_STREAM(i << " " << connection->sender_name(i));
//    }

    r.sleep();
  }

//  ros::spin();
  return 0;
}
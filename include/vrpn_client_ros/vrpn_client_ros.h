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

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/AccelStamped.h"
#include "geometry_msgs/TransformStamped.h"

#include <vrpn_Tracker.h>
#include <vrpn_Connection.h>
#include <map>
#include <string>
#include <unordered_map>

#include "LowPassFilter.h"
#include "filtered_derivative_wcb.h"
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

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
    VrpnTrackerRos(std::string tracker_name, ConnectionPtr connection, ros::NodeHandle nh);

    /**
     * Create and initialize VrpnTrackerRos, creating a new connection to tracker_name@host. This constructor will
     * register timer callbacks on nh to call mainloop.
     */
    VrpnTrackerRos(std::string tracker_name, std::string host, ros::NodeHandle nh);

    ~VrpnTrackerRos();

    /**
     * Call mainloop of underlying vrpn_Tracker_Remote
     */
    void mainloop();

  private:
    TrackerRemotePtr tracker_remote_;
    std::vector<ros::Publisher> pose_pubs_, body_pose_pubs_, twist_pubs_, accel_pubs_, dead_zone_pubs_;
    ros::NodeHandle output_nh_;
    bool use_server_time_, broadcast_tf_, process_sensor_id_;
    std::string tracker_name;

    ros::Timer mainloop_timer;

    geometry_msgs::PoseStamped pose_msg_;
    nav_msgs::Odometry pose_and_velocity_msg_, body_pose_and_velocity_msg_;
    std_msgs::Bool dead_zone_msg_;
    geometry_msgs::TwistStamped twist_msg_;
    geometry_msgs::AccelStamped accel_msg_;
    geometry_msgs::TransformStamped transform_stamped_;

    /**
      * Ciruclar buffer to generate velocities
     */
    static std::vector<CVG_BlockDiagram::FilteredDerivativeWCB> filtered_derivative_wcb_x_, filtered_derivative_wcb_y_, filtered_derivative_wcb_z_;
    static std::vector<std::string> tracker_names_;
    static std::vector<CVG_BlockDiagram::FilteredDerivativeWCB> body_filtered_derivative_wcb_x_, body_filtered_derivative_wcb_y_, body_filtered_derivative_wcb_z_;
    static std::vector<std::string> body_tracker_names_;
    static std::vector<std::string> dead_zone_tracker_names_;
    static std::vector<int> dead_zone_counters_;
    static std::vector<bool> dead_zones_;
    static std::vector<ros::Time> previous_times_;
    static const float MIN_FREQUENCY;
    static const int MIN_COUNTER;

    void init(std::string tracker_name, ros::NodeHandle nh, bool create_mainloop_timer);

    static void VRPN_CALLBACK handle_pose(void *userData, const vrpn_TRACKERCB tracker_pose);

    static void VRPN_CALLBACK handle_twist(void *userData, const vrpn_TRACKERVELCB tracker_twist);

    static void VRPN_CALLBACK handle_accel(void *userData, const vrpn_TRACKERACCCB tracker_accel);
  };

  std::vector<CVG_BlockDiagram::FilteredDerivativeWCB> VrpnTrackerRos::filtered_derivative_wcb_x_;
  std::vector<CVG_BlockDiagram::FilteredDerivativeWCB> VrpnTrackerRos::filtered_derivative_wcb_y_;
  std::vector<CVG_BlockDiagram::FilteredDerivativeWCB> VrpnTrackerRos::filtered_derivative_wcb_z_;
  std::vector<std::string> VrpnTrackerRos::tracker_names_;

  std::vector<CVG_BlockDiagram::FilteredDerivativeWCB> VrpnTrackerRos::body_filtered_derivative_wcb_x_;
  std::vector<CVG_BlockDiagram::FilteredDerivativeWCB> VrpnTrackerRos::body_filtered_derivative_wcb_y_;
  std::vector<CVG_BlockDiagram::FilteredDerivativeWCB> VrpnTrackerRos::body_filtered_derivative_wcb_z_;
  std::vector<std::string> VrpnTrackerRos::body_tracker_names_;

  std::vector<ros::Time> VrpnTrackerRos::previous_times_;
  std::vector<int> VrpnTrackerRos::dead_zone_counters_;
  std::vector<bool> VrpnTrackerRos::dead_zones_;
  std::vector<std::string> VrpnTrackerRos::dead_zone_tracker_names_;

  const float VrpnTrackerRos::MIN_FREQUENCY = 10; // Hz
  const int VrpnTrackerRos::MIN_COUNTER = 5;

  class VrpnClientRos
  {
  public:

    typedef std::shared_ptr<VrpnClientRos> Ptr;
    typedef std::unordered_map<std::string, VrpnTrackerRos::Ptr> TrackerMap;

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
     * Examine vrpn_Connection's senders and create new trackers as necessary.
     */
    void updateTrackers();

  private:
    std::string host_;
    ros::NodeHandle output_nh_;

    /**
     * Underlying VRPN connection object
     */
    ConnectionPtr connection_;

    /**
     * Map of registered trackers, accessible by name
     */
    TrackerMap trackers_;

    ros::Timer refresh_tracker_timer_, mainloop_timer;

  };
}  // namespace vrpn_client_ros

#endif  // VRPN_CLIENT_ROS_VRPN_CLIENT_ROS_H

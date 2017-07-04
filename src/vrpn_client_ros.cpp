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

#include <vector>
#include <unordered_set>

#include <eigen3/Eigen/Dense>
#include <tf/tf.h>


namespace
{
std::unordered_set<std::string> name_blacklist_({"VRPN Control"});
}

namespace vrpn_client_ros
{

VrpnTrackerRos::VrpnTrackerRos(std::string tracker_name, ConnectionPtr connection, ros::NodeHandle nh)
{
    tracker_remote_ = std::make_shared<vrpn_Tracker_Remote>(tracker_name.c_str(), connection.get());
    init(tracker_name, nh, false);
}

VrpnTrackerRos::VrpnTrackerRos(std::string tracker_name, std::string host, ros::NodeHandle nh)
{
    std::string tracker_address;
    tracker_address = tracker_name + "@" + host;
    tracker_remote_ = std::make_shared<vrpn_Tracker_Remote>(tracker_address.c_str());
    init(tracker_name, nh, true);
}

void VrpnTrackerRos::init(std::string tracker_name, ros::NodeHandle nh, bool create_mainloop_timer)
{
    ROS_INFO_STREAM("Creating new tracker " << tracker_name);

    tracker_remote_->register_change_handler(this, &VrpnTrackerRos::handle_pose);
    tracker_remote_->register_change_handler(this, &VrpnTrackerRos::handle_twist);
    tracker_remote_->register_change_handler(this, &VrpnTrackerRos::handle_accel);
    tracker_remote_->shutup = true;

    std::string error;
    if (!ros::names::validate(tracker_name, error))
    {
        ROS_ERROR_STREAM("Invalid tracker name " << tracker_name << ", not creating topics : " << error);
        return;
    }

    this->tracker_name = tracker_name;

    output_nh_ = ros::NodeHandle(nh, tracker_name);

    std::string frame_id;
    nh.param<std::string>("frame_id", frame_id, "world");
    nh.param<bool>("use_server_time", use_server_time_, false);
    nh.param<bool>("broadcast_tf", broadcast_tf_, false);
    nh.param<bool>("process_sensor_id", process_sensor_id_, false);

    pose_msg_.header.frame_id = twist_msg_.header.frame_id = accel_msg_.header.frame_id = transform_stamped_.header.frame_id = frame_id;

    if (create_mainloop_timer)
    {
        double update_frequency;
        nh.param<double>("update_frequency", update_frequency, 100.0);
        mainloop_timer = nh.createTimer(ros::Duration(1 / update_frequency),
                                        boost::bind(&VrpnTrackerRos::mainloop, this));
    }
}

VrpnTrackerRos::~VrpnTrackerRos()
{
    ROS_INFO_STREAM("Destroying tracker " << transform_stamped_.child_frame_id);
    tracker_remote_->unregister_change_handler(this, &VrpnTrackerRos::handle_pose);
    tracker_remote_->unregister_change_handler(this, &VrpnTrackerRos::handle_twist);
    tracker_remote_->unregister_change_handler(this, &VrpnTrackerRos::handle_accel);
}

void VrpnTrackerRos::mainloop()
{
    tracker_remote_->mainloop();
}

void VRPN_CALLBACK VrpnTrackerRos::handle_pose(void *userData, const vrpn_TRACKERCB tracker_pose)
{
    VrpnTrackerRos *tracker = static_cast<VrpnTrackerRos *>(userData);

    ros::Publisher *pose_pub, *body_pose_pub;
    std::size_t sensor_index(0);
    ros::NodeHandle nh = tracker->output_nh_;
    
    if (tracker->process_sensor_id_)
    {
        sensor_index = static_cast<std::size_t>(tracker_pose.sensor);
        nh = ros::NodeHandle(tracker->output_nh_, std::to_string(tracker_pose.sensor));
    }
    
    if (tracker->pose_pubs_.size() <= sensor_index)
    {
        tracker->pose_pubs_.resize(sensor_index + 1);
        tracker->body_pose_pubs_.resize(sensor_index + 1);
    }
    pose_pub = &(tracker->pose_pubs_[sensor_index]);
    body_pose_pub = &(tracker->body_pose_pubs_[sensor_index]);

    if (pose_pub->getTopic().empty())
    {
        //      *pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);
        *pose_pub = nh.advertise<nav_msgs::Odometry>("pose", 1);
    }

    if (body_pose_pub->getTopic().empty())
    {
        *body_pose_pub = nh.advertise<nav_msgs::Odometry>("body_pose", 1);
    }

    if (pose_pub->getNumSubscribers() > 0)
    {
//        if (tracker->use_server_time_)
//        {
//            tracker->pose_msg_.header.stamp.sec = tracker_pose.msg_time.tv_sec;
//            tracker->pose_msg_.header.stamp.nsec = tracker_pose.msg_time.tv_usec * 1000;
//        }
//        else
//        {
//            tracker->pose_msg_.header.stamp = ros::Time::now();
//        }

//        tracker->pose_msg_.pose.position.x = tracker_pose.pos[0];
//        tracker->pose_msg_.pose.position.y = tracker_pose.pos[1];
//        tracker->pose_msg_.pose.position.z = tracker_pose.pos[2];

//        tracker->pose_msg_.pose.orientation.x = tracker_pose.quat[0];
//        tracker->pose_msg_.pose.orientation.y = tracker_pose.quat[1];
//        tracker->pose_msg_.pose.orientation.z = tracker_pose.quat[2];
//        tracker->pose_msg_.pose.orientation.w = tracker_pose.quat[3];

        bool found = false;
        unsigned int index_i = -1;
        // Find tracker name
        for(unsigned int i=0; i < VrpnTrackerRos::tracker_names_.size(); i++){
            if (tracker->tracker_name.compare(VrpnTrackerRos::tracker_names_[i]) == 0){
                found = true;
                index_i = i;
            }
        }

        if (!found){

            // Insert new rigid body
            VrpnTrackerRos::tracker_names_.push_back(tracker->tracker_name);
            CVG_BlockDiagram::FilteredDerivativeWCB x;
            CVG_BlockDiagram::FilteredDerivativeWCB y;
            CVG_BlockDiagram::FilteredDerivativeWCB z;
            VrpnTrackerRos::filtered_derivative_wcb_x_.push_back(x);
            VrpnTrackerRos::filtered_derivative_wcb_y_.push_back(y);
            VrpnTrackerRos::filtered_derivative_wcb_z_.push_back(z);

            // Find tracker name
            for(unsigned int i=0; i < VrpnTrackerRos::tracker_names_.size(); i++){
                if (tracker->tracker_name.compare(VrpnTrackerRos::tracker_names_[i]) == 0){
                    found = true;
                    index_i = i;
                }
            }

            // Init ciruclar buffer
            VrpnTrackerRos::filtered_derivative_wcb_x_[index_i].setTimeParameters( 0.005,0.005,0.200,1.0,100.000);
            VrpnTrackerRos::filtered_derivative_wcb_y_[index_i].setTimeParameters( 0.005,0.005,0.200,1.0,100.000);
            VrpnTrackerRos::filtered_derivative_wcb_z_[index_i].setTimeParameters( 0.005,0.005,0.200,1.0,100.000);

            VrpnTrackerRos::filtered_derivative_wcb_x_[index_i].reset();
            VrpnTrackerRos::filtered_derivative_wcb_y_[index_i].reset();
            VrpnTrackerRos::filtered_derivative_wcb_z_[index_i].reset();

        }



        // Compute speeds from pose ground truth
        ros::Time current_timestamp = ros::Time::now();

        double x_raw_t = tracker_pose.pos[0];
        double y_raw_t = tracker_pose.pos[1];
        double z_raw_t = tracker_pose.pos[2];

        time_t tv_sec; suseconds_t tv_usec;
        {
            tv_sec  = current_timestamp.sec;
            tv_usec = current_timestamp.nsec / 1000.0;
            VrpnTrackerRos::filtered_derivative_wcb_x_[index_i].setInput( x_raw_t, tv_sec, tv_usec);
            VrpnTrackerRos::filtered_derivative_wcb_y_[index_i].setInput( y_raw_t, tv_sec, tv_usec);
            VrpnTrackerRos::filtered_derivative_wcb_z_[index_i].setInput( z_raw_t, tv_sec, tv_usec);
        }

        double x_t, dx_t;
        double y_t, dy_t;
        double z_t, dz_t;
        VrpnTrackerRos::filtered_derivative_wcb_x_[index_i].getOutput( x_t,  dx_t);
        VrpnTrackerRos::filtered_derivative_wcb_y_[index_i].getOutput( y_t,  dy_t);
        VrpnTrackerRos::filtered_derivative_wcb_z_[index_i].getOutput( z_t,  dz_t);

        // Publish velocities
        tracker->pose_and_velocity_msg_.header.stamp = ros::Time::now();
        tracker->pose_and_velocity_msg_.header.frame_id = "speeds_odom";
        tracker->pose_and_velocity_msg_.child_frame_id = "fcu";

        tracker->pose_and_velocity_msg_.pose.pose.position.x = tracker_pose.pos[0];
        tracker->pose_and_velocity_msg_.pose.pose.position.y = tracker_pose.pos[1];
        tracker->pose_and_velocity_msg_.pose.pose.position.z = tracker_pose.pos[2];

        tracker->pose_and_velocity_msg_.pose.pose.orientation.x = tracker_pose.quat[0];
        tracker->pose_and_velocity_msg_.pose.pose.orientation.y = tracker_pose.quat[1];
        tracker->pose_and_velocity_msg_.pose.pose.orientation.z = tracker_pose.quat[2];
        tracker->pose_and_velocity_msg_.pose.pose.orientation.w = tracker_pose.quat[3];

        tracker->pose_and_velocity_msg_.twist.twist.linear.x = dx_t;
        tracker->pose_and_velocity_msg_.twist.twist.linear.y = dy_t;
        tracker->pose_and_velocity_msg_.twist.twist.linear.z = dz_t;

        tracker->pose_and_velocity_msg_.pose.covariance[0]   = 0.0001;
        tracker->pose_and_velocity_msg_.pose.covariance[7]   = 0.0001;
        tracker->pose_and_velocity_msg_.pose.covariance[28]  = 0.0001;
        tracker->pose_and_velocity_msg_.pose.covariance[35]  = 0.0001;
        tracker->pose_and_velocity_msg_.twist.covariance[0]  = 0.0001;
        tracker->pose_and_velocity_msg_.twist.covariance[7]  = 0.0001;

        //      pose_pub->publish(tracker->pose_msg_);
        pose_pub->publish(tracker->pose_and_velocity_msg_);

    }

    if (body_pose_pub->getNumSubscribers() > 0)
    {
//        if (tracker->use_server_time_)
//        {
//            tracker->body_pose_msg_.header.stamp.sec = tracker_pose.msg_time.tv_sec;
//            tracker->pose_msg_.header.stamp.nsec = tracker_pose.msg_time.tv_usec * 1000;
//        }
//        else
//        {
//            tracker->pose_msg_.header.stamp = ros::Time::now();
//        }

//        tracker->pose_msg_.pose.position.x = tracker_pose.pos[0];
//        tracker->pose_msg_.pose.position.y = tracker_pose.pos[1];
//        tracker->pose_msg_.pose.position.z = tracker_pose.pos[2];

//        tracker->pose_msg_.pose.orientation.x = tracker_pose.quat[0];
//        tracker->pose_msg_.pose.orientation.y = tracker_pose.quat[1];
//        tracker->pose_msg_.pose.orientation.z = tracker_pose.quat[2];
//        tracker->pose_msg_.pose.orientation.w = tracker_pose.quat[3];

        bool found = false;
        unsigned int index_i = -1;
        // Find tracker name
        for(unsigned int i=0; i < VrpnTrackerRos::body_tracker_names_.size(); i++){
            if (tracker->tracker_name.compare(VrpnTrackerRos::body_tracker_names_[i]) == 0){
                found = true;
                index_i = i;
            }
        }

        if (!found){

            // Insert new rigid body
            VrpnTrackerRos::body_tracker_names_.push_back(tracker->tracker_name);
            CVG_BlockDiagram::FilteredDerivativeWCB x;
            CVG_BlockDiagram::FilteredDerivativeWCB y;
            CVG_BlockDiagram::FilteredDerivativeWCB z;
            VrpnTrackerRos::body_filtered_derivative_wcb_x_.push_back(x);
            VrpnTrackerRos::body_filtered_derivative_wcb_y_.push_back(y);
            VrpnTrackerRos::body_filtered_derivative_wcb_z_.push_back(z);

            // Find tracker name
            for(unsigned int i=0; i < VrpnTrackerRos::body_tracker_names_.size(); i++){
                if (tracker->tracker_name.compare(VrpnTrackerRos::body_tracker_names_[i]) == 0){
                    found = true;
                    index_i = i;
                }
            }

            // Init ciruclar buffer
            VrpnTrackerRos::body_filtered_derivative_wcb_x_[index_i].setTimeParameters( 0.005,0.005,0.200,1.0,100.000);
            VrpnTrackerRos::body_filtered_derivative_wcb_y_[index_i].setTimeParameters( 0.005,0.005,0.200,1.0,100.000);
            VrpnTrackerRos::body_filtered_derivative_wcb_z_[index_i].setTimeParameters( 0.005,0.005,0.200,1.0,100.000);

            VrpnTrackerRos::body_filtered_derivative_wcb_x_[index_i].reset();
            VrpnTrackerRos::body_filtered_derivative_wcb_y_[index_i].reset();
            VrpnTrackerRos::body_filtered_derivative_wcb_z_[index_i].reset();

        }



        // Compute speeds from pose ground truth
        ros::Time current_timestamp = ros::Time::now();

        double x_raw_t = tracker_pose.pos[0];
        double y_raw_t = tracker_pose.pos[1];
        double z_raw_t = tracker_pose.pos[2];

        time_t tv_sec; suseconds_t tv_usec;
        {
            tv_sec  = current_timestamp.sec;
            tv_usec = current_timestamp.nsec / 1000.0;
            VrpnTrackerRos::body_filtered_derivative_wcb_x_[index_i].setInput( x_raw_t, tv_sec, tv_usec);
            VrpnTrackerRos::body_filtered_derivative_wcb_y_[index_i].setInput( y_raw_t, tv_sec, tv_usec);
            VrpnTrackerRos::body_filtered_derivative_wcb_z_[index_i].setInput( z_raw_t, tv_sec, tv_usec);
        }

        double x_t, dx_t;
        double y_t, dy_t;
        double z_t, dz_t;
        VrpnTrackerRos::body_filtered_derivative_wcb_x_[index_i].getOutput( x_t,  dx_t);
        VrpnTrackerRos::body_filtered_derivative_wcb_y_[index_i].getOutput( y_t,  dy_t);
        VrpnTrackerRos::body_filtered_derivative_wcb_z_[index_i].getOutput( z_t,  dz_t);

        // Converting to Body
        /* Calculating Roll, Pitch, Yaw */
        tf::Quaternion q(tracker_pose.quat[0], tracker_pose.quat[1], tracker_pose.quat[2], tracker_pose.quat[3]);
        tf::Matrix3x3 m(q);

        //convert quaternion to euler angels
        double y, p, r;
        m.getEulerYPR(y, p, r);

        if( y < 0 )
              y = (2*M_PI) + y;

        Eigen::Vector3f BodyFrame;
        Eigen::Vector3f GlobalFrame;
        Eigen::Matrix3f RotationMat;

        GlobalFrame(0) = (+1)*dx_t;
        GlobalFrame(1) = (+1)*dy_t;
        GlobalFrame(2) = 0;

        RotationMat(0,0) = cos(y);
        RotationMat(1,0) = -sin(y);
        RotationMat(2,0) = 0;

        RotationMat(0,1) = sin(y);
        RotationMat(1,1) = cos(y);
        RotationMat(2,1) = 0;

        RotationMat(0,2) = 0;
        RotationMat(1,2) = 0;
        RotationMat(2,2) = 1;

        BodyFrame = RotationMat*GlobalFrame;

        //std::cout << "X velocity: " << BodyFrame(0) << "   Y velocity: " << BodyFrame(1) << std::endl;

        dx_t  = (+1) * BodyFrame(0);
        dy_t  = (+1) * BodyFrame(1);

        // Publish velocities
        tracker->body_pose_and_velocity_msg_.header.stamp = ros::Time::now();
        tracker->body_pose_and_velocity_msg_.header.frame_id = "speeds_odom";
        tracker->body_pose_and_velocity_msg_.child_frame_id = "fcu";

        tracker->body_pose_and_velocity_msg_.pose.pose.position.x = tracker_pose.pos[0];
        tracker->body_pose_and_velocity_msg_.pose.pose.position.y = tracker_pose.pos[1];
        tracker->body_pose_and_velocity_msg_.pose.pose.position.z = tracker_pose.pos[2];

        tracker->body_pose_and_velocity_msg_.pose.pose.orientation.x = tracker_pose.quat[0];
        tracker->body_pose_and_velocity_msg_.pose.pose.orientation.y = tracker_pose.quat[1];
        tracker->body_pose_and_velocity_msg_.pose.pose.orientation.z = tracker_pose.quat[2];
        tracker->body_pose_and_velocity_msg_.pose.pose.orientation.w = tracker_pose.quat[3];

        tracker->body_pose_and_velocity_msg_.twist.twist.linear.x = dx_t;
        tracker->body_pose_and_velocity_msg_.twist.twist.linear.y = dy_t;
        tracker->body_pose_and_velocity_msg_.twist.twist.linear.z = dz_t;

        tracker->body_pose_and_velocity_msg_.pose.covariance[0]   = 0.0001;
        tracker->body_pose_and_velocity_msg_.pose.covariance[7]   = 0.0001;
        tracker->body_pose_and_velocity_msg_.pose.covariance[28]  = 0.0001;
        tracker->body_pose_and_velocity_msg_.pose.covariance[35]  = 0.0001;
        tracker->body_pose_and_velocity_msg_.twist.covariance[0]  = 0.0001;
        tracker->body_pose_and_velocity_msg_.twist.covariance[7]  = 0.0001;

        //      pose_pub->publish(tracker->pose_msg_);
        body_pose_pub->publish(tracker->body_pose_and_velocity_msg_);

    }

    if (tracker->broadcast_tf_)
    {
        static tf2_ros::TransformBroadcaster tf_broadcaster;

        if (tracker->use_server_time_)
        {
            tracker->transform_stamped_.header.stamp.sec = tracker_pose.msg_time.tv_sec;
            tracker->transform_stamped_.header.stamp.nsec = tracker_pose.msg_time.tv_usec * 1000;
        }
        else
        {
            tracker->transform_stamped_.header.stamp = ros::Time::now();
        }

        if (tracker->process_sensor_id_)
        {
            tracker->transform_stamped_.child_frame_id = tracker->tracker_name + "/" + std::to_string(tracker_pose.sensor);
        }
        else
        {
            tracker->transform_stamped_.child_frame_id = tracker->tracker_name;
        }

        tracker->transform_stamped_.transform.translation.x = tracker_pose.pos[0];
        tracker->transform_stamped_.transform.translation.y = tracker_pose.pos[1];
        tracker->transform_stamped_.transform.translation.z = tracker_pose.pos[2];

        tracker->transform_stamped_.transform.rotation.x = tracker_pose.quat[0];
        tracker->transform_stamped_.transform.rotation.y = tracker_pose.quat[1];
        tracker->transform_stamped_.transform.rotation.z = tracker_pose.quat[2];
        tracker->transform_stamped_.transform.rotation.w = tracker_pose.quat[3];

        tf_broadcaster.sendTransform(tracker->transform_stamped_);
    }
}

void VRPN_CALLBACK VrpnTrackerRos::handle_twist(void *userData, const vrpn_TRACKERVELCB tracker_twist)
{
    VrpnTrackerRos *tracker = static_cast<VrpnTrackerRos *>(userData);

    ros::Publisher *twist_pub;
    std::size_t sensor_index(0);
    ros::NodeHandle nh = tracker->output_nh_;
    
    if (tracker->process_sensor_id_)
    {
        sensor_index = static_cast<std::size_t>(tracker_twist.sensor);
        nh = ros::NodeHandle(tracker->output_nh_, std::to_string(tracker_twist.sensor));
    }
    
    if (tracker->twist_pubs_.size() <= sensor_index)
    {
        tracker->twist_pubs_.resize(sensor_index + 1);
    }
    twist_pub = &(tracker->twist_pubs_[sensor_index]);

    if (twist_pub->getTopic().empty())
    {
        *twist_pub = nh.advertise<geometry_msgs::TwistStamped>("twist", 1);
    }

    if (twist_pub->getNumSubscribers() > 0)
    {
        if (tracker->use_server_time_)
        {
            tracker->twist_msg_.header.stamp.sec = tracker_twist.msg_time.tv_sec;
            tracker->twist_msg_.header.stamp.nsec = tracker_twist.msg_time.tv_usec * 1000;
        }
        else
        {
            tracker->twist_msg_.header.stamp = ros::Time::now();
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

        twist_pub->publish(tracker->twist_msg_);
    }
}

void VRPN_CALLBACK VrpnTrackerRos::handle_accel(void *userData, const vrpn_TRACKERACCCB tracker_accel)
{
    VrpnTrackerRos *tracker = static_cast<VrpnTrackerRos *>(userData);

    ros::Publisher *accel_pub;
    std::size_t sensor_index(0);
    ros::NodeHandle nh = tracker->output_nh_;

    if (tracker->process_sensor_id_)
    {
        sensor_index = static_cast<std::size_t>(tracker_accel.sensor);
        nh = ros::NodeHandle(tracker->output_nh_, std::to_string(tracker_accel.sensor));
    }
    
    if (tracker->accel_pubs_.size() <= sensor_index)
    {
        tracker->accel_pubs_.resize(sensor_index + 1);
    }
    accel_pub = &(tracker->accel_pubs_[sensor_index]);

    if (accel_pub->getTopic().empty())
    {
        *accel_pub = nh.advertise<geometry_msgs::TwistStamped>("accel", 1);
    }

    if (accel_pub->getNumSubscribers() > 0)
    {
        if (tracker->use_server_time_)
        {
            tracker->accel_msg_.header.stamp.sec = tracker_accel.msg_time.tv_sec;
            tracker->accel_msg_.header.stamp.nsec = tracker_accel.msg_time.tv_usec * 1000;
        }
        else
        {
            tracker->accel_msg_.header.stamp = ros::Time::now();
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

        accel_pub->publish(tracker->accel_msg_);
    }
}

VrpnClientRos::VrpnClientRos(ros::NodeHandle nh, ros::NodeHandle private_nh)
{
    output_nh_ = private_nh;

    host_ = getHostStringFromParams(private_nh);

    ROS_INFO_STREAM("Connecting to VRPN server at " << host_);
    connection_ = std::shared_ptr<vrpn_Connection>(vrpn_get_connection_by_name(host_.c_str()));
    ROS_INFO("Connection established");

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
    if (private_nh.getParam("trackers", param_tracker_names_))
    {
        for (std::vector<std::string>::iterator it = param_tracker_names_.begin();
             it != param_tracker_names_.end(); ++it)
        {
            trackers_.insert(std::make_pair(*it, std::make_shared<VrpnTrackerRos>(*it, connection_, output_nh_)));
        }
    }
}

std::string VrpnClientRos::getHostStringFromParams(ros::NodeHandle host_nh)
{
    std::stringstream host_stream;
    std::string server;
    int port;

    host_nh.param<std::string>("server", server, "localhost");
    host_stream << server;

    if (host_nh.getParam("port", port))
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
        ROS_WARN("VRPN connection is not 'doing okay'");
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
            ROS_INFO_STREAM("Found new sender: " << connection_->sender_name(i));
            trackers_.insert(std::make_pair(connection_->sender_name(i),
                                            std::make_shared<VrpnTrackerRos>(connection_->sender_name(i), connection_,
                                                                             output_nh_)));
        }
        i++;
    }
}
}  // namespace vrpn_client_ros

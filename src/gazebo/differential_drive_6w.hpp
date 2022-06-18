//=================================================================================================
// Copyright (c) 2011, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================
/*
 * Desc: ROS interface to a Position2d controller for a Differential drive.
 * Author: Daniel Hewlett (adapted from Nathan Koenig)
 */
#pragma once

#include <map>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>

// ROS
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// Custom Callback Queue
#include <ros/advertise_options.h>
#include <ros/callback_queue.h>

// Boost
#include <boost/bind.hpp>
#include <boost/thread.hpp>

namespace gazebo {

    class DiffDrivePlugin6W : public ModelPlugin {

    public:
        DiffDrivePlugin6W();

        ~DiffDrivePlugin6W() override;

    protected:
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

        void Reset() override;

        virtual void Update();

    private:
        void publish_odometry();

        void GetPositionCmd();

        physics::LinkPtr link;
        physics::WorldPtr world;
        physics::JointPtr joints[6];

        float wheelSep{};
        float wheelDiam{};
        float torque{};
        float wheelSpeed[2]{};

        // Simulation time of the last update
        common::Time prevUpdateTime;

        bool enableMotors{};
        float odomPose[3]{};
        float odomVel[3]{};

        // ROS STUFF
        ros::NodeHandle* rosnode_{};
        ros::Publisher pub_;
        ros::Subscriber sub_;
        tf::TransformBroadcaster* transform_broadcaster_{};
        nav_msgs::Odometry odom_;
        std::string tf_prefix_;

        boost::mutex lock;

        std::string namespace_;
        std::string topic_;
        std::string link_name_;

        // Custom Callback Queue
        ros::CallbackQueue queue_;
        boost::thread callback_queue_thread_;

        void QueueThread();

        // DiffDrive stuff
        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

        float x_{};
        float rot_{};
        bool alive_{};

        // Pointer to the update event connection
        event::ConnectionPtr updateConnection;
    };

} // namespace gazebo

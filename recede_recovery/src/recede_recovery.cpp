/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <recede_recovery/recede_recovery.h>
#include <pluginlib/class_list_macros.h>

//register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(recede_recovery::RecedeRecovery, nav_core::RecoveryBehavior)

namespace recede_recovery {
RecedeRecovery::RecedeRecovery(): global_costmap_(NULL), local_costmap_(NULL),
  tf_(NULL), initialized_(false), world_model_(NULL) {}

void RecedeRecovery::initialize(std::string name, tf::TransformListener* tf,
    costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap){
  if(!initialized_){
    name_ = name;
    tf_ = tf;
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;

    //get some parameters from the parameter server
    ros::NodeHandle private_nh("~/" + name_);
    // ros::NodeHandle blp_nh("~/TrajectoryPlannerROS");
    ros::NodeHandle blp_nh("~/DWAPlannerROS");

    //we'll simulate every degree by default
    private_nh.param("sim_granularity", sim_granularity_, 0.017);
    private_nh.param("frequency", frequency_, 20.0);

    blp_nh.param("acc_lim_th", acc_lim_th_, 3.2);
    blp_nh.param("max_rotational_vel", max_rotational_vel_, 1.0);
    blp_nh.param("min_in_place_rotational_vel", min_rotational_vel_, 0.4);
    blp_nh.param("yaw_goal_tolerance", tolerance_, 0.10);
    blp_nh.param("max_vel_x", max_vel_x_,  0.5);
    blp_nh.param("min_vel_x", min_vel_x_, -0.5);
    blp_nh.param("max_vel_y", max_vel_y_,  0.2);
    blp_nh.param("min_vel_y", min_vel_y_, -0.2);

    ROS_WARN("Recede Recovery initialized !!!");
    ROS_WARN("Max vel x : %lf", max_vel_x_);
    ROS_WARN("Min vel x : %lf", min_vel_x_);
    ROS_WARN("Max vel y : %lf", max_vel_y_);
    ROS_WARN("Min vel y : %lf", min_vel_y_);

    world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());

    initialized_ = true;
  }
  else{
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

RecedeRecovery::~RecedeRecovery(){
  delete world_model_;
}

void RecedeRecovery::runBehavior(){
  if(!initialized_){
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  if(global_costmap_ == NULL || local_costmap_ == NULL){
    ROS_ERROR("The costmaps passed to the RecedeRecovery object cannot be NULL. Doing nothing.");
    return;
  }
  ROS_WARN("Recede recovery behavior started.");

  ros::Rate r(frequency_);
  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  // nav_msgs::Path visited_path_;
  // int goal_itr = visited_path_size() - 1;
  tf::Stamped<tf::Pose> global_pose;

  while(n.ok()){
    local_costmap_->getRobotPose(global_pose);

    double norm_angle = angles::normalize_angle(tf::getYaw(global_pose.getRotation()));
    // double vel = sqrt(2 * acc_lim_th_ * dist_left);
    // vel = std::min(std::max(vel, min_rotational_vel_), max_rotational_vel_);

    geometry_msgs::Twist cmd_vel;
    // cmd_vel.linear.x = 0.0;
    // cmd_vel.linear.y = 0.0;
    // cmd_vel.angular.z = vel;

    cmd_vel.linear.x = min_vel_x_;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;

    vel_pub.publish(cmd_vel);

    // recovery done if footprint cost is not illegal
    double x = global_pose.getOrigin().x();
    double y = global_pose.getOrigin().y();
    double theta = tf::getYaw(global_pose.getRotation());

    ROS_WARN("[Recede Recovery] Robot pose");
    ROS_WARN("x : %lf  y : %lf  yaw : %lf", x, y, theta);

    double footprint_cost = world_model_->footprintCost(x, y, theta, local_costmap_->getRobotFootprint(), 0.0, 0.0);
    if(footprint_cost > 0.0){
      return;
    }

    r.sleep();
  }
}
};

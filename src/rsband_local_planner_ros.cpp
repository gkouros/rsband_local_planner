/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, George Kouros.
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
*   * Neither the name of the copyright holder nor the names of its
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
* Author:  George Kouros
*********************************************************************/

#include "rsband_local_planner/rsband_local_planner_ros.h"

#include <base_local_planner/goal_functions.h>
#include <pluginlib/class_list_macros.h>
#include <angles/angles.h>

PLUGINLIB_EXPORT_CLASS(rsband_local_planner::RSBandPlannerROS, nav_core::BaseLocalPlanner);

namespace rsband_local_planner
{

  RSBandPlannerROS::RSBandPlannerROS() : initialized_(false)
  {
  }


  RSBandPlannerROS::~RSBandPlannerROS()
  {
  }


  void RSBandPlannerROS::initialize(std::string name,
    tf2_ros::Buffer* tfBuffer, costmap_2d::Costmap2DROS* costmapROS)
  {
    if (initialized_)
    {
      ROS_WARN("Planner already initialized. Should not be called more than "
        "once");
      return;
    }

    ros::NodeHandle pnh("~/" + name);

    // store tflistener and costmapROS
    tfBuffer_ = tfBuffer;
    tfListener_ = new tf2_ros::TransformListener(*tfBuffer);
    costmapROS_ = costmapROS;

    // initialize plan publishers
    globalPlanPub_ = pnh.advertise<nav_msgs::Path>("global_plan", 1);
    localPlanPub_ = pnh.advertise<nav_msgs::Path>("local_plan", 1);
    ebandPlanPub_ = pnh.advertise<nav_msgs::Path>("eband_plan", 1);
    rsPlanPub_ = pnh.advertise<nav_msgs::Path>("reeds_shepp_plan", 1);

    emergencyPoses_.resize(3);

    // create new eband planner
    ebandPlanner_ = boost::shared_ptr<eband_local_planner::EBandPlanner>(
      new eband_local_planner::EBandPlanner(name, costmapROS_));

    // create new reeds shepp planner
    rsPlanner_ = boost::shared_ptr<ReedsSheppPlanner>(
      new ReedsSheppPlanner(name, costmapROS_, tfBuffer_));
    ebandToRSStrategy_ = static_cast<EbandToRSStrategy>(0);

    // create new path tracking controller
    ptc_ = boost::shared_ptr<FuzzyPTC>(new FuzzyPTC(name));

    // create and initialize dynamic reconfigure for rsband
    ros::NodeHandle rsband_pnh("~rsband");
    rsband_drs_.reset(new dynamic_reconfigure::Server<rsband_local_planner::RSBandPlannerConfig>(rsband_pnh));
    dynamic_reconfigure::Server<rsband_local_planner::RSBandPlannerConfig>::CallbackType rsband_drs_cb =
      boost::bind(&RSBandPlannerROS::rsbandReconfigureCallback, this, _1, _2);
    rsband_drs_->setCallback(rsband_drs_cb);

    // create and initialize dynamic reconfigure for eband
    ros::NodeHandle eband_pnh("~eband");
    eband_drs_.reset(new dynamic_reconfigure::Server<eband_local_planner::EBandPlannerConfig>(eband_pnh));
    dynamic_reconfigure::Server<eband_local_planner::EBandPlannerConfig>::CallbackType eband_drs_cb =
      boost::bind(&RSBandPlannerROS::ebandReconfigureCallback, this, _1, _2);
    eband_drs_->setCallback(eband_drs_cb);

    // set initilized
    initialized_ = true;

    ROS_DEBUG("Local Planner Plugin Initialized!");
  }

  void RSBandPlannerROS::rsbandReconfigureCallback(RSBandPlannerConfig& config, uint32_t level)
  {
    xyGoalTolerance_ = config.xy_goal_tolerance;
    yawGoalTolerance_ = config.yaw_goal_tolerance;
    updateSubGoalDistThreshold_ = config.update_sub_goal_dist_threshold;
    ebandToRSStrategy_ =
      static_cast<EbandToRSStrategy>(config.eband_to_rs_strategy);
    emergencyPlanning_ = config.emergency_planning;

    if (rsPlanner_)
      rsPlanner_->reconfigure(config);
    else
      ROS_ERROR("Reconfigure CB called before reeds shepp planner "
        "initialization");

    if (ptc_)
      ptc_->reconfigure(config);
    else
      ROS_ERROR("RSBand reconfigure CB called before path tracking controller "
        "initialization!");
  }

  void RSBandPlannerROS::ebandReconfigureCallback(eband_local_planner::EBandPlannerConfig& config, uint32_t level)
  {
    if (ebandPlanner_)
      ebandPlanner_->reconfigure(config);
    else
      ROS_ERROR("EBand reconfigure CB called before eband planner "
        "initialization");
  }


  bool RSBandPlannerROS::setPlan(
    const std::vector<geometry_msgs::PoseStamped>& globalPlan)
  {
    if (!initialized_)
    {
      ROS_ERROR("Planner must be initialized before setPlan is called!");
      return false;
    }

    globalPlan_ = globalPlan;


    std::vector<int> planStartEndCounters(2, globalPlan_.size());

    if (!eband_local_planner::transformGlobalPlan(*tfBuffer_, globalPlan_,
        *costmapROS_, costmapROS_->getGlobalFrameID(), transformedPlan_,
        planStartEndCounters))
    {
      ROS_WARN("Could not transform global plan to the local frame");
      return false;
    }

    if (transformedPlan_.empty())
    {
      ROS_WARN("Transformed Plan is empty.");
      return false;
    }

    if(!ebandPlanner_->setPlan(transformedPlan_))
    {
      costmapROS_->resetLayers();
      if(!ebandPlanner_->setPlan(transformedPlan_))
      {
        ROS_ERROR("Setting plan to Elastic Band failed!");
        return false;
      }
    }

    planStartEndCounters_ = planStartEndCounters;

    if (!ebandPlanner_->optimizeBand())
      ROS_WARN("Optimization of eband failed!");

    return true;
  }


  bool RSBandPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd)
  {
    if (!initialized_)
    {
      ROS_ERROR("Planner must be initialized before computeVelocityCommands "
        "is called!");
      return false;
    }

    std::vector<geometry_msgs::PoseStamped> ebandPlan, rsPlan, localPlan;

    if (isGoalReached())
    {
      cmd.linear.x = 0.0;
      cmd.linear.y = 0.0;
      cmd.angular.z = 0.0;
    }
    else
    {
      if (!updateEBand())
      {
        ROS_ERROR("Failed to update eband!");
        return false;
      }

      if (!ebandPlanner_->getPlan(ebandPlan)
        || ebandPlan.empty())
      {
        ROS_ERROR("Failed to get eband planner plan!");
        return false;
      }

      // interpolate orientations of eband plan
      interpolateOrientations(ebandPlan);

      // use reeds shepp planner to connect eband waypoints using RS paths
      // select between the available eband to reeds shepp conversion strategies
      int failIdx;
      switch (ebandToRSStrategy_)
      {
        case startToEnd:
          failIdx = ebandPlan.size() * rsPlanner_->planPath(
              ebandPlan.front(), ebandPlan.back(), rsPlan);
          break;
        case startToNext:
        {
          // TODO: move below segment to reeds_shepp_planner new function
          int next = 0;
          double dist = 0.0;
          while (dist < updateSubGoalDistThreshold_
            && next < ebandPlan.size()-1)
          {
            next++;
            dist += hypot(
              ebandPlan[next-1].pose.position.x
              - ebandPlan[next].pose.position.x,
              ebandPlan[next-1].pose.position.y
              - ebandPlan[next].pose.position.y);
          }
          failIdx = rsPlanner_->planPath(ebandPlan.front(), ebandPlan[next],
            rsPlan);
          break;
        }
        case pointToPoint:
          failIdx = rsPlanner_->planPathUntilFailure(ebandPlan, rsPlan);
          break;
        case skipFailures:
          failIdx = rsPlanner_->planPathSkipFailures(ebandPlan, rsPlan);
          break;
        case startToRecedingEnd:
          failIdx = rsPlanner_->planRecedingPath(ebandPlan, rsPlan);
          break;
        default:  // invalid strategy
          ROS_ERROR("Invalid eband_to_rs_strategy!");
          exit(EXIT_FAILURE);
      }

      double dyaw = fabs(angles::shortest_angular_distance(
        tf::getYaw(ebandPlan[0].pose.orientation),
        tf::getYaw(ebandPlan[1].pose.orientation)));

      // if reeds shepp planning failed or there is orientation error > π/4
      // attempt emergency planning to reach target orientation (if enabled)
      if (emergencyPlanning_ && (failIdx == 0 || dyaw > M_PI/2))
      {
        ROS_DEBUG("Failed to get reeds shepp plan. Attempting "
          "emergency planning...");

        // enable emergency mode if not already enable
        if (!emergencyMode_)
        {
          ROS_DEBUG("Commencing Emergency Mode.");
          emergencyMode_ = true;
          // add initial robot pose as emergency goal pose but with desired
          // orientation
          emergencyPoses_[1] = ebandPlan[0];
          emergencyPoses_[1].pose.orientation =
            ebandPlan[std::min<int>(1,ebandPlan.size()-1)].pose.orientation;
          emergencyPoses_[2] = ebandPlan[0];
        }

        emergencyPoses_[0] = ebandPlan[0];


        if (emergencyPlan(emergencyPoses_, rsPlan))
        {
          ROS_DEBUG("Emergency Planning succeeded!");
        }
        else
        {
          ROS_DEBUG("Emergency Planning failed!");
          emergencyMode_ = false;
          return false;
        }
      }
      else if (!failIdx)
      {
        ROS_DEBUG("Failed to convert eband to rsband plan!");
        return false;
      }
      else
        emergencyMode_ = false;

      // set reeds shepp plan as local plan
      localPlan = rsPlan;

      // publish plans
      base_local_planner::publishPlan(globalPlan_, globalPlanPub_);
      base_local_planner::publishPlan(localPlan, localPlanPub_);
      base_local_planner::publishPlan(ebandPlan, ebandPlanPub_);
      base_local_planner::publishPlan(rsPlan, rsPlanPub_);

      // compute velocity command
      if (!ptc_->computeVelocityCommands(localPlan, cmd))
      {
        ROS_ERROR("Path tracking controller failed to produce command");
        return false;
      }
    }

    return true;
  }


  bool RSBandPlannerROS::isGoalReached()
  {
    if (!initialized_)
    {
      ROS_ERROR("Planner must be initialized before isGoalReached is called!");
      return false;
    }

    geometry_msgs::PoseStamped robotPose;
    if (!costmapROS_->getRobotPose(robotPose))
    {
      ROS_ERROR("Could not get robot pose!");
      return false;
    }

    geometry_msgs::PoseStamped goal = globalPlan_.back();

    double dist = base_local_planner::getGoalPositionDistance(
      robotPose, goal.pose.position.x, goal.pose.position.y);
    double yawDiff = base_local_planner::getGoalOrientationAngleDifference(
      robotPose, tf::getYaw(goal.pose.orientation));

    if (dist < xyGoalTolerance_ && fabs(yawDiff) < yawGoalTolerance_)
    {
      ROS_INFO("Goal Reached!");
      return true;
    }

    return false;
  }


  bool RSBandPlannerROS::updateEBand()
  {
    if (!initialized_)
    {
      ROS_WARN("Planner must be initialized before updateEBand is called!");
      return false;
    }

    // get robot pose
    geometry_msgs::PoseStamped robotPose;
    if (!costmapROS_->getRobotPose(robotPose))
    {
      ROS_ERROR("Could not get robot pose!");
      return false;
    }

    std::vector<geometry_msgs::PoseStamped> tmpPlan(1, robotPose);

    // add robot pose to front of eband
    if (!ebandPlanner_->addFrames(tmpPlan, eband_local_planner::add_front))
    {
      ROS_WARN("Could not connect current robot pose to existing eband!");
      return false;
    }

    // add new frames that have entered the local costmap window and remove the
    // ones that have left it

    std::vector<int> planStartEndCounters = planStartEndCounters_;
    if (!eband_local_planner::transformGlobalPlan(*tfBuffer_, globalPlan_,
        *costmapROS_, costmapROS_->getGlobalFrameID(), transformedPlan_,
        planStartEndCounters))
    {
      ROS_WARN("Failed to transform the global plan to the local frame!");
      return false;
    }

    if (transformedPlan_.empty())
    {
      ROS_WARN("Transformed plan is empty!");
      return false;
    }

    std::vector<geometry_msgs::PoseStamped> planToAppend;

    if (planStartEndCounters_[1] > planStartEndCounters[1])
    {
      if (planStartEndCounters_[1] > planStartEndCounters[0])
      {
        planToAppend = transformedPlan_;
      }
      else
      {
        int numOfDiscardedFrames =
          planStartEndCounters[0] - planStartEndCounters_[1];
        planToAppend.assign(transformedPlan_.begin() + numOfDiscardedFrames + 1,
          transformedPlan_.end());
      }

      if (ebandPlanner_->addFrames(planToAppend, eband_local_planner::add_back))
        planStartEndCounters_ = planStartEndCounters;
      else
      {
        ROS_WARN("Failed to add frames to existing band");
        return false;
      }
    }

    if (!ebandPlanner_->optimizeBand())
    {
      ROS_WARN("Failed to optimize eband!");
      return false;
    }

    return true;
  }


  bool RSBandPlannerROS::emergencyPlan(
    std::vector<geometry_msgs::PoseStamped>& ebandPlan,
    std::vector<geometry_msgs::PoseStamped>& emergencyPlan)
  {
    std::vector<geometry_msgs::PoseStamped> tmpPlan = ebandPlan;

    double dyawOrig = angles::shortest_angular_distance(
      tf::getYaw(tmpPlan[2].pose.orientation),
      tf::getYaw(tmpPlan[1].pose.orientation));
    double dyaw = angles::shortest_angular_distance(
      tf::getYaw(tmpPlan[0].pose.orientation),
      tf::getYaw(tmpPlan[1].pose.orientation));

    int factor = floor(fabs(dyawOrig - dyaw) * 180.0 / M_PI / 45.0);

    double step = (factor+1) * 45.0 * M_PI / 180.0;

    tmpPlan[1].pose.orientation = tf::createQuaternionMsgFromYaw(
        tf::getYaw(tmpPlan[2].pose.orientation) + (step + 0.5) * (dyaw > 0 ? 1 : -1));

    if (rsPlanner_->planPath(tmpPlan[0], tmpPlan[1], emergencyPlan))
      return true;

    return false;
  }

  void RSBandPlannerROS::interpolateOrientations(
    std::vector<geometry_msgs::PoseStamped>& plan)
  {
    for (unsigned int i = 1; i < plan.size() - 1; i++)
    {
      double dx, dy, yaw;
      dx = plan[i+1].pose.position.x - plan[i].pose.position.x;
      dy = plan[i+1].pose.position.y - plan[i].pose.position.y;
      yaw = atan2(dy, dx);
      plan[i].pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
      plan[i].header.stamp = plan.front().header.stamp;
    }
    plan.back().header.stamp = plan.front().header.stamp;

    if ((!plan.back().pose.orientation.z && !plan.back().pose.orientation.w)
        || tf::getYaw(plan.back().pose.orientation) == 0.0)
      plan.back().pose.orientation = plan[plan.size()-2].pose.orientation;
  }

}  // namespace rsband_local_planner

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
*   * Neither the name of the the copyright holder nor the names of its
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

#ifndef RSBAND_LOCAL_PLANNER_REEDS_SHEPP_PLANNER_H
#define RSBAND_LOCAL_PLANNER_REEDS_SHEPP_PLANNER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/costmap_model.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

#include "rsband_local_planner/RSBandPlannerConfig.h"

namespace rsband_local_planner
{

  class ReedsSheppPlanner
  {
    public:

      ReedsSheppPlanner(
        std::string name,
        costmap_2d::Costmap2DROS* costmapROS,
        tf::TransformListener* tfListener);

      ~ReedsSheppPlanner();

      void initialize(
        std::string name,
        costmap_2d::Costmap2DROS* costmapROS,
        tf::TransformListener* tfListener);

      void reconfigure(RSBandPlannerConfig& config);

      bool planPath(
        const geometry_msgs::PoseStamped& startPose,
        const geometry_msgs::PoseStamped& goalPose,
        std::vector<geometry_msgs::PoseStamped>& pathPoses);

      bool planPath(
        const std::vector<geometry_msgs::PoseStamped>& path,
        std::vector<geometry_msgs::PoseStamped>& newPath);

      bool planRecedingPath(
        const std::vector<geometry_msgs::PoseStamped>& path,
        std::vector<geometry_msgs::PoseStamped>& newPath);

      double getMinTurningRadius() {return minTurningRadius_;}
      double getMaxPlanningDuration() {return maxPlanningDuration_;}
      double getBX() {return bx_;}
      double getBY() {return by_;}
      void setMinTurningRadius(double rho) {minTurningRadius_ = rho;}
      void setMaxPlanningDuration(double tmax) {maxPlanningDuration_ = tmax;}
      void setBoundaries(double bx, double by);

    private:

      void transform(const geometry_msgs::PoseStamped& poseIn,
        geometry_msgs::PoseStamped& poseOut, std::string newFrameID);

      void transform(const tf::Stamped<tf::Pose>& tfIn,
        tf::Stamped<tf::Pose>& tfOut, std::string newFrameID);

      void state2pose(
        const ompl::base::State* state, geometry_msgs::PoseStamped& pose);

      void pose2state(
        const geometry_msgs::PoseStamped& pose, ompl::base::State* state);

      bool isStateValid(
        const ompl::base::SpaceInformation* si, const ompl::base::State *state);

    private:

      ompl::base::StateSpacePtr reedsSheppStateSpace_;
      ompl::geometric::SimpleSetupPtr simpleSetup_;
      ompl::base::RealVectorBounds bounds_;

      costmap_2d::Costmap2D* costmap_;
      costmap_2d::Costmap2DROS* costmapROS_;
      base_local_planner::CostmapModel* costmapModel_;
      std::vector<geometry_msgs::Point> footprint_;

      tf::TransformListener* tfListener_;

      std::string robotFrame_;
      std::string globalFrame_;

      ros::Time stamp_;

      double minTurningRadius_;
      double maxPlanningDuration_;
      int interpolationNumPoses_;
      int skipPoses_;
      int validStateMaxCost_;
      bool allowUnknown_;
      bool displayPlannerOutput_;

      double bx_;
      double by_;

      bool initialized_;
  };

}  // namespace rsband_local_planner

#endif  // RSBAND_LOCAL_PLANNER_REEDS_SHEPP_PLANNER_H

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

#ifndef RSBAND_LOCAL_PLANNER_RSBAND_LOCAL_PLANNER_ROS_H
#define RSBAND_LOCAL_PLANNER_RSBAND_LOCAL_PLANNER_ROS_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>

#include <nav_core/base_local_planner.h>
#include "eband_local_planner/eband_local_planner.h"
#include "rsband_local_planner/reeds_shepp_planner.h"
#include "rsband_local_planner/car_like_fuzzy_ptc.h"

#include <dynamic_reconfigure/server.h>
#include "rsband_local_planner/RSBandPlannerConfig.h"

#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>

namespace rsband_local_planner
{

  class RSBandPlannerROS : public nav_core::BaseLocalPlanner
  {
    public:

      RSBandPlannerROS();

      ~RSBandPlannerROS();

      void initialize(std::string name, tf::TransformListener* tf,
        costmap_2d::Costmap2DROS* costmapROS);

      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& globalPlan);

      bool computeVelocityCommands(geometry_msgs::Twist& cmd);

      bool isGoalReached();

    private:

      void interpolateOrientations(
        std::vector<geometry_msgs::PoseStamped>& plan);

      bool updateEBand();

      void reconfigureCallback(RSBandPlannerConfig& config, uint32_t level);

      typedef dynamic_reconfigure::Server<
        rsband_local_planner::RSBandPlannerConfig> drs;
      boost::shared_ptr<drs> drs_;

      tf::TransformListener* tfListener_;
      costmap_2d::Costmap2DROS* costmapROS_;


      boost::shared_ptr<eband_local_planner::EBandPlanner> ebandPlanner_;
      boost::shared_ptr<ReedsSheppPlanner> rsbandPlanner_;
      boost::shared_ptr<CarLikeFuzzyPTC> ptc_;

      // goal tolerances
      double xyGoalTolerance_;
      double yawGoalTolerance_;

      // reeds shepp strategy
      unsigned int eband2RSStrategy_;

      // plan publishers
      ros::Publisher globalPlanPub_;
      ros::Publisher localPlanPub_;
      ros::Publisher ebandPlanPub_;
      ros::Publisher rsbandPlanPub_;

      // global plan container
      std::vector<geometry_msgs::PoseStamped> globalPlan_;
      std::vector<geometry_msgs::PoseStamped> transformedPlan_;

      std::vector<int> planStartEndCounters_;

      bool initialized_;
  };

}  // namespace rsband_local_planner

#endif  // RSBAND_LOCAL_PLANNER_RSBAND_LOCAL_PLANNER_ROS_H

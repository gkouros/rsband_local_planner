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

#include "rsband_local_planner/reeds_shepp_planner.h"
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <boost/foreach.hpp>

namespace rsband_local_planner
{
  ReedsSheppPlanner::ReedsSheppPlanner(
    std::string name,
    costmap_2d::Costmap2DROS* costmapROS,
    tf2_ros::Buffer* tfBuffer)
    :
      reedsSheppStateSpace_(new ompl::base::ReedsSheppStateSpace),
      simpleSetup_(new ompl::geometric::SimpleSetup(reedsSheppStateSpace_)),
      costmapROS_(), tfBuffer_(), bx_(10), by_(10), bounds_(2),
      initialized_(false)
  {
    initialize(name, costmapROS, tfBuffer);
  }


  ReedsSheppPlanner::~ReedsSheppPlanner()
  {
    delete costmapModel_;
  }


  void ReedsSheppPlanner::initialize(
    std::string name,
    costmap_2d::Costmap2DROS* costmapROS,
    tf2_ros::Buffer* tfBuffer)
  {
    costmapROS_ = costmapROS;
    tfBuffer_ = tfBuffer;

    ros::NodeHandle pnh("~/" + name);
    pnh.param("min_turning_radius", minTurningRadius_, 1.0);
    pnh.param("max_planning_duration", maxPlanningDuration_, 0.2);
    pnh.param<int>("valid_state_max_cost", validStateMaxCost_, 252);
    pnh.param<int>("interpolation_num_poses", interpolationNumPoses_, 20);
    pnh.param<bool>("allow_unknown", allowUnknown_, false);
    pnh.param<bool>("robot_state_valid", robotStateValid_, false);
    pnh.param<bool>("display_planner_output", displayPlannerOutput_, false);

    if (costmapROS_)
    {
      costmap_ = costmapROS_->getCostmap();
      costmapModel_ = new base_local_planner::CostmapModel(*costmap_);
      footprint_ = costmapROS_->getRobotFootprint();

      if (!tfBuffer_)
      {
        ROS_FATAL("No tf listener provided.");
        exit(EXIT_FAILURE);
      }

      robotFrame_ = costmapROS_->getBaseFrameID();
      globalFrame_ = costmapROS_->getGlobalFrameID();

      // initialize the state space boundary
      setBoundaries(costmap_->getSizeInMetersX() + 0.02,
        costmap_->getSizeInMetersY() + 0.02);
    }
    else
    {
      ROS_WARN("No costmap provided. Collision checking unavailable. "
        "Same robot_frame and global_frame assumed.");
      robotFrame_ = globalFrame_ = "base_footprint";

      setBoundaries(10.0, 10.0);
    }

    initialized_ = true;
  }

  void ReedsSheppPlanner::reconfigure(
    rsband_local_planner::RSBandPlannerConfig& config)
  {
    maxPlanningDuration_ = config.max_planning_duration;
    interpolationNumPoses_ = config.interpolation_num_poses;
    robotStateValid_ = config.robot_state_valid;
    stateCheckingMode_ =
      static_cast<StateCheckingMode>(config.state_checking_mode);
    validStateMaxCost_ = config.valid_state_max_cost;
    allowUnknown_ = config.allow_unknown;
    displayPlannerOutput_ = config.display_planner_output;

    RearSteeringMode rearSteeringMode = static_cast<RearSteeringMode>(
      config.rear_steering_mode);
    switch (rearSteeringMode)
    {
      case none:
      case crab:
        minTurningRadius_ = config.wheelbase / tan(config.max_steering_angle);
        break;
      case counter:
      case hybrid:
        minTurningRadius_ =
          config.wheelbase / 2 / tan(config.max_steering_angle);
        break;
      default:
        ROS_ERROR("Invalid rear steering mode:[%d]. Exiting...",
          rearSteeringMode);
        exit(EXIT_FAILURE);
    }
  }

  void ReedsSheppPlanner::setBoundaries(const double bx, const double by)
  {
    bx_ = bx;
    by_ = by;
    bounds_.low[0] = -by_ / 2;
    bounds_.low[1] = -bx_ / 2;
    bounds_.high[0] = by_ / 2;
    bounds_.high[1] = bx_ / 2;
    reedsSheppStateSpace_->as<ompl::base::SE2StateSpace>()->setBounds(bounds_);
  }

  void ReedsSheppPlanner::transform(const geometry_msgs::PoseStamped& poseIn,
    geometry_msgs::PoseStamped& poseOut, std::string targetFrameID)
  {
    if (!tfBuffer_)
      return;

    geometry_msgs::TransformStamped transform = tfBuffer_->lookupTransform(targetFrameID, poseIn.header.frame_id, stamp_,
      ros::Duration(1.0));

    tf2::doTransform(poseIn, poseOut, transform);
  }

  void ReedsSheppPlanner::state2pose(
    const ompl::base::State* state, geometry_msgs::PoseStamped& pose)
  {
    const ompl::base::SE2StateSpace::StateType *s =
      state->as<ompl::base::SE2StateSpace::StateType>();
    pose.pose.position.x = s->getX();
    pose.pose.position.y = s->getY();
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(s->getYaw());
    pose.header.frame_id = robotFrame_;
    pose.header.stamp = stamp_;
  }


  void ReedsSheppPlanner::pose2state(
    const geometry_msgs::PoseStamped& pose, ompl::base::State* state)
  {
    ompl::base::SE2StateSpace::StateType *s =
      state->as<ompl::base::SE2StateSpace::StateType>();
    s->setX(pose.pose.position.x);
    s->setY(pose.pose.position.y);
    s->setYaw(tf::getYaw(pose.pose.orientation));
  }


  bool ReedsSheppPlanner::isStateValid(
    const ompl::base::SpaceInformation* si, const ompl::base::State *state)
  {
    // check if state is inside boundary
    if (!si->satisfiesBounds(state))
      return false;

    if (!costmapROS_ || !tfBuffer_)
      return true;

    const ompl::base::SE2StateSpace::StateType *s =
      state->as<ompl::base::SE2StateSpace::StateType>();

    if (robotStateValid_)
    {
      // robot pose is always valid
      if (fabs(s->getX()) < 1e-3 && fabs(s->getY()) < 1e-3
          && fabs(s->getYaw()) < 0.1)
        return true;
    }

    geometry_msgs::PoseStamped statePose;
    state2pose(s, statePose);

    transform(statePose, statePose, globalFrame_);

    // calculate cost of robot center or footprint
    int cost;
    switch (stateCheckingMode_)
    {
      case center: // state checking of center of robot footprint
        int mx, my;
        costmap_->worldToMapEnforceBounds(statePose.pose.position.x,
          statePose.pose.position.y, mx, my);
        cost = costmap_->getCost(mx, my);
        break;
      case footprint: // state checking of footprint
        cost = costmapModel_->footprintCost(
          statePose.pose.position.x, statePose.pose.position.y,
          tf::getYaw(statePose.pose.orientation), footprint_);
    }

    // check if state is valid
    if (cost > validStateMaxCost_)
      return false;

    if (cost == -1)
      return allowUnknown_;

    return true;
  }


  bool ReedsSheppPlanner::planPath(
    const geometry_msgs::PoseStamped& startPose,
    const geometry_msgs::PoseStamped& goalPose,
    std::vector<geometry_msgs::PoseStamped>& path)
  {
    if (!initialized_)
    {
      ROS_ERROR("Planner not initialized!");
      return false;
    }

    // disable planner console output
    if (!displayPlannerOutput_)
    {
      std::cout.setstate(std::ios_base::failbit);
      std::cerr.setstate(std::ios_base::failbit);
    }

    // create start and goal states
    ompl::base::ScopedState<> start(reedsSheppStateSpace_);
    ompl::base::ScopedState<> goal(reedsSheppStateSpace_);

    // initialize state valididy checker
    ompl::base::SpaceInformationPtr si(simpleSetup_->getSpaceInformation());
    simpleSetup_->setStateValidityChecker(
      boost::bind(&ReedsSheppPlanner::isStateValid, this, si.get(), _1));
    si->setStateValidityCheckingResolution(costmap_->getResolution());

    stamp_ = startPose.header.stamp;

    // transform the start and goal poses to the robot/local reference frame
    geometry_msgs::PoseStamped localStartPose, localGoalPose;
    if (tfBuffer_)
    {
      transform(startPose, localStartPose, robotFrame_);
      transform(goalPose, localGoalPose, robotFrame_);
    }
    else
    {
      localStartPose = startPose;
      localGoalPose = goalPose;
    }

    // convert start and goal poses to ompl base states
    pose2state(localStartPose, start());
    pose2state(localGoalPose, goal());

    // clear all planning data
    simpleSetup_->clear();
    // set new start and goal states
    simpleSetup_->setStartAndGoalStates(start, goal);

    simpleSetup_->solve(maxPlanningDuration_);

    if (!simpleSetup_->haveExactSolutionPath())
      return false;
    else
      ROS_DEBUG("[reeds_shepp_planner] Valid plan found");

    // simplify solution
    simpleSetup_->simplifySolution();

    // get solution path
    ompl::geometric::PathGeometric omplPath = simpleSetup_->getSolutionPath();
    // interpolate between poses
    omplPath.interpolate(interpolationNumPoses_);

    if (omplPath.getStateCount() > interpolationNumPoses_)
      return false;


    // resize path
    path.resize(omplPath.getStateCount());

    // convert each state to a pose and store it in path vector
    for (unsigned int i = 0; i < omplPath.getStateCount(); i++)
    {
      const ompl::base::State* state = omplPath.getState(i);
      state2pose(state, path[i]);
      path[i].header.frame_id = robotFrame_;
      path[i].header.stamp = ros::Time::now();
    }

    // enable console output
    std::cout.clear();
    std::cerr.clear();

    return true;
  }


  int ReedsSheppPlanner::planPathUntilFailure(
    const std::vector<geometry_msgs::PoseStamped>& path,
    std::vector<geometry_msgs::PoseStamped>& newPath)
  {
    newPath.clear();

    for (unsigned int i = 0; i < path.size() - 1; i++)
    {
      std::vector<geometry_msgs::PoseStamped> tmpPath;

      if (!planPath(path[i], path[i+1], tmpPath))
        return i;

      newPath.insert(newPath.end(), tmpPath.begin(), tmpPath.end());
    }

    return path.size()-1;
  }

  int ReedsSheppPlanner::planPathSkipFailures(
    const std::vector<geometry_msgs::PoseStamped>& path,
    std::vector<geometry_msgs::PoseStamped>& newPath)
  {
    newPath.clear();

    for (unsigned int i = 0; i < path.size() - 1; i++)
    {
      std::vector<geometry_msgs::PoseStamped> tmpPath;

      unsigned int end = i + 1;

      while (!planPath(path[i], path[end++], tmpPath))
        if (end > path.size() - 1)
          return i;

      i = end;

      newPath.insert(newPath.end(), tmpPath.begin(), tmpPath.end());
    }

    return path.size()-1;
  }

  int ReedsSheppPlanner::planRecedingPath(
    const std::vector<geometry_msgs::PoseStamped>& path,
    std::vector<geometry_msgs::PoseStamped>& newPath)
  {
    newPath.clear();

    for (unsigned int i = path.size() - 1; i > 0; i--)
    {
      if (planPath(path[0], path[i], newPath))
        return i;
    }

    return 0;
  }



}  // namespace rsband_local_planner

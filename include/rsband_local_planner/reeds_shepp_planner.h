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

#ifndef RSBAND_LOCAL_PLANNER_REEDS_SHEPP_PLANNER_H
#define RSBAND_LOCAL_PLANNER_REEDS_SHEPP_PLANNER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/costmap_model.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include "tf/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"

#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

#include "rsband_local_planner/RSBandPlannerConfig.h"

namespace rsband_local_planner
{

  /**
   * @class ReedsSheppPlanner
   * @brief Planner - ROS wrapper for the Reeds-Shepp state space of OMPL
   */
  class ReedsSheppPlanner
  {
    public:

      /**
       * @brief Constructor
       * @param name: Name used to load planner parameters
       * @param costmapROS: Ptr to the ROS wrapper of the local costmap
       * @param tfBuffer: Ptr to a tf transform buffer
       */
      ReedsSheppPlanner(
        std::string name,
        costmap_2d::Costmap2DROS* costmapROS,
        tf2_ros::Buffer* tfBuffer);

      /**
       * @brief Destructor
       */
      ~ReedsSheppPlanner();

      /**
       * @param name: Name used to load planner parameters
       * @brief Initializes the planner
       * @param costmapROS: Ptr to the ROS wrapper of the local costmap
       * @param tfBuffer: Ptr to a tf transform listener
       */
      void initialize(
        std::string name,
        costmap_2d::Costmap2DROS* costmapROS,
        tf2_ros::Buffer* tfBuffer);

      /**
       * @brief Reconfigures the parameters of the planner
       * @param config: The dynamic reconfigure configuration
       */
      void reconfigure(RSBandPlannerConfig& config);

      /**
       * @brief Plans a Reeds-Shepp path between the start and end poses
       * @param startPose: The start pose of the path
       * @param goalPose: The final pose of the path
       * @param path: The Reeds-Shepp path that will be returned
       * @return true if planning succeeds
       */
      bool planPath(
        const geometry_msgs::PoseStamped& startPose,
        const geometry_msgs::PoseStamped& goalPose,
        std::vector<geometry_msgs::PoseStamped>& path);

      /**
       * @brief Plans a series of Reeds-Shepp paths that connect the path poses
       * @details If a subpath fails the function returns
       * @param path: Contains the path poses to connect via Reeds-Shepp paths
       * @param newPath: Container for the returned path
       * @return The idx of the pose the planning failed at
       */
      int planPathUntilFailure(
        const std::vector<geometry_msgs::PoseStamped>& path,
        std::vector<geometry_msgs::PoseStamped>& newPath);

      /**
       * @brief Plans a series of Reeds-Shepp paths that connect the path poses
       * @details If a subpath fails, the planner continues with the next pose
       * @param path: Contains the path poses to connect via Reeds-Shepp paths
       * @param newPath: Container for the returned path
       * @return The idx of the pose the planning failed at
       */
      int planPathSkipFailures(
        const std::vector<geometry_msgs::PoseStamped>& path,
        std::vector<geometry_msgs::PoseStamped>& newPath);

      /**
       * @brief Plans a Reeds-Shepp path between the start pose and a receding
       * end pose
       * @details Each time the planner fails, it uses the immediate pose before
       * the current end pose
       * @param path: Contains the path poses to connect via Reeds-Shepp paths
       * @param newPath: Container for the returned path
       * @return The idx of the pose the planning failed at
       */
      int planRecedingPath(
        const std::vector<geometry_msgs::PoseStamped>& path,
        std::vector<geometry_msgs::PoseStamped>& newPath);

      double getMinTurningRadius() {return minTurningRadius_;}
      double getMaxPlanningDuration() {return maxPlanningDuration_;}
      double getBX() {return bx_;}
      double getBY() {return by_;}
      void setMinTurningRadius(double rho) {minTurningRadius_ = rho;}
      void setMaxPlanningDuration(double tmax) {maxPlanningDuration_ = tmax;}
      void setBoundaries(double bx, double by);

      //! state checking mode enum
      enum StateCheckingMode{center, footprint};

      //! steering mode enum
      enum RearSteeringMode{none, counter, crab, hybrid};

    private:

      /**
       * @brief Transforms the given pose to the target reference frame
       * @param poseIn: The given pose to transform
       * @param poseOut: The transformed pose container
       * @param newFrameID: The target reference frame to use in the transform
       */
      void transform(const geometry_msgs::PoseStamped& poseIn,
        geometry_msgs::PoseStamped& poseOut, std::string newFrameID);

      /**
       * @brief Transforms the given pose to the target reference frame
       * @param tfIn: The given tf pose to transform
       * @param tfOut: The transformed tf pose container
       * @param newFrameID: The target reference frame to use in the transform
       */
      void transform(const tf::Stamped<tf::Pose>& tfIn,
        tf::Stamped<tf::Pose>& tfOut, std::string newFrameID);

      /**
       * @brief Converts an ompl state to a pose msg
       * @param state: The ompl state to convert to a pose msg
       * @param pose: The converted pose msg container
       */
      void state2pose(
        const ompl::base::State* state, geometry_msgs::PoseStamped& pose);

      /**
       * @brief Converts a pose msg to an ompl state
       * @param pose: The pose to convert to an ompl state
       * @param state: The converted ompl state container
       */
      void pose2state(
        const geometry_msgs::PoseStamped& pose, ompl::base::State* state);

      /**
       * @brief Checks whether a state/pose is valid
       * @details A state is valid if it is within defined boundaries and not
       * in collision with the environment
       * @param si: Ptr to an ompl space information object
       * @param state: Ptr to the state to check for validity
       * @return true if state is valid
       */
      bool isStateValid(
        const ompl::base::SpaceInformation* si, const ompl::base::State *state);


      //! The ompl Reeds-Sheep state space
      ompl::base::StateSpacePtr reedsSheppStateSpace_;
      //! OMPL geometric simple setup object
      ompl::geometric::SimpleSetupPtr simpleSetup_;
      //! planner space boundaries (should match local costmap boundaries)
      ompl::base::RealVectorBounds bounds_;

      //! ptr to costmap
      costmap_2d::Costmap2D* costmap_;
      //! ptr to costmap ros wrapper
      costmap_2d::Costmap2DROS* costmapROS_;
      //! ptr to costmap model
      base_local_planner::CostmapModel* costmapModel_;
      //! robot footprint, used in validity checking
      std::vector<geometry_msgs::Point> footprint_;

      //! tf transform listener
      tf2_ros::Buffer* tfBuffer_;

      //! the reference frame of the robot base
      std::string robotFrame_;
      //! the reference frame of the costmap
      std::string globalFrame_;

      //! the stamp to use in path poses
      ros::Time stamp_;

      //! minimum turning radius of the robot (dependent on rear steering mode)
      double minTurningRadius_;
      //! maximum Reeds-Shepp planning duration
      double maxPlanningDuration_;
      //! number of poses to interpolate in the reeds shepp path
      int interpolationNumPoses_;
      //! state checking mode: center(0) or footprint(1)
      StateCheckingMode stateCheckingMode_;
      //! regard robot pose as free
      bool robotStateValid_;
      //! below this cost, a state is considered valid
      int validStateMaxCost_;
      //! if true considers unknown costmap cells as free
      bool allowUnknown_;
      //! boundary size x
      double bx_;
      //! boundary size y
      double by_;
      //! display planning information
      bool displayPlannerOutput_;

      //! is set if planner is initialized
      bool initialized_;
  };

}  // namespace rsband_local_planner

#endif  // RSBAND_LOCAL_PLANNER_REEDS_SHEPP_PLANNER_H

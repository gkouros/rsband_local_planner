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

#ifndef RSBAND_LOCAL_PLANNER_FUZZY_PTC_H
#define RSBAND_LOCAL_PLANNER_FUZZY_PTC_H

// ros related libraries
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

// dynamic reconfigure configuration
#include "rsband_local_planner/RSBandPlannerConfig.h"

// fuzzylite headers
#include <fl/Headers.h>


namespace rsband_local_planner
{

  /**
   * @class FuzzyPTC
   * @brief Path tracking controller for car like robots, based on fuzzy logic
   */
  class FuzzyPTC
  {
    public:
      /*
       * @brief Constructor
       * @param name: Name used to load the paramaters of the controller
       */
      FuzzyPTC(std::string name);

      /**
       * @brief Destructor
       */
      ~FuzzyPTC();

      /**
       * @brief Reconfigures controller params
       */
      void reconfigure(RSBandPlannerConfig& config);

      /**
       * @brief Computes velocity commands
       * @param path: The path to track
       * @param cmd: The command that will be returned by the controller
       * @return true if a command was produced successfully
       */
      bool computeVelocityCommands(
        const std::vector<geometry_msgs::PoseStamped>& path,
        geometry_msgs::Twist& cmd);

      /**
       * @brief Checks if the goal position and orientation have been reached
       * @param path: The path to follow
       * @return true: if the goal has been reached
       */
      bool isGoalReached(const std::vector<geometry_msgs::PoseStamped>& path);

      //! rear steering mode enum
      enum RearSteeringMode{none, counter, crab, hybrid};

    private:

      /**
       * @brief Initializes controller
       */
      void initialize();

      /**
       * @brief Initializes fuzzy engine
       */
      void initializeFuzzyEngine();

      /**
       * @brief Calculates angular deviation from subgoal
       * @param path: The path that the controller tracks
       * @param subGoalIdx: The current sub goal to track
       * @return The angular deviation from the subgoal
       */
      double calcAngularDeviationError(
        const std::vector<geometry_msgs::PoseStamped>& path,
        unsigned int subGoalIdx);

      /**
       * @brief Calculates the orientation error between the robot and the sub
       * goal
       * @param path: The path that the controller tracks
       * @param subGoalIdx: The path index to the current sub goal
       * @return The orientation error between the robot and the sub goal
       */
      double calcOrientationError(
        const std::vector<geometry_msgs::PoseStamped>& path,
        unsigned int subGoalIdx);

      /**
       * @brief Calculates the distance between the robot and the sub goal
       * @param path: The path that the controller tracks
       * @param subGoalIdx: The path idx to the sub goal
       * @return The distance between the robot and the sub goal
       */
      double calcPositionError(
        const std::vector<geometry_msgs::PoseStamped>& path,
        unsigned int subGoalIdx);

      /**
       * @brief Calculates the lateral deviation from the sub goal
       * @param path: The path that the controller tracks
       * @param subGoalIdx: The path idx to the sub goal
       * @return The lateral deviation from the sub goal
       */
      double calcLateralDeviationError(
        const std::vector<geometry_msgs::PoseStamped>& path,
        unsigned int subGoalIdx);

      /**
       * @brief Calculates the distance between two path waypoints
       * @param path: The path that the controller tracks
       * @param idx1: The path idx to the first waypoint
       * @param idx2: The path idx to the second waypoint
       * @return The distance between the two path waypoints
       */
      double calcDistance(
        const std::vector<geometry_msgs::PoseStamped>& path,
        unsigned int idx1, unsigned int idx2);

      /**
       * @brief Finds the current sub goal to follow
       * @param path: The path that the controller tracks
       * @return The path index to the current sub goal to follow
       */
      unsigned int findSubGoal(
        const std::vector<geometry_msgs::PoseStamped>& path);

      /**
       * @brief Checks whether a path waypoint is a cusp point
       * @param path: The path that the controller tracks
       * @param idx: The point to check if it is a cusp point
       * return true if given path waypoint is a cusp point
       */
      bool isCuspPoint(
        const std::vector<geometry_msgs::PoseStamped>& path,
        unsigned int idx);


      //! private ros node handle
      ros::NodeHandle* pnh_;

      //! variable indicating whether the controller has been initialized
      bool initialized_;

      //! publisher of the current sub goal marker
      ros::Publisher subGoalPub_;
      //! sub goal visualization marker
      visualization_msgs::Marker subGoal_;

      //! rear wheel steering mode (none/counter/crab/hybrid)
      RearSteeringMode rearSteeringMode_;

      //! the robot wheelbase
      double wheelbase_;
      //! the max steering angle of the virtual middle wheel
      double maxSteeringAngle_;
      //! max speed
      double maxSpeed_;
      //! xy goal tolerance threshold
      double xyGoalTolerance_;
      //! yaw goal tolerance
      double yawGoalTolerance_;
      //! distance to goal threshold
      double goalDistThreshold_;
      //! lateral deviation tolerance
      double latDevTolerance_;
      //! subGoal step
      double updateSubGoalDistThreshold_;
      //! display current trajectory information
      bool displayControllerIO_;
      //! if true controller returns zero velocity commands
      bool stop_;

      // fuzzylite related variables

      //! position FLC engine
      fl::Engine* engine_;

      //! direction (forwards/backwards) input variable
      fl::InputVariable* direction_;
      //! robot to goal direction orientation error input variable
      fl::InputVariable* angularDeviationError_;
      //! robot to final goal orientation error input variable
      fl::InputVariable* orientationError_;
      //! robot to goal distance error input variable
      fl::InputVariable* positionError_;
      //! lateral deviation error
      fl::InputVariable* lateralDeviationError_;
      //! steering angle output variable 1
      fl::OutputVariable* frontSteeringAngle_;
      //! rear steering deviation angle output variable
      fl::OutputVariable* rearSteeringDeviationAngle_;
      //! speed output variable
      fl::OutputVariable* speed_;

      //! flc rule block
      fl::RuleBlock* ruleBlock_;

      //! speed rules
      std::vector<std::string> speedRules_;
      //! front steering rules
      std::vector<std::string> frontSteeringRules_;
      //! rear steering deviation rules
      std::vector<std::string> rearSteeringDeviationRules_;

  };

}

#endif  // RSBAND_LOCAL_PLANNER_FUZZY_PTC_H

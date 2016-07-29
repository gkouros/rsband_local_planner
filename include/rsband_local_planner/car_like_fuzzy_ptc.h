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

#ifndef RSBAND_LOCAL_PLANNER_CAR_LIKE_FUZZY_PTC_H
#define RSBAND_LOCAL_PLANNER_CAR_LIKE_FUZZY_PTC_H

// ros related libraries
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

#include "rsband_local_planner/RSBandPlannerConfig.h"

// fuzzylite headers
#include <fl/Headers.h>


namespace rsband_local_planner
{

  class CarLikeFuzzyPTC
  {
    public:
      /*
       * @brief Constructor
       */
      CarLikeFuzzyPTC(std::string name);

      /**
       * @brief Destructor
       */
      ~CarLikeFuzzyPTC();

      /**
       * @brief Initializes controller
       */
      void initialize();

      void reconfigure(RSBandPlannerConfig& config);

      /**
       * @brief Initializes fuzzy engine
       */
      void initializeFuzzyEngine();

      /**
       * @brief Computes velocity commands
       * @return The new velocity command
       */
      bool computeVelocityCommands(
        const std::vector<geometry_msgs::PoseStamped>& path,
        geometry_msgs::Twist& cmd);

      /**
       * @brief Checks if the goal position and orientation have been reached
       * @return true: if the goal has been reached
       * @return false: if the goal has not been reached yet
       */
      bool isGoalReached(const std::vector<geometry_msgs::PoseStamped>& path);

      double calcOrientationError(
        const std::vector<geometry_msgs::PoseStamped>& path,
        unsigned int subGoalIdx);

      double calcFinalOrientationError(
        const std::vector<geometry_msgs::PoseStamped>& path,
        unsigned int subGoalIdx);

      double calcPositionError(
        const std::vector<geometry_msgs::PoseStamped>& path,
        unsigned int subGoalIdx);

      double calcLateralDeviationError(
        const std::vector<geometry_msgs::PoseStamped>& path,
        unsigned int subGoalIdx);

      double calcLongitudinalDistanceFromGoal(
        const std::vector<geometry_msgs::PoseStamped>& path,
        unsigned int subGoalIdx);

      double calcDistance(
        const std::vector<geometry_msgs::PoseStamped>& path,
        unsigned int idx1, unsigned int idx2);

      unsigned int findSubGoal(
        const std::vector<geometry_msgs::PoseStamped>& path);

      bool isCuspPoint(
        const std::vector<geometry_msgs::PoseStamped>& path,
        unsigned int idx);

    private:

      //!< private ros node handle
      ros::NodeHandle* pnh_;

      ros::Publisher subGoalPub_;
      visualization_msgs::Marker subGoal_;

      //!< rear wheel steering mode (zero/counter/crab/hybrid)
      std::string rearSteeringMode_;

      //!< the robot wheelbase
      double wheelbase_;
      //!< the max steering angle of the virtual middle wheel
      double maxSteeringAngle_;
      //!< max speed
      double maxSpeed_;
      //!< xy goal tolerance threshold
      double xyGoalTolerance_;
      //!< yaw goal tolerance
      double yawGoalTolerance_;
      //!< lateral deviation tolerance
      double latDevTolerance_;
      //!< subGoal step
      double updateSubGoalDistThreshold_;
      //!< display current trajectory information
      bool displayControllerIO_;
      //!< if true controller returns zero velocity commands
      bool stop_;

      //!< position FLC engine
      fl::Engine* engine_;
      //!< flc rule block
      fl::RuleBlock* ruleBlock_;
      //!< direction (forwards/backwards) input variable
      fl::InputVariable* direction_;
      //!< robot to goal direction orientation error input variable
      fl::InputVariable* orientationError_;
      //!< robot to final goal orientation error input variable
      fl::InputVariable* finalOrientationError_;
      //!< robot to goal distance error input variable
      fl::InputVariable* positionError_;
      //!< lateral deviation error
      fl::InputVariable* lateralDeviationError_;
      //!< steering angle output variable 1
      fl::OutputVariable* frontSteeringAngle_;
      //!< rear steering angle output variable
      fl::OutputVariable* rearSteeringAngle_;
      //!< speed output variable
      fl::OutputVariable* speed_;
  };

}

#endif  // RSBAND_LOCAL_PLANNER_CAR_LIKE_FUZZY_PTC_H

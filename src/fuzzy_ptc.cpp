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

#include "rsband_local_planner/fuzzy_ptc.h"

#include <ros/package.h>
#include <math.h>
#include <tf/tf.h>
#include <boost/foreach.hpp>
#include <yaml-cpp/yaml.h>

namespace
{
  double rad2deg(double rad)
  {
    return rad * 180.0 / M_PI;
  }

  double deg2rad(double deg)
  {
    return deg * M_PI / 180.0;
  }
}

namespace rsband_local_planner
{

  FuzzyPTC::FuzzyPTC(std::string name) : initialized_(false)
  {
    pnh_ = new ros::NodeHandle("~/" + name);
    initialize();
  }


  FuzzyPTC::~FuzzyPTC()
  {
    delete engine_;
    // no need to delete fuzzy variables and ruleblock, since it is done in
    // the destructor of the engine
  }


  void FuzzyPTC::initialize()
  {
    if (initialized_)
    {
      ROS_ERROR("Controller already initialized!");
      return;
    }

    // load fuzzy rules
    if (pnh_->hasParam("fuzzy_rules"))  // load from param server if available
    {
      if (pnh_->getParam("fuzzy_rules/speed_rules", speedRules_))
      {
        ROS_FATAL("Failed to load speed fuzzy rules!");
        exit(EXIT_FAILURE);
      }

      if (pnh_->getParam("fuzzy_rules/front_steering_rules",
          frontSteeringRules_))
      {
        ROS_FATAL("Failed to load front steering fuzzy rules!");
        exit(EXIT_FAILURE);
      }

      if (pnh_->getParam("fuzzy_rules/rear_steering_deviation_rules",
          rearSteeringDeviationRules_))
        ROS_WARN("Failed to load rear steering deviation fuzzy rules!");
    }
    else  // load from yaml file in pkg
    {
      std::string pkgPath = ros::package::getPath("rsband_local_planner");
      YAML::Node config = YAML::LoadFile(
        pkgPath + "/cfg/path_tracking_controller_fuzzy_rules.yaml");

      speedRules_ =
        config["fuzzy_rules"]["speed_rules"].as< std::vector<std::string> >();
      frontSteeringRules_ =
        config["fuzzy_rules"]["front_steering_rules"].as<
        std::vector<std::string> >();
      rearSteeringDeviationRules_ =
        config["fuzzy_rules"]["rear_steering_deviation_rules"].as<
        std::vector<std::string> >();
    }

    // initialize sub goal msg and publisher
    subGoal_.type = visualization_msgs::Marker::SPHERE;
    subGoal_.scale.x = subGoal_.scale.y = subGoal_.scale.z = 0.1;
    subGoal_.color.b = 1.0f; subGoal_.color.a = 1;
    subGoalPub_ = pnh_->advertise<visualization_msgs::Marker>("sub_goal", 1);

    initialized_ = true;
  }


  void FuzzyPTC::reconfigure(RSBandPlannerConfig& config)
  {
    wheelbase_ = config.wheelbase;
    maxSteeringAngle_ = config.max_steering_angle;
    maxSpeed_ = config.max_speed;
    xyGoalTolerance_ = config.xy_goal_tolerance;
    yawGoalTolerance_ = config.yaw_goal_tolerance;
    latDevTolerance_ = config.lateral_deviation_tolerance;
    updateSubGoalDistThreshold_ = config.update_sub_goal_dist_threshold;
    goalDistThreshold_ = config.goal_dist_threshold;
    displayControllerIO_ = config.display_controller_io;
    stop_ = config.stop;
    rearSteeringMode_ =
      static_cast<RearSteeringMode>(config.rear_steering_mode);

    // reinitialize fuzzy engine with the updated parameters
    initializeFuzzyEngine();
  }


  void FuzzyPTC::initializeFuzzyEngine()
  {
    if (!initialized_)
    {
      ROS_ERROR("Fuzzy Engine Initialization attempted before controller "
        "initialization");
      return;
    }
    // initialize FLC engine
    engine_ = new fl::Engine();
    engine_->setName("fuzzy_path_tracking_controller");

    //========================= INPUT VARIABLES ================================

    // direction input variable
    direction_ = new fl::InputVariable;
    direction_->setEnabled(true);
    direction_->setName("Direction");
    direction_->setRange(-1.0, 1.0);
    direction_->addTerm(new fl::Ramp("FW", -0.1, 0.1));
    direction_->addTerm(new fl::Ramp("BW", 0.1, -0.1));
    engine_->addInputVariable(direction_);

    // angle deviation error fuzzy input variable initialization
    angularDeviationError_ = new fl::InputVariable;
    angularDeviationError_->setEnabled(true);
    angularDeviationError_->setName("Ea");
    angularDeviationError_->setRange(-180.0, 180.0);
    angularDeviationError_->addTerm(new fl::Trapezoid("RBR", -180.0, -180.0, -175.0, -165.0));
    angularDeviationError_->addTerm(new fl::Trapezoid("RR", -175.0, -165.0, -130.0, -120.0));
    angularDeviationError_->addTerm(new fl::Trapezoid("SR", -130.0, -120.0, -35.0, -25.0));
    angularDeviationError_->addTerm(new fl::Trapezoid("FR", -35.0, -25.0, -15.0, -5.0));
    angularDeviationError_->addTerm(new fl::Trapezoid("FA", -15.0, -5.0, 5.0, 15.0));
    angularDeviationError_->addTerm(new fl::Trapezoid("FL", 5.0, 15.0, 25.0, 35.0));
    angularDeviationError_->addTerm(new fl::Trapezoid("SL", 25.0, 35.0, 120.0, 130.0));
    angularDeviationError_->addTerm(new fl::Trapezoid("RL", 120.0, 130.0, 165.0, 175.0));
    angularDeviationError_->addTerm(new fl::Trapezoid("RBL", 165.0, 175.0, 180.0, 180.0));
    engine_->addInputVariable(angularDeviationError_);

    // orientation error input variable initialization
    orientationError_ = new fl::InputVariable;
    orientationError_->setEnabled(true);
    orientationError_->setName("Eo");
    orientationError_->setRange(-180.0, 180.0);
    orientationError_->addTerm(new fl::Trapezoid("RBR", -180.0, -180.0, -175.0, -165.0));
    orientationError_->addTerm(new fl::Trapezoid("RR", -175.0, -165.0, -130.0, -120.0));
    orientationError_->addTerm(new fl::Trapezoid("SR", -130.0, -120.0, -35.0, -25.0));
    orientationError_->addTerm(new fl::Trapezoid("FR", -40.0, -30.0, -15.0, -10.0));
    orientationError_->addTerm(new fl::Trapezoid("FA", -15.0, -5.0, 5.0, 15.0));
    orientationError_->addTerm(new fl::Trapezoid("FL", 5.0, 15.0, 25.0, 35.0));
    orientationError_->addTerm(new fl::Trapezoid("SL", 25.0, 35.0, 120.0, 130.0));
    orientationError_->addTerm(new fl::Trapezoid("RL", 120.0, 130.0, 165.0, 175.0));
    orientationError_->addTerm(new fl::Trapezoid("RBL", 165.0, 175.0, 180.0, 180.0));
    engine_->addInputVariable(orientationError_);

    // position error input variable initialization
    positionError_ = new fl::InputVariable;
    positionError_->setEnabled(true);
    positionError_->setName("Ep");
    positionError_->setRange(0.0, std::numeric_limits<double>::infinity());
    positionError_->addTerm(new fl::Trapezoid("CLOSE", 0.0, 0.0, goalDistThreshold_, 2 * goalDistThreshold_));
    positionError_->addTerm(new fl::Ramp("FAR", goalDistThreshold_, 2 * goalDistThreshold_));
    engine_->addInputVariable(positionError_);

    // lateral deviation error input variable initialization
    lateralDeviationError_ = new fl::InputVariable;
    lateralDeviationError_->setEnabled(true);
    lateralDeviationError_->setName("Ey");
    lateralDeviationError_->setRange(
      -std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity());
    lateralDeviationError_->addTerm(new fl::Ramp("BN", -latDevTolerance_, -3*latDevTolerance_));
    lateralDeviationError_->addTerm(new fl::Triangle("SN", -2*latDevTolerance_, 0.0));
    lateralDeviationError_->addTerm(new fl::Triangle("Z", -latDevTolerance_, latDevTolerance_));
    lateralDeviationError_->addTerm(new fl::Triangle("SP", 0.0, 2*latDevTolerance_));
    lateralDeviationError_->addTerm(new fl::Ramp("BP", latDevTolerance_, 3*latDevTolerance_));
    engine_->addInputVariable(lateralDeviationError_);

    //========================= OUTPUT VARIABLES ===============================

    // front steering angle output variable initialization
    frontSteeringAngle_ = new fl::OutputVariable;
    frontSteeringAngle_->setEnabled(true);
    frontSteeringAngle_->setName("FSA");
    frontSteeringAngle_->setRange(-rad2deg(maxSteeringAngle_), rad2deg(maxSteeringAngle_));
    frontSteeringAngle_->fuzzyOutput()->setAggregation(fl::null);
    frontSteeringAngle_->setDefuzzifier(new fl::WeightedAverage("TakagiSugeno"));
    frontSteeringAngle_->setDefaultValue(0.0);
    frontSteeringAngle_->setLockPreviousValue(true);
    frontSteeringAngle_->setLockValueInRange(false);
    frontSteeringAngle_->addTerm(new fl::Constant("RH", -rad2deg(maxSteeringAngle_)));
    frontSteeringAngle_->addTerm(new fl::Constant("RL", -rad2deg(maxSteeringAngle_) / 2));
    frontSteeringAngle_->addTerm(new fl::Constant("Z", 0.0));
    frontSteeringAngle_->addTerm(new fl::Constant("LL", rad2deg(maxSteeringAngle_) / 2));
    frontSteeringAngle_->addTerm(new fl::Constant("LH", rad2deg(maxSteeringAngle_)));
    engine_->addOutputVariable(frontSteeringAngle_);

    // rear steering angle output variable initialization
    rearSteeringDeviationAngle_ = new fl::OutputVariable;
    rearSteeringDeviationAngle_->setEnabled(true);
    rearSteeringDeviationAngle_->setName("RSDA");
    rearSteeringDeviationAngle_->setRange(-rad2deg(maxSteeringAngle_), rad2deg(maxSteeringAngle_));
    rearSteeringDeviationAngle_->fuzzyOutput()->setAggregation(fl::null);
    rearSteeringDeviationAngle_->setDefuzzifier(new fl::WeightedAverage("TakagiSugeno"));
    rearSteeringDeviationAngle_->setDefaultValue(0);
    rearSteeringDeviationAngle_->setLockPreviousValue(true);
    rearSteeringDeviationAngle_->setLockValueInRange(false);
    rearSteeringDeviationAngle_->addTerm(new fl::Constant("RH", -rad2deg(maxSteeringAngle_)));
    rearSteeringDeviationAngle_->addTerm(new fl::Constant("RL", -rad2deg(maxSteeringAngle_) / 2));
    rearSteeringDeviationAngle_->addTerm(new fl::Constant("Z", 0.0));
    rearSteeringDeviationAngle_->addTerm(new fl::Constant("LL", rad2deg(maxSteeringAngle_) / 2));
    rearSteeringDeviationAngle_->addTerm(new fl::Constant("LH", rad2deg(maxSteeringAngle_)));
    engine_->addOutputVariable(rearSteeringDeviationAngle_);

    // speed output variable initialization
    speed_ = new fl::OutputVariable;
    speed_->setEnabled(true);
    speed_->setName("Speed");
    speed_->setDefaultValue(0.0);
    speed_->setRange(0.0, maxSpeed_);
    speed_->fuzzyOutput()->setAggregation(fl::null);
    speed_->setDefuzzifier(new fl::WeightedAverage("TakagiSugeno"));
    speed_->addTerm(new fl::Constant("SLOW", maxSpeed_ / 2));
    speed_->addTerm(new fl::Constant("FAST", maxSpeed_));
    engine_->addOutputVariable(speed_);


    //=============================== RULES ====================================

    // rule block initialization
    ruleBlock_ = new fl::RuleBlock;

    // front steering rules
    BOOST_FOREACH(std::string rule, frontSteeringRules_)
      ruleBlock_->addRule(fl::Rule::parse(rule, engine_));

    // rear steering rules
    BOOST_FOREACH(std::string rule, rearSteeringDeviationRules_)
      ruleBlock_->addRule(fl::Rule::parse(rule, engine_));

    // speed rules
    BOOST_FOREACH(std::string rule, speedRules_)
      ruleBlock_->addRule(fl::Rule::parse(rule, engine_));

    engine_->addRuleBlock(ruleBlock_);
    engine_->configure(
      "Minimum", "Maximum", "Minimum", "Maximum", "WeightedAverage", "General");

    std::string status;
    if (!engine_->isReady(&status))
        throw fl::Exception("Engine not ready. "
          "The following errors were encountered:\n" + status, FL_AT);
  }


  bool FuzzyPTC::computeVelocityCommands(
    const std::vector<geometry_msgs::PoseStamped>& path,
    geometry_msgs::Twist& cmd)
  {
    if (path.empty())
    {
      ROS_ERROR("Plan given to path tracking controller is empty!");
      return false;
    }

    unsigned int subGoalIdx = findSubGoal(path);

    double ea = calcAngularDeviationError(path, subGoalIdx);
    double eo = calcOrientationError(path, subGoalIdx);
    double ep = calcPositionError(path, subGoalIdx);
    double ey = calcLateralDeviationError(path, subGoalIdx);
    int drcn = (rad2deg(fabs(ea)) < 120) ? 1 : -1;

    ROS_INFO_COND(displayControllerIO_, "Ea: %f, Eo: %f, Ep: %f, Ey: %f",
      rad2deg(ea), rad2deg(eo), ep, ey);

    if (isGoalReached(path))
    {
      cmd = *new geometry_msgs::Twist;  // return cleared twist
      return true;
    }

    angularDeviationError_->setValue(rad2deg(ea));
    orientationError_->setValue(rad2deg(eo));
    positionError_->setValue(ep);
    lateralDeviationError_->setValue(ey);
    direction_->setValue(drcn);

    engine_->process();

    double vel = drcn * speed_->getValue();  // linear velocity
    double fsa = deg2rad(frontSteeringAngle_->getValue());  // front steering angle

    double rsa;  // rear steering angle

    switch(rearSteeringMode_)
    {
      case none:
        rsa = 0.0;
        break;
      case counter:
        rsa = -fsa;
        break;
      case crab:
        rsa = deg2rad(rearSteeringDeviationAngle_->getValue());
        break;
      case hybrid:
        rsa = -fsa + deg2rad(rearSteeringDeviationAngle_->getValue());
        break;
      default:
        ROS_FATAL("Invalid Steering Mode. Exiting...");
        exit(EXIT_FAILURE);
    }

    // clamp rsa in case -fsa+rsda exceeds limits
    rsa = std::min(maxSteeringAngle_, std::max(-maxSteeringAngle_, rsa));

    double beta = atan((tan(fsa) + tan(rsa)) / 2);  // angle between vel.x and vel.y

    // create command
    cmd.linear.x = vel * sqrt(1 / (1 + pow(tan(beta), 2)));
    cmd.linear.y = vel * tan(beta) * sqrt(1 / (1 + pow(tan(beta), 2)));
    cmd.angular.z = vel * cos(beta) * (tan(fsa) - tan(rsa)) / wheelbase_;

    ROS_INFO_COND(displayControllerIO_,
      "vx: %.3f, vy: %.3f, w: %.3f, fsa: %.3f, rsa: %.3f",
      cmd.linear.x, cmd.linear.y, cmd.angular.z, rad2deg(fsa), rad2deg(rsa));

    if (std::isnan(cmd.linear.x) or std::isnan(cmd.linear.y))
    {
      ROS_ERROR("Speed=Nan. Something went wrong!");
      cmd.linear.x = 0.0;
      cmd.linear.y = 0.0;
      return false;
    }
    if (std::isnan(cmd.angular.z))
    {
      ROS_ERROR("RotVel=Nan. Something went wrong!");
      cmd.angular.z = 0.0;
      return false;
    }

    if (stop_)
    {
      cmd.linear.x = 0.0;
      cmd.linear.y = 0.0;
      cmd.angular.z = 0.0;
    }

    return true;
  }


  bool FuzzyPTC::isGoalReached(
    const std::vector<geometry_msgs::PoseStamped>& path)
  {
    bool positionReached =
      calcPositionError(path, path.size()-1) < xyGoalTolerance_;
    bool orientationReached =
      fabs(calcOrientationError(path, path.size()-1)) < yawGoalTolerance_;
    return positionReached && orientationReached;
  }


  double FuzzyPTC::calcAngularDeviationError(
    const std::vector<geometry_msgs::PoseStamped>& path,
    unsigned int subGoalIdx)
  {
    return atan2(path[subGoalIdx].pose.position.y,
      path[subGoalIdx].pose.position.x);
  }


  double FuzzyPTC::calcOrientationError(
    const std::vector<geometry_msgs::PoseStamped>& path,
    unsigned int subGoalIdx)
  {
    return tf::getYaw(path[subGoalIdx].pose.orientation);
  }


  double FuzzyPTC::calcPositionError(
    const std::vector<geometry_msgs::PoseStamped>& path,
    unsigned int subGoalIdx)
  {
    return hypot(path[subGoalIdx].pose.position.x, path[subGoalIdx].pose.position.y);
  }


  double FuzzyPTC::calcLateralDeviationError(
    const std::vector<geometry_msgs::PoseStamped>& path,
    unsigned int subGoalIdx)
  {
    return calcPositionError(path, subGoalIdx)
      * sin(calcAngularDeviationError(path, subGoalIdx));
  }


  double FuzzyPTC::calcDistance(
    const std::vector<geometry_msgs::PoseStamped>& path,
    unsigned int idx1, unsigned int idx2)
  {
    double x0, x1, y0, y1;
    x0 = path[idx1].pose.position.x;
    y0 = path[idx1].pose.position.y;
    x1 = path[idx2].pose.position.x;
    y1 = path[idx2].pose.position.y;
    return hypot(x1 - x0, y1 - y0);
  }


  unsigned int FuzzyPTC::findSubGoal(
    const std::vector<geometry_msgs::PoseStamped>& path)
  {
    unsigned int subGoalIdx = 0;
    double distance = 0;

    while (subGoalIdx < path.size()-1 && distance < updateSubGoalDistThreshold_)
    {
      distance += calcDistance(path, subGoalIdx, subGoalIdx+1);
      subGoalIdx++;

      if (isCuspPoint(path, subGoalIdx)
        && calcPositionError(path, subGoalIdx) > xyGoalTolerance_)
      {
        break;
      }
    }

    subGoal_.header = path.front().header;
    subGoal_.pose = path[subGoalIdx].pose;
    subGoalPub_.publish(subGoal_);

    return subGoalIdx;
  }


  bool FuzzyPTC::isCuspPoint(
    const std::vector<geometry_msgs::PoseStamped>& path,
    unsigned int idx)
  {
    if (idx < 1 || idx >= path.size() - 2)
      return false;

    geometry_msgs::Point a, b;
    a.x = path[idx].pose.position.x - path[idx-1].pose.position.x;
    a.y = path[idx].pose.position.y - path[idx-1].pose.position.y;
    b.x = path[idx].pose.position.x - path[idx+1].pose.position.x;
    b.y = path[idx].pose.position.y - path[idx+1].pose.position.y;

    double angle = acos((a.x * b.x + a.y * b.y) / hypot(a.x, a.y)
      / hypot(b.x, b.y));

    return !std::isnan(angle) && (fabs(angle) < 1.0);// && (fabs(angle) > 0.085);
  }

}

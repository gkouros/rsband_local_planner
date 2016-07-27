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

#include "rsband_local_planner/car_like_fuzzy_ptc.h"
#include <math.h>
#include <tf/tf.h>

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

  CarLikeFuzzyPTC::CarLikeFuzzyPTC(std::string name)
  {
    pnh_ = new ros::NodeHandle("~/" + name);
    initialize();
  }


  CarLikeFuzzyPTC::~CarLikeFuzzyPTC()
  {
    delete engine_;
    // no need to delete fuzzy variables and ruleblock, since it is done in
    // the destructor of the engine
  }


  void CarLikeFuzzyPTC::initialize()
  {
    pnh_->param("wheelbase", wheelbase_, 0.32);
    pnh_->param("max_steering_angle", maxSteeringAngle_, 0.35);
    pnh_->param("max_speed", maxSpeed_, 0.2);
    pnh_->param("updateSubGoalDistThreshold", updateSubGoalDistThreshold_, 0.4);
    pnh_->param("xy_goal_tolerance", xyGoalTolerance_, 0.25);
    pnh_->param("yaw_goal_tolerance", yawGoalTolerance_, 0.1);
    pnh_->param("lateral_deviation_tolerance", latDevTolerance_, 0.2);
    pnh_->param<std::string>("rear_steering_mode", rearSteeringMode_, "none");
    pnh_->param<bool>("display_controller_io", displayControllerIO_, false);

    if (!(rearSteeringMode_ == "counter" || rearSteeringMode_ == "crab"
      || rearSteeringMode_ == "hybrid" || rearSteeringMode_ == "none"))
    {
      ROS_FATAL("Invalid rear steering mode selected."
        "Available modes are {none, counter, crab, hybrid}. Exiting...");
      exit(EXIT_FAILURE);
    }

    // initialize sub goal msg and publisher
    subGoal_.type = visualization_msgs::Marker::SPHERE;
    subGoal_.scale.x = subGoal_.scale.y = subGoal_.scale.z = 0.1;
    subGoal_.color.b = 1.0f; subGoal_.color.a = 1;
    subGoalPub_ = pnh_->advertise<visualization_msgs::Marker>("sub_goal", 1);

    // initialize fuzzy engine
    initializeFuzzyEngine();
  }


  void CarLikeFuzzyPTC::reconfigure(RSBandPlannerConfig& config)
  {
    wheelbase_ = config.wheelbase;
    maxSteeringAngle_ = config.max_steering_angle;
    maxSpeed_ = config.max_speed;
    xyGoalTolerance_ = config.xy_goal_tolerance;
    yawGoalTolerance_ = config.yaw_goal_tolerance;
    latDevTolerance_ = config.lateral_deviation_tolerance;
    updateSubGoalDistThreshold_ = config.update_sub_goal_dist_threshold;
    displayControllerIO_ = config.display_controller_io;
    stop_ = config.stop;

    switch(config.rear_steering_mode)
    {
      case 0:
        rearSteeringMode_ = "none";
        break;
      case 1:
        rearSteeringMode_ = "counter";
        break;
      case 2:
        rearSteeringMode_ = "crab";
        break;
      case 3:
        rearSteeringMode_ = "hybrid";
        break;
    }

    // reinitialize fuzzy engine with the updated parameters
    initializeFuzzyEngine();
  }


  void CarLikeFuzzyPTC::initializeFuzzyEngine()
  {
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

    // orientation error input variable initialization
    orientationError_ = new fl::InputVariable;
    orientationError_->setEnabled(true);
    orientationError_->setName("Eo");
    orientationError_->setRange(-180.0, 180.0);
    orientationError_->addTerm(new fl::Trapezoid("RBR", -180.0, -180.0, -175.0, -165.0));
    orientationError_->addTerm(new fl::Trapezoid("RR", -175.0, -165.0, -130.0, -120.0));
    orientationError_->addTerm(new fl::Trapezoid("SR", -130.0, -120.0, -40.0, -30.0));
    orientationError_->addTerm(new fl::Trapezoid("FR", -40.0, -30.0, -15.0, -10.0));
    orientationError_->addTerm(new fl::Trapezoid("FA", -15.0, -5.0, 5.0, 15.0));
    orientationError_->addTerm(new fl::Trapezoid("FL", 5.0, 15.0, 30.0, 40.0));
    orientationError_->addTerm(new fl::Trapezoid("SL", 30.0, 40.0, 120.0, 130.0));
    orientationError_->addTerm(new fl::Trapezoid("RL", 120.0, 130.0, 165.0, 175.0));
    orientationError_->addTerm(new fl::Trapezoid("RBL", 165.0, 175.0, 180.0, 180.0));
    engine_->addInputVariable(orientationError_);

    // final orientation error fuzzy input variable initialization
    finalOrientationError_ = new fl::InputVariable;
    finalOrientationError_->setEnabled(true);
    finalOrientationError_->setName("Efo");
    finalOrientationError_->setRange(-180.0, 180.0);
    finalOrientationError_->addTerm(new fl::Trapezoid("RBR", -180.0, -180.0, -175.0, -165.0));
    finalOrientationError_->addTerm(new fl::Trapezoid("RR", -175.0, -165.0, -130.0, -120.0));
    finalOrientationError_->addTerm(new fl::Trapezoid("SR", -130.0, -120.0, -40.0, -30.0));
    finalOrientationError_->addTerm(new fl::Trapezoid("FR", -40.0, -30.0, -15.0, -5.0));
    finalOrientationError_->addTerm(new fl::Trapezoid("FA", -15.0, -5.0, 10.0, 5.0));
    finalOrientationError_->addTerm(new fl::Trapezoid("FL", 5.0, 15.0, 30.0, 40.0));
    finalOrientationError_->addTerm(new fl::Trapezoid("SL", 30.0, 40.0, 120.0, 130.0));
    finalOrientationError_->addTerm(new fl::Trapezoid("RL", 120.0, 130.0, 165.0, 175.0));
    finalOrientationError_->addTerm(new fl::Trapezoid("RBL", 165.0, 175.0, 180.0, 180.0));
    engine_->addInputVariable(finalOrientationError_);

    // position error input variable initialization
    positionError_ = new fl::InputVariable;
    positionError_->setEnabled(true);
    positionError_->setName("Ep");
    positionError_->setRange(0.0, std::numeric_limits<double>::infinity());
    positionError_->addTerm(new fl::Trapezoid("CLOSE", 0.0, 0.0, xyGoalTolerance_, 2 * xyGoalTolerance_));
    positionError_->addTerm(new fl::Ramp("FAR", xyGoalTolerance_, 2 * xyGoalTolerance_));
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
    frontSteeringAngle_->fuzzyOutput()->setAccumulation(fl::null);
    frontSteeringAngle_->setDefuzzifier(new fl::WeightedAverage("TakagiSugeno"));
    frontSteeringAngle_->setDefaultValue(0.0);
    frontSteeringAngle_->setLockPreviousOutputValue(true);
    frontSteeringAngle_->setLockOutputValueInRange(false);
    frontSteeringAngle_->addTerm(new fl::Constant("RH", -rad2deg(maxSteeringAngle_)));
    frontSteeringAngle_->addTerm(new fl::Constant("RL", -rad2deg(maxSteeringAngle_) / 2));
    frontSteeringAngle_->addTerm(new fl::Constant("Z", 0.0));
    frontSteeringAngle_->addTerm(new fl::Constant("LL", rad2deg(maxSteeringAngle_) / 2));
    frontSteeringAngle_->addTerm(new fl::Constant("LH", rad2deg(maxSteeringAngle_)));
    engine_->addOutputVariable(frontSteeringAngle_);

    // rear steering angle output variable initialization
    rearSteeringAngle_ = new fl::OutputVariable;
    rearSteeringAngle_->setEnabled(true);
    rearSteeringAngle_->setName("RSA");
    rearSteeringAngle_->setRange(-rad2deg(maxSteeringAngle_), rad2deg(maxSteeringAngle_));
    rearSteeringAngle_->fuzzyOutput()->setAccumulation(fl::null);
    rearSteeringAngle_->setDefuzzifier(new fl::WeightedAverage("TakagiSugeno"));
    rearSteeringAngle_->setDefaultValue(fl::null);
    rearSteeringAngle_->setLockPreviousOutputValue(true);
    rearSteeringAngle_->setLockOutputValueInRange(false);
    rearSteeringAngle_->addTerm(new fl::Constant("RH", -rad2deg(maxSteeringAngle_)));
    rearSteeringAngle_->addTerm(new fl::Constant("RL", -rad2deg(maxSteeringAngle_) / 2));
    rearSteeringAngle_->addTerm(new fl::Constant("Z", 0.0));
    rearSteeringAngle_->addTerm(new fl::Constant("LL", rad2deg(maxSteeringAngle_) / 2));
    rearSteeringAngle_->addTerm(new fl::Constant("LH", rad2deg(maxSteeringAngle_)));
    engine_->addOutputVariable(rearSteeringAngle_);

    // speed output variable initialization
    speed_ = new fl::OutputVariable;
    speed_->setEnabled(true);
    speed_->setName("Speed");
    speed_->setDefaultValue(0.0);
    speed_->setRange(0.0, maxSpeed_);
    speed_->fuzzyOutput()->setAccumulation(fl::null);
    speed_->setDefuzzifier(new fl::WeightedAverage("TakagiSugeno"));
    // speed_->addTerm(new fl::Constant("REVERSE", -maxSpeed_));
    speed_->addTerm(new fl::Constant("SLOW", maxSpeed_ / 2));
    speed_->addTerm(new fl::Constant("FAST", maxSpeed_));
    engine_->addOutputVariable(speed_);

    //=============================== RULES ====================================
    // rule block initialization
    ruleBlock_ = new fl::RuleBlock;

    // front steering rules
    ruleBlock_->addRule(fl::Rule::parse("if Ep is FAR and Eo is RBL then FSA is Z", engine_));
    ruleBlock_->addRule(fl::Rule::parse("if Ep is FAR and Eo is RL  then FSA is LH", engine_));
    ruleBlock_->addRule(fl::Rule::parse("if Ep is FAR and Eo is SL  then FSA is LH", engine_));
    ruleBlock_->addRule(fl::Rule::parse("if Ep is FAR and Eo is FL  then FSA is LL", engine_));
    ruleBlock_->addRule(fl::Rule::parse("if Ep is FAR and Eo is FA  then FSA is Z", engine_));
    ruleBlock_->addRule(fl::Rule::parse("if Ep is FAR and Eo is FR  then FSA is RL", engine_));
    ruleBlock_->addRule(fl::Rule::parse("if Ep is FAR and Eo is SR  then FSA is RH", engine_));
    ruleBlock_->addRule(fl::Rule::parse("if Ep is FAR and Eo is RR  then FSA is RH", engine_));
    ruleBlock_->addRule(fl::Rule::parse("if Ep is FAR and Eo is RBR then FSA is Z", engine_));
    ruleBlock_->addRule(fl::Rule::parse("if Ep is CLOSE and Direction is FW and Efo is SL then FSA is LH", engine_));
    ruleBlock_->addRule(fl::Rule::parse("if Ep is CLOSE and Direction is FW and Efo is FL then FSA is LL", engine_));
    ruleBlock_->addRule(fl::Rule::parse("if Ep is CLOSE and Direction is FW and Efo is FA then FSA is Z", engine_));
    ruleBlock_->addRule(fl::Rule::parse("if Ep is CLOSE and Direction is FW and Efo is FR then FSA is RL", engine_));
    ruleBlock_->addRule(fl::Rule::parse("if Ep is CLOSE and Direction is FW and Efo is SR then FSA is RH", engine_));
    ruleBlock_->addRule(fl::Rule::parse("if Ep is CLOSE and Direction is BW and Efo is SL then FSA is RH", engine_));
    ruleBlock_->addRule(fl::Rule::parse("if Ep is CLOSE and Direction is BW and Efo is FL then FSA is RL", engine_));
    ruleBlock_->addRule(fl::Rule::parse("if Ep is CLOSE and Direction is BW and Efo is FR then FSA is LH", engine_));
    ruleBlock_->addRule(fl::Rule::parse("if Ep is CLOSE and Direction is BW and Efo is SR then FSA is LL", engine_));

    // rear steering rules
    if (rearSteeringMode_ == "crab")
    {
      ruleBlock_->addRule(fl::Rule::parse("if Direction is FW and Efo is somewhat FA and Ey is BP then RSA is RH", engine_));
      ruleBlock_->addRule(fl::Rule::parse("if Direction is FW and Efo is somewhat FA and Ey is SP then RSA is RH", engine_));
      ruleBlock_->addRule(fl::Rule::parse("if Direction is FW and Efo is somewhat FA and Ey is Z then RSA is Z", engine_));
      ruleBlock_->addRule(fl::Rule::parse("if Direction is FW and Efo is somewhat FA and Ey is SN then RSA is LH", engine_));
      ruleBlock_->addRule(fl::Rule::parse("if Direction is FW and Efo is somewhat FA and Ey is BN then RSA is LH", engine_));

      ruleBlock_->addRule(fl::Rule::parse("if Direction is BW and Efo is somewhat FA and Ey is BP then RSA is LH", engine_));
      ruleBlock_->addRule(fl::Rule::parse("if Direction is BW and Efo is somewhat FA and Ey is SP then RSA is LH", engine_));
      ruleBlock_->addRule(fl::Rule::parse("if Direction is BW and Efo is somewhat FA and Ey is Z then RSA is Z", engine_));
      ruleBlock_->addRule(fl::Rule::parse("if Direction is BW and Efo is somewhat FA and Ey is SN then RSA is RH", engine_));
      ruleBlock_->addRule(fl::Rule::parse("if Direction is BW and Efo is somewhat FA and Ey is BN then RSA is RH", engine_));
      ruleBlock_->addRule(fl::Rule::parse("if Efo is not FA then RSA is Z", engine_));
    }
    else if (rearSteeringMode_ == "counter")
    { // use same rules as in front steering but with opposite steering angle
      ruleBlock_->addRule(fl::Rule::parse("if Ep is FAR and Eo is RBL then RSA is Z", engine_));
      ruleBlock_->addRule(fl::Rule::parse("if Ep is FAR and Eo is RL  then RSA is RL", engine_));
      ruleBlock_->addRule(fl::Rule::parse("if Ep is FAR and Eo is SL  then RSA is RH", engine_));
      ruleBlock_->addRule(fl::Rule::parse("if Ep is FAR and Eo is FL  then RSA is RL", engine_));
      ruleBlock_->addRule(fl::Rule::parse("if Ep is FAR and Eo is FA  then RSA is Z", engine_));
      ruleBlock_->addRule(fl::Rule::parse("if Ep is FAR and Eo is FR  then RSA is LL", engine_));
      ruleBlock_->addRule(fl::Rule::parse("if Ep is FAR and Eo is SR  then RSA is LH", engine_));
      ruleBlock_->addRule(fl::Rule::parse("if Ep is FAR and Eo is RR  then RSA is LL", engine_));
      ruleBlock_->addRule(fl::Rule::parse("if Ep is FAR and Eo is RBR then RSA is Z", engine_));
      ruleBlock_->addRule(fl::Rule::parse("if Ep is CLOSE and Direction is FW and Efo is SL then RSA is RH", engine_));
      ruleBlock_->addRule(fl::Rule::parse("if Ep is CLOSE and Direction is FW and Efo is FL then RSA is RL", engine_));
      ruleBlock_->addRule(fl::Rule::parse("if Ep is CLOSE and Direction is FW and Efo is FA then RSA is Z", engine_));
      ruleBlock_->addRule(fl::Rule::parse("if Ep is CLOSE and Direction is FW and Efo is FR then RSA is LL", engine_));
      ruleBlock_->addRule(fl::Rule::parse("if Ep is CLOSE and Direction is FW and Efo is SR then RSA is LH", engine_));
      ruleBlock_->addRule(fl::Rule::parse("if Ep is CLOSE and Direction is BW and Efo is SL then RSA is LH", engine_));
      ruleBlock_->addRule(fl::Rule::parse("if Ep is CLOSE and Direction is BW and Efo is FL then RSA is LL", engine_));
      ruleBlock_->addRule(fl::Rule::parse("if Ep is CLOSE and Direction is BW and Efo is FR then RSA is RH", engine_));
      ruleBlock_->addRule(fl::Rule::parse("if Ep is CLOSE and Direction is BW and Efo is SR then RSA is RL", engine_));
    }
    else if (rearSteeringMode_ == "hybrid")
    {
      ruleBlock_->addRule(fl::Rule::parse("if Efo is not FA then RSA is Z", engine_));

      ruleBlock_->addRule(fl::Rule::parse("if Direction is FW and Efo is FA and Ey is BP then RSA is Z", engine_));
      ruleBlock_->addRule(fl::Rule::parse("if Direction is FW and Efo is FA and Ey is SP then RSA is RH", engine_));
      ruleBlock_->addRule(fl::Rule::parse("if Direction is FW and Efo is FA and Ey is Z then RSA is Z", engine_));
      ruleBlock_->addRule(fl::Rule::parse("if Direction is FW and Efo is FA and Ey is SN then RSA is LH", engine_));
      ruleBlock_->addRule(fl::Rule::parse("if Direction is FW and Efo is FA and Ey is BN then RSA is Z", engine_));

      ruleBlock_->addRule(fl::Rule::parse("if Direction is BW and Efo is FA and Ey is BP then RSA is Z", engine_));
      ruleBlock_->addRule(fl::Rule::parse("if Direction is BW and Efo is FA and Ey is SP then RSA is RH", engine_));
      ruleBlock_->addRule(fl::Rule::parse("if Direction is BW and Efo is FA and Ey is Z then RSA is Z", engine_));
      ruleBlock_->addRule(fl::Rule::parse("if Direction is BW and Efo is FA and Ey is SN then RSA is LH", engine_));
      ruleBlock_->addRule(fl::Rule::parse("if Direction is BW and Efo is FA and Ey is BN then RSA is Z", engine_));
    }
    else if (rearSteeringMode_ == "none")
    {
      ruleBlock_->addRule(fl::Rule::parse("if Direction is FW then RSA is Z", engine_));
      ruleBlock_->addRule(fl::Rule::parse("if Direction is BW then RSA is Z", engine_));
    }
    else
    {
      ROS_FATAL("Rear Steering Mode [%s] Unrecognized!", rearSteeringMode_.c_str());
      exit(EXIT_FAILURE);
    }

    // speed rules
    ruleBlock_->addRule(fl::Rule::parse("if Ep is FAR then Speed is FAST", engine_));
    ruleBlock_->addRule(fl::Rule::parse("if Ep is very CLOSE then Speed is SLOW", engine_));

    engine_->addRuleBlock(ruleBlock_);
    engine_->configure("Minimum", "Maximum", "Minimum", "Maximum", "WeightedAverage");

    std::string status;
    if (!engine_->isReady(&status))
        throw fl::Exception("Engine not ready. "
          "The following errors were encountered:\n" + status, FL_AT);
  }


  void CarLikeFuzzyPTC::computeVelocityCommands(
    const std::vector<geometry_msgs::PoseStamped>& path,
    geometry_msgs::Twist& cmd)
  {
    unsigned int subGoalIdx = findSubGoal(path);

    double eo = calcOrientationError(path, subGoalIdx);
    double efo = calcFinalOrientationError(path, subGoalIdx);
    double ep = calcPositionError(path, subGoalIdx);
    double ey = calcLateralDeviationError(path, subGoalIdx);
    int drcn = (rad2deg(fabs(eo)) < 120) ? 1 : -1;

    ROS_INFO_COND(displayControllerIO_, "Eo: %f, Efo: %f, Ep: %f, Ey: %f",
      rad2deg(eo), rad2deg(efo), ep, ey);

    if (isGoalReached(path))
    {
      cmd = *new geometry_msgs::Twist;  // return cleared twist
      return;
    }

    orientationError_->setInputValue(rad2deg(eo));
    finalOrientationError_->setInputValue(rad2deg(efo));
    positionError_->setInputValue(ep);
    lateralDeviationError_->setInputValue(ey);
    direction_->setInputValue(drcn);

    engine_->process();

    double vel = drcn * speed_->getOutputValue();  // linear velocity
    double fsa = deg2rad(frontSteeringAngle_->getOutputValue());  // front steering angle

    double rsa;  // rear steering angle
    if (rearSteeringMode_ == "crab"
      || rearSteeringMode_ == "counter"
      || rearSteeringMode_ == "none")
    {
      rsa = deg2rad(rearSteeringAngle_->getOutputValue());
    }
    else if (rearSteeringMode_ == "hybrid")
    {
      rsa = - (fsa + deg2rad(rearSteeringAngle_->getOutputValue()));
    }
    else
    { // it should never come here
      ROS_FATAL("Invalid Steering Mode. Exiting...");
      exit(EXIT_FAILURE);
    }

    double beta = atan((tan(fsa) + tan(rsa)) / 2);  // angle between vel.x and vel.y

    // create command
    cmd.linear.x = vel * sqrt(1 / (1 + pow(tan(beta), 2)));
    cmd.linear.y = vel * tan(beta) * sqrt(1 / (1 + pow(tan(beta), 2)));
    cmd.angular.z = vel * cos(beta) * (tan(fsa) - tan(rsa)) / wheelbase_;

    ROS_INFO_COND(displayControllerIO_,
      "vx: %.3f, vy: %.3f, w: %.3f, fsa: %.3f, rsa: %.3f",
      cmd.linear.x, cmd.linear.y, cmd.angular.z, rad2deg(fsa), rad2deg(rsa));

    if (isnan(cmd.linear.x) or isnan(cmd.linear.y))
    {
      ROS_ERROR("Speed=Nan. Something went wrong!");
      cmd.linear.x = 0.0;
      cmd.linear.y = 0.0;
    }
    if (isnan(cmd.angular.z))
    {
      ROS_ERROR("RotVel=Nan. Something went wrong!");
      cmd.angular.z = 0.0;
    }

    if (stop_)
    {
      cmd.linear.x = 0.0;
      cmd.linear.y = 0.0;
      cmd.angular.z = 0.0;
    }
  }


  bool CarLikeFuzzyPTC::isGoalReached(
    const std::vector<geometry_msgs::PoseStamped>& path)
  {
    bool positionReached =
      calcPositionError(path, path.size()-1) < xyGoalTolerance_;
    bool orientationReached =
      fabs(calcFinalOrientationError(path, path.size()-1)) < yawGoalTolerance_;
    return positionReached && orientationReached;
  }


  double CarLikeFuzzyPTC::calcOrientationError(
    const std::vector<geometry_msgs::PoseStamped>& path,
    unsigned int subGoalIdx)
  {
    return atan2(path[subGoalIdx].pose.position.y,
      path[subGoalIdx].pose.position.x);
  }


  double CarLikeFuzzyPTC::calcFinalOrientationError(
    const std::vector<geometry_msgs::PoseStamped>& path,
    unsigned int subGoalIdx)
  {
    return tf::getYaw(path[subGoalIdx].pose.orientation);
  }


  double CarLikeFuzzyPTC::calcPositionError(
    const std::vector<geometry_msgs::PoseStamped>& path,
    unsigned int subGoalIdx)
  {
    return hypot(path[subGoalIdx].pose.position.x, path[subGoalIdx].pose.position.y);
  }


  double CarLikeFuzzyPTC::calcLongitudinalDistanceFromGoal(
    const std::vector<geometry_msgs::PoseStamped>& path,
    unsigned int subGoalIdx)
  {
    return path[subGoalIdx].pose.position.x;
  }


  double CarLikeFuzzyPTC::calcLateralDeviationError(
    const std::vector<geometry_msgs::PoseStamped>& path,
    unsigned int subGoalIdx)
  {
    return calcPositionError(path, subGoalIdx)
      * sin(calcOrientationError(path, subGoalIdx));
  }


  double CarLikeFuzzyPTC::calcDistance(
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


  unsigned int CarLikeFuzzyPTC::findSubGoal(
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


  bool CarLikeFuzzyPTC::isCuspPoint(
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

    return !isnan(angle) && (angle < 1.0) && (angle > 0.17);
  }

}

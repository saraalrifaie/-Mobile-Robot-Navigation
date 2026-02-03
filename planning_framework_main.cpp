#include <iostream>
#include <vector>
#include <boost/bind.hpp>
#include <memory>
#include <cmath>
#include <algorithm>  

//  OMPL
#include <ompl/base/MotionValidator.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/service.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// ROS2 messages
#include <std_srvs/srv/empty.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_msgs/msg/path.hpp>

// Planner
#include <state_validity_checker_octomap_fcl_R2.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;

//! Planning Framework class.
/*!
 * Setup a sampling-based planner for computation of collision-free paths.
 * C-Space: R2
 * Workspace is represented with Octomaps
 */
class PlannFramework : public rclcpp::Node
{
public:
  //! Constructor
  PlannFramework();

  //! Planner setup
  void run();

  //! Periodic callback to solve the query.
  void planningCallback();

  //! Callback for getting current vehicle odometry
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);

  //! Callback for getting the 2D navigation goal
  void queryGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr nav_goal_msg);

  //! Procedure to visualize the resulting path
  void visualizeRRT(og::PathGeometric &geopath);

private:
  
  // ROS2
  
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr nav_goal_sub_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr solution_path_rviz_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr solution_path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_motion_pub_;

  // ROS TF
  tf2::Transform last_robot_pose_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // OMPL planner
  og::SimpleSetupPtr simple_setup_;
  double timer_period_, solving_time_, goal_tolerance_, yaw_goal_tolerance_, robot_base_radius_;
  bool odom_available_, goal_available_, visualize_tree_;

  std::vector<double> planning_bounds_x_, planning_bounds_y_;
  std::vector<double> start_state_, goal_map_frame_, goal_odom_frame_;
  double goal_radius_;

  std::string planner_name_;
  std::string odometry_topic_, query_goal_topic_, solution_path_topic_;
  std::string world_frame_, octomap_service_;
  std::vector<const ob::State *> solution_path_states_;

  rclcpp::Client<GetOctomap>::SharedPtr octomap_client_;

  // ------------------------------------------------------------------
  // Helpers (your additions)
  // ------------------------------------------------------------------
  bool isStoredPathValid(const ob::StateValidityCheckerPtr &checker);
  nav_msgs::msg::Path buildPathMsgFromStoredStates(const ob::StateSpacePtr &space);
  std::vector<geometry_msgs::msg::PoseStamped>
  movingAverage(const std::vector<geometry_msgs::msg::PoseStamped> &poses, int window_size = 4);
};

//! Constructor.
/*!
 * Load planner parameters from configuration file.
 * Publishers to visualize the resulting path.
 */
PlannFramework::PlannFramework()
  : Node("plann_framework")
{
  //=======================================================================
  // ! Get parameters
  //=======================================================================
  planning_bounds_x_.resize(2);
  planning_bounds_y_.resize(2);
  start_state_.resize(2);
  goal_map_frame_.resize(3);
  goal_odom_frame_.resize(3);

  this->declare_parameter<std::string>("world_frame", "map");
  this->declare_parameter<std::vector<double>>("planning_bounds_x", {0.0, 10.0});
  this->declare_parameter<std::vector<double>>("planning_bounds_y", {0.0, 10.0});
  this->declare_parameter<std::vector<double>>("start_state", {0.0, 0.0});
  this->declare_parameter<std::vector<double>>("goal_state", {5.0, 5.0, 0.0});
  this->declare_parameter<double>("timer_period", 1.0);
  this->declare_parameter<double>("solving_time", 1.0);
  this->declare_parameter<std::string>("planner_name", "RRTstar");
  this->declare_parameter<std::string>("odometry_topic", "/odom");
  this->declare_parameter<std::string>("query_goal_topic", "/move_base_simple/goal");
  this->declare_parameter<std::string>("solution_path_topic", "/path");
  this->declare_parameter<double>("goal_tolerance", 0.4);  
  this->declare_parameter<double>("yaw_goal_tolerance", 0.1);
  this->declare_parameter<bool>("visualize_tree", false);
  this->declare_parameter<double>("robot_base_radius", 0.5);
  this->declare_parameter<std::string>("octomap_service", "/octomap_full");

  this->get_parameter("world_frame", world_frame_);
  this->get_parameter("planning_bounds_x", planning_bounds_x_);
  this->get_parameter("planning_bounds_y", planning_bounds_y_);
  this->get_parameter("start_state", start_state_);
  this->get_parameter("goal_state", goal_map_frame_);
  this->get_parameter("timer_period", timer_period_);
  this->get_parameter("solving_time", solving_time_);
  this->get_parameter("planner_name", planner_name_);
  this->get_parameter("odometry_topic", odometry_topic_);
  this->get_parameter("query_goal_topic", query_goal_topic_);
  this->get_parameter("solution_path_topic", solution_path_topic_);
  this->get_parameter("goal_tolerance", goal_tolerance_);
  this->get_parameter("yaw_goal_tolerance", yaw_goal_tolerance_);
  this->get_parameter("visualize_tree", visualize_tree_);
  this->get_parameter("robot_base_radius", robot_base_radius_);
  this->get_parameter("octomap_service", octomap_service_);

  goal_radius_ = goal_tolerance_;
  goal_available_ = false;

  //=======================================================================
  // ! Subscribers
  //=======================================================================
  // Odometry data
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_, 10,
      std::bind(&PlannFramework::odomCallback, this, std::placeholders::_1));
  odom_available_ = false;

  // 2D Nav Goal
  nav_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      query_goal_topic_, 10,
      std::bind(&PlannFramework::queryGoalCallback, this, std::placeholders::_1));

  //=======================================================================
  // ! Publishers
  //=======================================================================
  solution_path_rviz_pub_ =
      this->create_publisher<visualization_msgs::msg::Marker>("solution_path", 10);

  solution_path_pub_ =
      this->create_publisher<nav_msgs::msg::Path>(solution_path_topic_, 10);

  goal_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_controller", 10);

  stop_motion_pub_ =
      this->create_publisher<std_msgs::msg::Bool>("/stop_motion", 10);

  //=======================================================================
  // ! TF
  //=======================================================================
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  //=======================================================================
  // Waiting for odometry
  //=======================================================================
  rclcpp::Rate loop_rate(10);
  while (rclcpp::ok() && !odom_available_) {
    rclcpp::spin_some(this->get_node_base_interface());
    loop_rate.sleep();
    RCLCPP_WARN(this->get_logger(), "Waiting for vehicle's odometry");
  }
  RCLCPP_WARN(this->get_logger(), "Odometry received");

  octomap_client_ = this->create_client<GetOctomap>(octomap_service_);
  while (!octomap_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(),
                "Waiting for the Octomap service to become available...");
  }
}

// Odometry callback

void PlannFramework::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
  if (!odom_available_)
    odom_available_ = true;

  geometry_msgs::msg::Pose predictedPose = odom_msg->pose.pose;
  predictedPose.position.x = odom_msg->pose.pose.position.x;
  predictedPose.position.y = odom_msg->pose.pose.position.y;
  tf2::fromMsg(predictedPose, last_robot_pose_);

  double useless_pitch, useless_roll, yaw;
  tf2::Matrix3x3(last_robot_pose_.getRotation())
      .getEulerYPR(yaw, useless_pitch, useless_roll);

  if (goal_available_ &&
      std::sqrt(std::pow(goal_odom_frame_[0] - last_robot_pose_.getOrigin().getX(), 2.0) +
                std::pow(goal_odom_frame_[1] - last_robot_pose_.getOrigin().getY(), 2.0)) <
          (goal_radius_ + 0.6) &&
      std::abs(yaw - goal_odom_frame_[2]) <
          (yaw_goal_tolerance_ + 0.20)) {

    goal_available_ = false;
    // when goal is reached, forget stored path
    solution_path_states_.clear();
  }
}

void PlannFramework::queryGoalCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr nav_goal_msg)
{
  double useless_pitch, useless_roll, yaw;
  tf2::Quaternion q(
      nav_goal_msg->pose.orientation.x,
      nav_goal_msg->pose.orientation.y,
      nav_goal_msg->pose.orientation.z,
      nav_goal_msg->pose.orientation.w);

  tf2::Matrix3x3(q).getRPY(useless_roll, useless_pitch, yaw);

  goal_map_frame_[0] = nav_goal_msg->pose.position.x;
  goal_map_frame_[1] = nav_goal_msg->pose.position.y;
  goal_map_frame_[2] = yaw;

  
  geometry_msgs::msg::TransformStamped tf_map_to_fixed;
  try {
    tf_map_to_fixed = tf_buffer_->lookupTransform(
        "map", "odom", tf2::TimePointZero);
  }
  catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(),
                "Could not transform map to odom: %s", ex.what());
    return;
  }

  tf2::Transform tf2_map_to_fixed;
  tf2::fromMsg(tf_map_to_fixed.transform, tf2_map_to_fixed);
  tf2::Matrix3x3(tf2_map_to_fixed.getRotation())
      .getRPY(useless_roll, useless_pitch, yaw);

  tf2::Vector3 goal_point_odom_frame(
      goal_map_frame_[0], goal_map_frame_[1], 0.0);

  goal_point_odom_frame =
      tf2_map_to_fixed.inverse() * goal_point_odom_frame;

  goal_odom_frame_[0] = goal_point_odom_frame.x();
  goal_odom_frame_[1] = goal_point_odom_frame.y();
  goal_odom_frame_[2] = goal_map_frame_[2] - yaw;

  
  solution_path_states_.clear();
  goal_available_ = true;

  goal_pub_->publish(*nav_goal_msg);
}

// Planner setup.

void PlannFramework::run()
{
 
  rclcpp::Rate loop_rate(1 / (timer_period_ - solving_time_));

  while (rclcpp::ok()) {
    if (goal_available_) {
     
      planningCallback();
    }
    rclcpp::spin_some(this->get_node_base_interface());
    loop_rate.sleep();
  }
}


bool PlannFramework::isStoredPathValid(
    const ob::StateValidityCheckerPtr &checker)
{
  if (solution_path_states_.empty())
    return false;

  for (const auto *state : solution_path_states_) {
    if (!checker->isValid(state)) {
      RCLCPP_WARN(this->get_logger(),
                  "Stored path became invalid with latest map.");
      return false;
    }
  }
  return true;
}


nav_msgs::msg::Path PlannFramework::buildPathMsgFromStoredStates(
    const ob::StateSpacePtr &space)
{
  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = world_frame_;
  path_msg.header.stamp = this->now();

  for (const auto *s : solution_path_states_) {
    auto *state_r2 = s->as<ob::RealVectorStateSpace::StateType>();

    geometry_msgs::msg::PoseStamped p;
    p.header = path_msg.header;
    p.pose.position.x = state_r2->values[0];
    p.pose.position.y = state_r2->values[1];
    p.pose.position.z = 0.0;

    path_msg.poses.push_back(p);
  }

  return path_msg;
}


std::vector<geometry_msgs::msg::PoseStamped>
PlannFramework::movingAverage(
    const std::vector<geometry_msgs::msg::PoseStamped> &poses,
    int window_size)
{
  if (poses.size() <= 2 || window_size < 2)
    return poses;

  std::vector<geometry_msgs::msg::PoseStamped> smoothed;
  smoothed.reserve(poses.size());

  // keep first as is
  smoothed.push_back(poses.front());

  for (size_t i = 1; i + 1 < poses.size(); ++i) {
    int half = window_size / 2;
    int start = std::max<int>(1, static_cast<int>(i) - half);
    int end   = std::min<int>(
        static_cast<int>(poses.size()) - 2,
        static_cast<int>(i) + half);

    double sum_x = 0.0, sum_y = 0.0;
    int count = 0;

    for (int j = start; j <= end; ++j) {
      sum_x += poses[j].pose.position.x;
      sum_y += poses[j].pose.position.y;
      ++count;
    }

    geometry_msgs::msg::PoseStamped p = poses[i];
    p.pose.position.x = sum_x / static_cast<double>(count);
    p.pose.position.y = sum_y / static_cast<double>(count);
    smoothed.push_back(p);
  }

  // keep last as is
  smoothed.push_back(poses.back());
  return smoothed;
}

void PlannFramework::planningCallback()
{
  
  static rclcpp::Time last_plan_time = this->now();
  if ((this->now() - last_plan_time).seconds() < 1.0)
    return;
  last_plan_time = this->now();

  
  double useless_pitch, useless_roll, yaw;
  geometry_msgs::msg::TransformStamped tf_map_to_fixed;

  try {
    tf_map_to_fixed = tf_buffer_->lookupTransform(
        "map", "odom", tf2::TimePointZero);
  }
  catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(),
                "Could not transform map to odom: %s", ex.what());
    return;
  }

  tf2::Transform tf2_map_to_fixed;
  tf2::fromMsg(tf_map_to_fixed.transform, tf2_map_to_fixed);
  tf2::Matrix3x3(tf2_map_to_fixed.getRotation())
      .getRPY(useless_roll, useless_pitch, yaw);

  tf2::Vector3 goal_point_odom_frame(
      goal_map_frame_[0], goal_map_frame_[1], 0.0);

  goal_point_odom_frame =
      tf2_map_to_fixed.inverse() * goal_point_odom_frame;

  goal_odom_frame_[0] = goal_point_odom_frame.x();
  goal_odom_frame_[1] = goal_point_odom_frame.y();
  goal_odom_frame_[2] = goal_map_frame_[2] - yaw;

  
  ob::StateSpacePtr space(new ob::RealVectorStateSpace(2));

  
  ob::RealVectorBounds bounds(2);
  bounds.setLow(0, planning_bounds_x_[0]);
  bounds.setHigh(0, planning_bounds_x_[1]);
  bounds.setLow(1, planning_bounds_y_[0]);
  bounds.setHigh(1, planning_bounds_y_[1]);

  space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

 
  simple_setup_ = og::SimpleSetupPtr(new og::SimpleSetup(space));
  ob::SpaceInformationPtr si = simple_setup_->getSpaceInformation();

  
  start_state_[0] = last_robot_pose_.getOrigin().getX();
  start_state_[1] = last_robot_pose_.getOrigin().getY();

  ob::ScopedState<> start(space);
  start[0] = start_state_[0];
  start[1] = start_state_[1];

  ob::ScopedState<> goal(space);
  goal[0] = goal_map_frame_[0];
  goal[1] = goal_map_frame_[1];

  
  RCLCPP_DEBUG(this->get_logger(), "Requesting the map from %s...",
               octomap_service_.c_str());

  auto request = std::make_shared<GetOctomap::Request>();
  auto result = octomap_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(
          this->get_node_base_interface(), result) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to call service %s", octomap_service_.c_str());
    return;
  }

  auto octomap_msg = result.get()->map;

  
  ob::StateValidityCheckerPtr om_stat_val_check(
      new OmFclStateValidityCheckerR2(
          simple_setup_->getSpaceInformation(),
          planning_bounds_x_, planning_bounds_y_,
          robot_base_radius_, octomap_msg));

  simple_setup_->setStateValidityChecker(om_stat_val_check);
  simple_setup_->getStateSpace()->setValidSegmentCountFactor(15.0);

  
  if (!solution_path_states_.empty()) {
    if (isStoredPathValid(om_stat_val_check)) {
     
      std_msgs::msg::Bool stop_msg;
      stop_msg.data = false;
      stop_motion_pub_->publish(stop_msg);

      RCLCPP_INFO(this->get_logger(),
                  "Reusing previous OMPL path (still valid).");
      return;
    } else {
      // stop and replan
      std_msgs::msg::Bool stop_msg;
      stop_msg.data = true;
      stop_motion_pub_->publish(stop_msg);

      RCLCPP_WARN(this->get_logger(),
                  "Stored path invalid with latest map → replanning...");
    }
  }

  
  simple_setup_->getProblemDefinition()->setOptimizationObjective(
      ob::OptimizationObjectivePtr(
          new ob::PathLengthOptimizationObjective(si)));

  
  ob::PlannerPtr planner;
  if (planner_name_ == "RRT") {
    planner = ob::PlannerPtr(new og::RRT(si));
  } else if (planner_name_ == "PRMstar") {
    planner = ob::PlannerPtr(new og::PRMstar(si));
  } else if (planner_name_ == "PRM") {
    planner = ob::PlannerPtr(new og::PRM(si));
  } else {
    planner = ob::PlannerPtr(new og::RRTstar(si));  // default
  }

  
  if (planner_name_ == "RRT") {
    auto rrt = std::dynamic_pointer_cast<og::RRT>(planner);
    if (rrt) {
      rrt->setRange(0.7);
      rrt->setGoalBias(0.25);
    }
  }

  if (planner_name_ == "RRTstar") {
    auto rrtstar = std::dynamic_pointer_cast<og::RRTstar>(planner);
    if (rrtstar) {
      rrtstar->setRange(0.7);
      rrtstar->setGoalBias(0.25);
    }
  }

  simple_setup_->setPlanner(planner);
  simple_setup_->setStartState(start);
  simple_setup_->setGoalState(goal, goal_radius_);

 
  simple_setup_->setup();

  
  ob::PlannerStatus solved = simple_setup_->solve(solving_time_);

  if (!(solved && simple_setup_->haveExactSolutionPath())) {
    RCLCPP_INFO(this->get_logger(), "Path NOT found");
    std_msgs::msg::Bool stop_msg;
    stop_msg.data = true;
    stop_motion_pub_->publish(stop_msg);
    return;
  }

  // path found
  og::PathGeometric path = simple_setup_->getSolutionPath();
  path.interpolate(int(path.length() / 0.3));

  RCLCPP_INFO(this->get_logger(),
              "Path found with cost %f",
              path.cost(simple_setup_->getProblemDefinition()
                        ->getOptimizationObjective()).value());

  std::vector<ob::State *> path_states = path.getStates();

  double distance_to_goal =
      std::sqrt(std::pow(goal_odom_frame_[0] -
                         path_states.back()
                             ->as<ob::RealVectorStateSpace::StateType>()
                             ->values[0],
                         2.0) +
                std::pow(goal_odom_frame_[1] -
                         path_states.back()
                             ->as<ob::RealVectorStateSpace::StateType>()
                             ->values[1],
                         2.0));

  if (simple_setup_->haveExactSolutionPath() ||
      distance_to_goal <= goal_radius_) {

    visualizeRRT(path);

    nav_msgs::msg::Path solution_path_for_control;
    solution_path_for_control.header.frame_id = world_frame_;
    solution_path_for_control.header.stamp = this->now();

    for (size_t i = 0; i < path_states.size(); ++i) {
      geometry_msgs::msg::PoseStamped p;
      p.header = solution_path_for_control.header;

      auto *curr =
          path_states[i]->as<ob::RealVectorStateSpace::StateType>();
      p.pose.position.x = curr->values[0];
      p.pose.position.y = curr->values[1];
      p.pose.position.z = 0.0;

    
      if (i + 1 < path_states.size()) {
        auto *next =
            path_states[i + 1]->as<ob::RealVectorStateSpace::StateType>();

        double dx = next->values[0] - curr->values[0];
        double dy = next->values[1] - curr->values[1];
        double yaw_local = std::atan2(dy, dx);

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw_local);
        q.normalize();
        p.pose.orientation = tf2::toMsg(q);
      } else {
     
        tf2::Quaternion q;
        q.setRPY(0, 0, goal_odom_frame_[2]);
        q.normalize();
        p.pose.orientation = tf2::toMsg(q);
      }

      solution_path_for_control.poses.push_back(p);
    }

    solution_path_for_control.poses =
        movingAverage(solution_path_for_control.poses, 4);

 
    solution_path_pub_->publish(solution_path_for_control);

    // allow robot to move
    std_msgs::msg::Bool stop_msg;
    stop_msg.data = false;
    stop_motion_pub_->publish(stop_msg);


    solution_path_states_.clear();
    for (auto *s : path_states) {
      ob::State *copy = space->allocState();
      space->copyState(copy, s);
      solution_path_states_.push_back(copy);
    }
  }

  
  // Clear previous solution path 
  simple_setup_->clear();
}


// Resulting path visualization.
// Visualize resulting path.
 
void PlannFramework::visualizeRRT(og::PathGeometric &geopath)
{
  tf2::Quaternion orien_quat;
  visualization_msgs::msg::Marker visual_rrt, visual_result_path;

  visual_result_path.header.frame_id =
      visual_rrt.header.frame_id = world_frame_;
  visual_result_path.header.stamp =
      visual_rrt.header.stamp = this->now();

  visual_rrt.ns = "planner_rrt";
  visual_result_path.ns = "planner_result_path";

  visual_result_path.action =
      visual_rrt.action = visualization_msgs::msg::Marker::ADD;

  visual_result_path.pose.orientation.w =
      visual_rrt.pose.orientation.w = 1.0;

  visual_rrt.id = 0;
  visual_result_path.id = 1;

  visual_rrt.type =
      visual_result_path.type = visualization_msgs::msg::Marker::LINE_LIST;

  visual_rrt.scale.x = 0.03;
  visual_result_path.scale.x = 0.08;

  // Points are green
  visual_result_path.color.g = 1.0;
  visual_result_path.color.a = 1.0;

  // Line strip is blue
  visual_rrt.color.b = 1.0;
  visual_rrt.color.a = 1.0;

  const ob::RealVectorStateSpace::StateType *state_r2;
  geometry_msgs::msg::Point p;

  ob::PlannerData planner_data(simple_setup_->getSpaceInformation());
  simple_setup_->getPlannerData(planner_data);

  std::vector<unsigned int> edgeList;

  RCLCPP_DEBUG(this->get_logger(),
               "number of states in the tree: %d",
               planner_data.numVertices());

  if (visualize_tree_) {
    for (unsigned int i = 1; i < planner_data.numVertices(); ++i) {
      if (planner_data.getVertex(i).getState() &&
          planner_data.getIncomingEdges(i, edgeList) > 0) {

        state_r2 = planner_data.getVertex(i)
                       .getState()
                       ->as<ob::RealVectorStateSpace::StateType>();
        p.x = state_r2->values[0];
        p.y = state_r2->values[1];
        p.z = 0.1;
        visual_rrt.points.push_back(p);

        state_r2 = planner_data.getVertex(edgeList[0])
                       .getState()
                       ->as<ob::RealVectorStateSpace::StateType>();
        p.x = state_r2->values[0];
        p.y = state_r2->values[1];
        p.z = 0.1;
        visual_rrt.points.push_back(p);
      }
    }
    solution_path_rviz_pub_->publish(visual_rrt);
  }

  std::vector<ob::State *> states = geopath.getStates();

  for (uint32_t i = 0; i < geopath.getStateCount(); ++i) {
    state_r2 = states[i]->as<ob::RealVectorStateSpace::StateType>();
    p.x = state_r2->values[0];
    p.y = state_r2->values[1];
    p.z = 0.1;

    if (i > 0) {
      visual_result_path.points.push_back(p);

      state_r2 = states[i - 1]
                     ->as<ob::RealVectorStateSpace::StateType>();
      p.x = state_r2->values[0];
      p.y = state_r2->values[1];
      p.z = 0.1;

      visual_result_path.points.push_back(p);
    }
  }

  solution_path_rviz_pub_->publish(visual_result_path);
}
//! Main function
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto planning_framework = std::make_shared<PlannFramework>();
  planning_framework->run();
  return 0;
}

/*! \file state_validity_checker_octomap_fcl_R2.hpp
* \brief State validity checker.
*
* \date August 26, 2025
* \author Juan David Hernandez Vega, HernandezVegaJ@cardiff.ac.uk
* \author Steven Silva, HernandezVegaJ@cardiff.ac.uk
*
* \details Check if a given configuration R2 is collision-free.
* The workspace is represented by an Octomap and collision check is done with FCL.
*
* Based on Juan D. Hernandez Vega's PhD thesis, University of Girona
* http://hdl.handle.net/10803/457592, http://www.tdx.cat/handle/10803/457592
*/

#ifndef OMPL_CONTRIB_STATE_VALIDITY_CHECKER_FCL_OCTOMAP_R2_
#define OMPL_CONTRIB_STATE_VALIDITY_CHECKER_FCL_OCTOMAP_R2_

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// Standard libraries
#include <cstdlib>
#include <cmath>
#include <string>
#include <iostream>

// Octomap
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/srv/get_octomap.hpp>

// OMPL
#include <ompl/config.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/tools/debug/Profiler.h>

// Boost
#include <boost/pointer_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>

// Eigen
#include <Eigen/Dense>

// FCL
#include <fcl/fcl.h>
#include <fcl/geometry/octree/octree.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/distance.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/broadphase/default_broadphase_callbacks.h>
#include <fcl/broadphase/broadphase_spatialhash.h>
#include <fcl/common/types.h>
#include <fcl/config.h>
#include <fcl/geometry/shape/cylinder.h>
#include <fcl/math/geometry-inl.h>
#include <fcl/narrowphase/collision_request.h>
#include <fcl/narrowphase/collision_result.h>

// ROS-Octomap interface
using octomap_msgs::srv::GetOctomap;

// Standard namespace
using namespace std;

// Octomap namespace
using namespace octomap;

// OMPL namespaces
namespace ob = ompl::base;
namespace og = ompl::geometric;

//! OmFclStateValidityCheckerR2 class.
/*!
Octomap State Validity checker.
Extension of an abstract class used to implement the state validity checker over an octomap using FCL.
*/
class OmFclStateValidityCheckerR2 : public ob::StateValidityChecker
{
public:
//! OmFclStateValidityCheckerR2 constructor.
/*!
* Besides initializing the private attributes, it loads the octomap.
*/
OmFclStateValidityCheckerR2(const ob::SpaceInformationPtr &si,
std::vector<double> planning_bounds_x, std::vector<double> planning_bounds_y, const double robot_radius, octomap_msgs::msg::Octomap octomap_msg);

//! OmFclStateValidityCheckerR2 destructor.
/*!
* Destroy the octomap.
*/
~OmFclStateValidityCheckerR2();

//! State validator.
/*!
* Function that verifies if the given state is valid (i.e. is free of collision) using FCL.
*/
virtual bool isValid(const ob::State *state) const;

virtual bool isValidPoint(const ob::State *state) const;

private:
// Octomap
octomap::AbstractOcTree *abs_octree_;
octomap::OcTree *octree_;
double octree_min_x_, octree_min_y_, octree_min_z_;
double octree_max_x_, octree_max_y_, octree_max_z_;
std::vector<double> planning_bounds_x_, planning_bounds_y_;
double robot_base_radius_, robot_base_height_;

double octree_res_;

// FCL
std::shared_ptr<fcl::OcTreef> tree_fcl_;
std::shared_ptr<fcl::CollisionObjectf> tree_obj_;
std::shared_ptr<fcl::Cylinderf> robot_collision_solid_;
};

#endif


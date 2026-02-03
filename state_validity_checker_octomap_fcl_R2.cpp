/*! \file state_validity_checker_octomap_fcl_R2.cpp
 * \brief State validity checker.
 *
 * \date March 5, 2015
 * \author Juan David Hernandez Vega, juandhv@rice.edu
 *
 * \details Check if a given configuration R2 is collision-free.
 *  The workspace is represented by an Octomap and collision check is done with FCL.
 *
 * Based on Juan D. Hernandez Vega's PhD thesis, University of Girona
 * http://hdl.handle.net/10803/457592, http://www.tdx.cat/handle/10803/457592
 */

#include "state_validity_checker_octomap_fcl_R2.hpp"

OmFclStateValidityCheckerR2::OmFclStateValidityCheckerR2(
    const ob::SpaceInformationPtr &si,
    std::vector<double> planning_bounds_x,
    std::vector<double> planning_bounds_y,
    const double robot_radius,
    octomap_msgs::msg::Octomap octomap_msg)
    : ob::StateValidityChecker(si),
      robot_base_radius_(robot_radius),
      robot_base_height_(0.1)
{
   
    planning_bounds_x_ = planning_bounds_x;
    planning_bounds_y_ = planning_bounds_y;

 
    abs_octree_ = octomap_msgs::msgToMap(octomap_msg);
    octree_ = dynamic_cast<octomap::OcTree *>(abs_octree_);

 
    if (!octree_) {
        RCLCPP_WARN(rclcpp::get_logger("FCL_Checker"),
                    "Octree not loaded – assuming free space.");
        tree_fcl_ = nullptr;
        tree_obj_ = nullptr;
        return;
    }

    
    auto octree_shared =
        std::shared_ptr<const octomap::OcTree>(octree_, [](const octomap::OcTree *) {});
    tree_fcl_ = std::make_shared<fcl::OcTreef>(octree_shared);
    tree_obj_ = std::make_shared<fcl::CollisionObjectf>(tree_fcl_);

 
    robot_collision_solid_ =
        std::make_shared<fcl::Cylinderf>(robot_base_radius_, robot_base_height_);


    octree_res_ = octree_->getResolution();
    octree_->getMetricMin(octree_min_x_, octree_min_y_, octree_min_z_);
    octree_->getMetricMax(octree_max_x_, octree_max_y_, octree_max_z_);

    RCLCPP_INFO(rclcpp::get_logger("FCL_Checker"),
                "FCL validity checker initialized.");
}

bool OmFclStateValidityCheckerR2::isValid(const ob::State *state) const
{
    
    if (!octree_ || !tree_obj_ || !tree_fcl_)
        return true;

   
    const auto *s = state->as<ob::RealVectorStateSpace::StateType>();
    double x = s->values[0];
    double y = s->values[1];

    
    if (x < octree_min_x_ || y < octree_min_y_ ||
        x > octree_max_x_ || y > octree_max_y_)
        return true;

  
    octomap::point3d q(x, y, 0.25f);
    octomap::OcTreeNode *node = octree_->search(q);

    if (node && node->getOccupancy() > 0.5)
        return false;

    
    float inflated_radius = robot_base_radius_ + 0.05f;
    auto inflated_geom =
        std::make_shared<fcl::Cylinderf>(inflated_radius, 0.10f);
     
        //FCL

    fcl::Transform3f tf;
    tf.setIdentity();
    tf.translate(fcl::Vector3f(x, y, 0.15f));

    fcl::CollisionObjectf robot_obj(inflated_geom, tf);

    fcl::CollisionRequestf req;
    fcl::CollisionResultf res;

    
    fcl::collide(tree_obj_.get(), &robot_obj, req, res);

    return !res.isCollision();
}

bool OmFclStateValidityCheckerR2::isValidPoint(const ob::State *state) const
{
    // octree not loaded → assume valid
    if (!octree_)
        return true;

    // extract the component of the state and cast it to what we expect
    const auto *s = state->as<ob::RealVectorStateSpace::StateType>();

    octomap::point3d q(s->values[0], s->values[1], 0.0);

    auto *node = octree_->search(q);
    if (!node)
        return true;

    return node->getOccupancy() < 0.5;
}

OmFclStateValidityCheckerR2::~OmFclStateValidityCheckerR2()
{
    // free FCL objects
    tree_fcl_.reset();
    tree_obj_.reset();
}

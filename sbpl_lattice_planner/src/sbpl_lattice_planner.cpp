/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
* Author: Mike Phillips
*********************************************************************/

#include <sbpl_lattice_planner/sbpl_lattice_planner.h>
#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Path.h>
#include <sbpl_lattice_planner/SBPLLatticePlannerStats.h>
#include <math.h>

using namespace std;
using namespace ros;


PLUGINLIB_DECLARE_CLASS(sbpl_latice_planner, SBPLLatticePlanner, sbpl_lattice_planner::SBPLLatticePlanner, nav_core::BaseGlobalPlanner);

namespace sbpl_lattice_planner{

class LatticeSCQ : public StateChangeQuery{
  public:
    LatticeSCQ(EnvironmentNAVXYTHETALAT* env, std::vector<nav2dcell_t> const & changedcellsV)
      : env_(env), changedcellsV_(changedcellsV) {
    }

    // lazy init, because we do not always end up calling this method
    virtual std::vector<int> const * getPredecessors() const{
      if(predsOfChangedCells_.empty() && !changedcellsV_.empty())
        env_->GetPredsofChangedEdges(&changedcellsV_, &predsOfChangedCells_);
      return &predsOfChangedCells_;
    }

    // lazy init, because we do not always end up calling this method
    virtual std::vector<int> const * getSuccessors() const{
      if(succsOfChangedCells_.empty() && !changedcellsV_.empty())
        env_->GetSuccsofChangedEdges(&changedcellsV_, &succsOfChangedCells_);
      return &succsOfChangedCells_;
    }

    EnvironmentNAVXYTHETALAT * env_;
    std::vector<nav2dcell_t> const & changedcellsV_;
    mutable std::vector<int> predsOfChangedCells_;
    mutable std::vector<int> succsOfChangedCells_;
};

SBPLLatticePlanner::SBPLLatticePlanner()
  : initialized_(false), costmap_ros_(NULL){
}

SBPLLatticePlanner::SBPLLatticePlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : initialized_(false), costmap_ros_(NULL){
  initialize(name, costmap_ros);
}


void SBPLLatticePlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
  if(!initialized_){
    ros::NodeHandle private_nh("~/"+name);
    ros::NodeHandle nh(name);

    ROS_INFO("Name is %s", name.c_str());

    private_nh.param("planner_type", planner_type_, string("ARAPlanner"));
    private_nh.param("allocated_time", allocated_time_, 10.0);
    private_nh.param("initial_epsilon",initial_epsilon_,3.0);
    private_nh.param("environment_type", environment_type_, string("XYThetaLattice"));
    private_nh.param("forward_search", forward_search_, bool(false));
    private_nh.param("primitive_filename",primitive_filename_,string(""));
    ROS_INFO_STREAM("primitive_filename: " << primitive_filename_);
    private_nh.param("force_scratch_limit",force_scratch_limit_,500);

    double nominalvel_mpersecs, timetoturn45degsinplace_secs;
    private_nh.param("nominalvel_mpersecs", nominalvel_mpersecs, 0.4);
    private_nh.param("timetoturn45degsinplace_secs", timetoturn45degsinplace_secs, 0.6);
    private_nh.param("min_euclidian_distance", min_euclidian_distance_, 0.2);
    private_nh.param("goal_tolerance", goal_tolerance_, 0.0);

    private_nh.param("use_orthogonal_goals", use_orthogonal_goals_, false);
    private_nh.param("orthogonal_step_size", orthogonal_step_size_, 0.1);
    private_nh.param("max_orthogonal_distance", max_orthogonal_distance_, 2.0);

    int lethal_obstacle;
    private_nh.param("lethal_obstacle",lethal_obstacle,20);
    lethal_obstacle_ = (unsigned char) lethal_obstacle;
    inscribed_inflated_obstacle_ = lethal_obstacle_-1;
    sbpl_cost_multiplier_ = (unsigned char) (costmap_2d::INSCRIBED_INFLATED_OBSTACLE/inscribed_inflated_obstacle_ + 1);
    ROS_DEBUG("SBPL: lethal: %uz, inscribed inflated: %uz, multiplier: %uz",lethal_obstacle,inscribed_inflated_obstacle_,sbpl_cost_multiplier_);

    costmap_ros_ = costmap_ros;
    clearRobotFootprint();

    std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();

    if ("XYThetaLattice" == environment_type_){
      ROS_DEBUG("Using a 3D costmap for theta lattice\n");
      env_ = new EnvironmentNAVXYTHETALAT();
    }
    else{
      ROS_ERROR("XYThetaLattice is currently the only supported environment!\n");
      exit(1);
    }

    if(!env_->SetEnvParameter("cost_inscribed_thresh",costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE))){
      ROS_ERROR("Failed to set cost_inscribed_thresh parameter");
      exit(1);
    }
    if(!env_->SetEnvParameter("cost_possibly_circumscribed_thresh", costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE))){
      ROS_ERROR("Failed to set cost_possibly_circumscribed_thresh parameter");
      exit(1);
    }
    int obst_cost_thresh = costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE);
    vector<sbpl_2Dpt_t> perimeterptsV;
    perimeterptsV.reserve(footprint.size());
    for (size_t ii(0); ii < footprint.size(); ++ii) {
      sbpl_2Dpt_t pt;
      pt.x = footprint[ii].x;
      pt.y = footprint[ii].y;
      perimeterptsV.push_back(pt);
    }

    bool ret;
    try{
      ret = env_->InitializeEnv(costmap_ros_->getCostmap()->getSizeInCellsX(), // width
                                costmap_ros_->getCostmap()->getSizeInCellsY(), // height
                                NULL, // mapdata
                                0, 0, 0, // start (x, y, theta, t)
                                0, 0, 0, // goal (x, y, theta)
                                0, 0, 0, //goal tolerance
                                perimeterptsV, costmap_ros_->getCostmap()->getResolution(), nominalvel_mpersecs,
                                timetoturn45degsinplace_secs, obst_cost_thresh,
                                primitive_filename_.c_str());
    }
    catch(SBPL_Exception e){
      ROS_ERROR_STREAM("SBPL encountered a fatal exception!" << e.what());
      ret = false;
    }
    if(!ret){
      ROS_ERROR("SBPL initialization failed!");
      exit(1);
    }
    for (ssize_t ix(0); ix < costmap_ros_->getCostmap()->getSizeInCellsX(); ++ix)
      for (ssize_t iy(0); iy < costmap_ros_->getCostmap()->getSizeInCellsY(); ++iy)
        env_->UpdateCost(ix, iy, costMapCostToSBPLCost(costmap_ros_->getCostmap()->getCost(ix,iy)));

    if ("ARAPlanner" == planner_type_){
      ROS_INFO("Planning with ARA*");
      planner_ = new ARAPlanner(env_, forward_search_);
    }
    else if ("ADPlanner" == planner_type_){
      ROS_INFO("Planning with AD*");
      planner_ = new ADPlanner(env_, forward_search_);
    }
    else{
      ROS_ERROR("ARAPlanner and ADPlanner are currently the only supported planners!\n");
      exit(1);
    }

    ROS_INFO("[sbpl_lattice_planner] Initialized successfully");
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
    stats_publisher_ = private_nh.advertise<sbpl_lattice_planner::SBPLLatticePlannerStats>("sbpl_lattice_planner_stats", 1);

    initialized_ = true;
  }
}

//Taken from Sachin's sbpl_cart_planner
//This rescales the costmap according to a rosparam which sets the obstacle cost
unsigned char SBPLLatticePlanner::costMapCostToSBPLCost(unsigned char newcost){
  if(newcost == costmap_2d::LETHAL_OBSTACLE)
    return lethal_obstacle_;
  else if(newcost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    return inscribed_inflated_obstacle_;
  else if(newcost == 0 || newcost == costmap_2d::NO_INFORMATION)
    return 0;
  else
    return (unsigned char) (newcost/sbpl_cost_multiplier_ + 0.5);
}

void SBPLLatticePlanner::publishStats(int solution_cost, int solution_size,
                                      const geometry_msgs::PoseStamped& start,
                                      const geometry_msgs::PoseStamped& goal){
  // Fill up statistics and publish
  sbpl_lattice_planner::SBPLLatticePlannerStats stats;
  stats.initial_epsilon = initial_epsilon_;
  stats.plan_to_first_solution = false;
  stats.final_number_of_expands = planner_->get_n_expands();
  stats.allocated_time = allocated_time_;

  stats.time_to_first_solution = planner_->get_initial_eps_planning_time();
  stats.actual_time = planner_->get_final_eps_planning_time();
  stats.number_of_expands_initial_solution = planner_->get_n_expands_init_solution();
  stats.final_epsilon = planner_->get_final_epsilon();

  stats.solution_cost = solution_cost;
  stats.path_size = solution_size;
  stats.start = start;
  stats.goal = goal;
  stats_publisher_.publish(stats);
}

bool SBPLLatticePlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                 const geometry_msgs::PoseStamped& planner_goal,
                                 std::vector<geometry_msgs::PoseStamped>& plan){
  if(!initialized_){
    ROS_ERROR("Global planner is not initialized");
    return false;
  }

  plan.clear();

  ROS_DEBUG("[sbpl_lattice_planner] getting fresh copy of costmap");
  clearRobotFootprint();
  ROS_DEBUG("[sbpl_lattice_planner] robot footprint cleared");

  geometry_msgs::PoseStamped goal;
  goal = planner_goal;

  if (positionInCollision(goal.pose))
  {
    if (!use_orthogonal_goals_)
    {
      ROS_WARN("goal is in collision no plan can be found");
      return false;
    }

    ROS_INFO("goal is in collision use orthogonal goal");

    geometry_msgs::Pose new_goal_pose;
    if (findOrthogonalGoal(goal.pose, new_goal_pose))
    {
      goal.pose = new_goal_pose;
    }
    else
    {
      ROS_WARN("no orthogonal goals possible no plan can be found");
      return false;
    }
  }

  ROS_INFO("[sbpl_lattice_planner] getting start point (%g,%g) goal point (%g,%g)",
           start.pose.position.x, start.pose.position.y,goal.pose.position.x, goal.pose.position.y);
  double theta_start = 2 * atan2(start.pose.orientation.z, start.pose.orientation.w);
  double theta_goal = 2 * atan2(goal.pose.orientation.z, goal.pose.orientation.w);

  try{
    int ret = env_->SetStart(start.pose.position.x - costmap_ros_->getCostmap()->getOriginX(), start.pose.position.y - costmap_ros_->getCostmap()->getOriginY(), theta_start);
    if(ret < 0 || planner_->set_start(ret) == 0){
      ROS_ERROR("ERROR: failed to set start state\n");
      return false;
    }
  }
  catch(SBPL_Exception e){
    ROS_ERROR("SBPL encountered a fatal exception while setting the start state");
    return false;
  }

  try{
    int ret = env_->SetGoal(goal.pose.position.x - costmap_ros_->getCostmap()->getOriginX(), goal.pose.position.y - costmap_ros_->getCostmap()->getOriginY(), theta_goal);
    env_->SetGoalTolerance(goal_tolerance_, goal_tolerance_, M_PI);
    if(ret < 0 || planner_->set_goal(ret) == 0){
      ROS_ERROR("ERROR: failed to set goal state\n");
      return false;
    }
  }
  catch(SBPL_Exception e){
    ROS_ERROR("SBPL encountered a fatal exception while setting the goal state");
    return false;
  }

  int offOnCount = 0;
  int onOffCount = 0;
  int allCount = 0;
  vector<nav2dcell_t> changedcellsV;

  for(unsigned int ix = 0; ix < costmap_ros_->getCostmap()->getSizeInCellsX(); ix++) {
    for(unsigned int iy = 0; iy < costmap_ros_->getCostmap()->getSizeInCellsY(); iy++) {

      unsigned char oldCost = env_->GetMapCost(ix,iy);
      unsigned char newCost = costMapCostToSBPLCost(costmap_ros_->getCostmap()->getCost(ix,iy));

      if(oldCost == newCost) continue;

      allCount++;

      //first case - off cell goes on

      if((oldCost != costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) && oldCost != costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE)) &&
          (newCost == costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) || newCost == costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE))) {
        offOnCount++;
      }

      if((oldCost == costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) || oldCost == costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE)) &&
          (newCost != costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) && newCost != costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE))) {
        onOffCount++;
      }
      env_->UpdateCost(ix, iy, costMapCostToSBPLCost(costmap_ros_->getCostmap()->getCost(ix,iy)));

      nav2dcell_t nav2dcell;
      nav2dcell.x = ix;
      nav2dcell.y = iy;
      changedcellsV.push_back(nav2dcell);
    }
  }

  try{
    if(!changedcellsV.empty()){
      StateChangeQuery* scq = new LatticeSCQ(env_, changedcellsV);
      planner_->costs_changed(*scq);
      delete scq;
    }

    if(allCount > force_scratch_limit_)
      planner_->force_planning_from_scratch();
  }
  catch(SBPL_Exception e){
    ROS_ERROR("SBPL failed to update the costmap");
    return false;
  }

  //setting planner parameters
  ROS_DEBUG("allocated:%f, init eps:%f\n",allocated_time_,initial_epsilon_);
  planner_->set_initialsolution_eps(initial_epsilon_);
  planner_->set_search_mode(false);

  ROS_DEBUG("[sbpl_lattice_planner] run planner");
  vector<int> solution_stateIDs;
  int solution_cost;
  try{
    int ret = planner_->replan(allocated_time_, &solution_stateIDs, &solution_cost);
    if(ret)
      ROS_DEBUG("Solution is found\n");
    else{
      ROS_INFO("Solution not found\n");
      publishStats(solution_cost, 0, start, goal);
      return false;
    }
  }
  catch(SBPL_Exception e){
    ROS_ERROR("SBPL encountered a fatal exception while planning");
    return false;
  }

  ROS_DEBUG("size of solution=%d", (int)solution_stateIDs.size());

  vector<EnvNAVXYTHETALAT3Dpt_t> sbpl_path;
  try{
    env_->ConvertStateIDPathintoXYThetaPath(&solution_stateIDs, &sbpl_path);
  }
  catch(SBPL_Exception e){
    ROS_ERROR("SBPL encountered a fatal exception while reconstructing the path");
    return false;
  }
  ROS_DEBUG("Plan has %d points.\n", (int)sbpl_path.size());

  ros::Time plan_time = ros::Time::now();
  // check if the path can be shortened
  // first calculate the distance of each point relative to the start of the path
  std::vector<std::pair<double, geometry_msgs::PoseStamped> > path_with_absolut_distance;
  for (unsigned int i = 0; i < sbpl_path.size(); ++i)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = plan_time;
    pose.header.frame_id = costmap_ros_->getGlobalFrameID();

    pose.pose.position.x = sbpl_path[i].x + costmap_ros_->getCostmap()->getOriginX();
    pose.pose.position.y = sbpl_path[i].y + costmap_ros_->getCostmap()->getOriginY();
    pose.pose.position.z = start.pose.position.z;

    pose.pose.orientation = tf::createQuaternionMsgFromYaw(sbpl_path[i].theta);

    if (i == 0)
    {
      path_with_absolut_distance.push_back(std::make_pair(0.0, pose));
    }
    else
    {
      const geometry_msgs::PoseStamped previous_pose = path_with_absolut_distance[i - 1].second;
      const double previous_distance = path_with_absolut_distance[i - 1].first;

      double distance = previous_distance +
        sqrt(std::pow(pose.pose.position.x - previous_pose.pose.position.x, 2.0) +
                  std::pow(pose.pose.position.y - previous_pose.pose.position.y, 2.0));

      path_with_absolut_distance.push_back(std::make_pair(distance, pose));
    }
  }
  // create a path by removing those parts which are "far" away in the path distance but close in euclidan sense
  for (size_t i = 0; i < path_with_absolut_distance.size(); ++i)
  {
    const geometry_msgs::PoseStamped current_pose = path_with_absolut_distance[i].second;
    const double current_path_distance = path_with_absolut_distance[i].first;

    if (i > 0)
    {
      bool should_remove_elements = false;
      size_t start_index_of_removel = 0;

      for (size_t j = 0; j < plan.size(); ++j)
      {
        const geometry_msgs::PoseStamped previous_pose = path_with_absolut_distance[j].second;
        const double previous_distance = path_with_absolut_distance[j].first;

        if (std::abs(current_path_distance - previous_distance) <= min_euclidian_distance_)
          continue; //skip elements which are close in the path

        double euclidian_distance = sqrt(std::pow(current_pose.pose.position.x - previous_pose.pose.position.x, 2.0) +
                                         std::pow(current_pose.pose.position.y - previous_pose.pose.position.y, 2.0));

        if (euclidian_distance < min_euclidian_distance_)
        {
          start_index_of_removel = j;
          should_remove_elements = true;
          break;
        }
      }

      if (should_remove_elements)
        plan.erase(plan.begin() + start_index_of_removel,
                   plan.begin() + plan.size() - 1);
    }

    plan.push_back(current_pose);
  }

  //create a message for the plan 
  nav_msgs::Path gui_path;
  gui_path.poses.resize(sbpl_path.size());
  gui_path.header.frame_id = costmap_ros_->getGlobalFrameID();
  gui_path.header.stamp = plan_time;
  gui_path.poses = plan;
  plan_pub_.publish(gui_path);
  publishStats(solution_cost, sbpl_path.size(), start, goal);

  return true;
}

void SBPLLatticePlanner::clearRobotFootprint()
{
  std::vector<geometry_msgs::Point> oriented_footprint;
  costmap_ros_->getOrientedFootprint(oriented_footprint);
  geometry_msgs::Polygon oriented_polygon = costmap_2d::toPolygon(oriented_footprint);

  std::vector<geometry_msgs::Point> polygon_to_clear(oriented_polygon.points.size());
  for (size_t i = 0; i < oriented_polygon.points.size(); ++i)
  {
    polygon_to_clear[i].x = oriented_polygon.points[i].x;
    polygon_to_clear[i].y = oriented_polygon.points[i].y;
    polygon_to_clear[i].z = oriented_polygon.points[i].z;
  }

  costmap_ros_->getCostmap()->setConvexPolygonCost(polygon_to_clear, costmap_2d::FREE_SPACE);
}

bool SBPLLatticePlanner::positionInCollision(const geometry_msgs::Pose &pose)
{
  return positionInCollision(pose.position);
}

bool SBPLLatticePlanner::positionInCollision(const geometry_msgs::Point &position)
{
  int map_x;
  int map_y;
  costmap_ros_->getCostmap()->worldToMapEnforceBounds(position.x, position.y, map_x, map_y);

  const unsigned char costs = costmap_ros_->getCostmap()->getCost(map_x, map_y);
  return ((costs == costmap_2d::LETHAL_OBSTACLE) || (costs == costmap_2d::INSCRIBED_INFLATED_OBSTACLE));
}

bool SBPLLatticePlanner::findOrthogonalGoal(const geometry_msgs::Pose& goal_pose, geometry_msgs::Pose& new_goal_pose)
{
  const double goal_angular = tf::getYaw(goal_pose.orientation);
  const double plus_orthognal_angular = goal_angular + M_PI_2;
  const double minus_orthognal_angular = goal_angular - M_PI_2;

  for (double distance = 0.; distance < max_orthogonal_distance_; distance += orthogonal_step_size_)
  {
    new_goal_pose = calcPose(goal_pose, distance, plus_orthognal_angular);
    if (!positionInCollision(new_goal_pose))
      return true;

    new_goal_pose = calcPose(goal_pose, distance, minus_orthognal_angular);
    if (!positionInCollision(new_goal_pose))
      return true;
  }

  new_goal_pose = goal_pose;
  return false;
}

geometry_msgs::Pose SBPLLatticePlanner::calcPose(const geometry_msgs::Pose& pose, double distance, double angular)
{
  geometry_msgs::Pose resulting_pose;
  resulting_pose.orientation = pose.orientation;

  resulting_pose.position.z = pose.position.z;

  resulting_pose.position.x = pose.position.x + distance * std::cos(angular);
  resulting_pose.position.y = pose.position.y + distance * std::sin(angular);

  return resulting_pose;
}


};

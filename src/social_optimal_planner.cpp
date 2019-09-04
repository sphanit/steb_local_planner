/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
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
 *   * Neither the name of the institute nor the names of its
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
 * Author: Christoph Rösmann
 *********************************************************************/

#include <teb_local_planner/social_optimal_planner.h>
#include <map>
#include <memory>
#include <limits>


namespace teb_local_planner
{

// ============== Implementation ===================

SocialTebOptimalPlanner::SocialTebOptimalPlanner() : TebOptimalPlanner(), humans_(NULL)
{
}
SocialTebOptimalPlanner::SocialTebOptimalPlanner(
    const TebConfig &cfg, ObstContainer *obstacles,
    RobotFootprintModelPtr robot_model, TebVisualizationPtr visual,
    const ViaPointContainer *via_points, HumanContainer *humans): TebOptimalPlanner(cfg, obstacles, robot_model, visual, via_points), humans_(humans) {
    initialize(cfg, obstacles, robot_model, visual, via_points, humans);
}

void SocialTebOptimalPlanner::initialize(
    const teb_local_planner::TebConfig &cfg,
    teb_local_planner::ObstContainer *obstacles,
    teb_local_planner::RobotFootprintModelPtr robot_model,
    teb_local_planner::TebVisualizationPtr visual,
    const teb_local_planner::ViaPointContainer *via_points,
    teb_local_planner::HumanContainer *humans) {

    // init optimizer (set solver and block ordering settings)
    optimizer_ = initOptimizerWithHumans();

    cfg_ = &cfg;
    obstacles_ = obstacles;
    humans_ = humans;
    robot_model_ = robot_model;
    via_points_ = via_points;
    cost_ = HUGE_VAL;
    prefer_rotdir_ = RotType::none;
    setVisualization(visual);

    vel_start_.first = true;
    vel_start_.second.linear.x = 0;
    vel_start_.second.linear.y = 0;
    vel_start_.second.angular.z = 0;

    vel_goal_.first = true;
    vel_goal_.second.linear.x = 0;
    vel_goal_.second.linear.y = 0;
    vel_goal_.second.angular.z = 0;
    initialized_ = true;
  }

void SocialTebOptimalPlanner::registerG2OTypesWithHumans()
{
  g2o::Factory* factory = g2o::Factory::instance();
  /*
  factory->registerType("VERTEX_POSE", new g2o::HyperGraphElementCreator<VertexPose>);
  factory->registerType("VERTEX_TIMEDIFF", new g2o::HyperGraphElementCreator<VertexTimeDiff>);

  factory->registerType("EDGE_TIME_OPTIMAL", new g2o::HyperGraphElementCreator<EdgeTimeOptimal>);
  factory->registerType("EDGE_VELOCITY", new g2o::HyperGraphElementCreator<EdgeVelocity>);
  factory->registerType("EDGE_VELOCITY_HOLONOMIC", new g2o::HyperGraphElementCreator<EdgeVelocityHolonomic>);
  factory->registerType("EDGE_ACCELERATION", new g2o::HyperGraphElementCreator<EdgeAcceleration>);
  factory->registerType("EDGE_ACCELERATION_START", new g2o::HyperGraphElementCreator<EdgeAccelerationStart>);
  factory->registerType("EDGE_ACCELERATION_GOAL", new g2o::HyperGraphElementCreator<EdgeAccelerationGoal>);
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC", new g2o::HyperGraphElementCreator<EdgeAccelerationHolonomic>);
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC_START", new g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicStart>);
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC_GOAL", new g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicGoal>);
  factory->registerType("EDGE_KINEMATICS_DIFF_DRIVE", new g2o::HyperGraphElementCreator<EdgeKinematicsDiffDrive>);
  factory->registerType("EDGE_KINEMATICS_CARLIKE", new g2o::HyperGraphElementCreator<EdgeKinematicsCarlike>);
  factory->registerType("EDGE_OBSTACLE", new g2o::HyperGraphElementCreator<EdgeObstacle>);
  factory->registerType("EDGE_INFLATED_OBSTACLE", new g2o::HyperGraphElementCreator<EdgeInflatedObstacle>);
  factory->registerType("EDGE_DYNAMIC_OBSTACLE", new g2o::HyperGraphElementCreator<EdgeDynamicObstacle>);
  factory->registerType("EDGE_VIA_POINT", new g2o::HyperGraphElementCreator<EdgeViaPoint>);
  factory->registerType("EDGE_PREFER_ROTDIR", new g2o::HyperGraphElementCreator<EdgePreferRotDir>);
  */
  factory->registerType("EDGE_SOCIAL_PROXEMICS", new g2o::HyperGraphElementCreator<EdgeProxemics>);
  factory->registerType("EDGE_SOCIAL_LOOKAT_HUMAN", new g2o::HyperGraphElementCreator<EdgeLookatHuman>);

}

boost::shared_ptr<g2o::SparseOptimizer> SocialTebOptimalPlanner::initOptimizerWithHumans(){
  // Call register_g2o_types once, even for multiple TebOptimalPlanner instances (thread-safe)
  static boost::once_flag flag = BOOST_ONCE_INIT;
  boost::call_once(&registerG2OTypesWithHumans, flag);

  // allocating the optimizer
  boost::shared_ptr<g2o::SparseOptimizer> optimizer = boost::make_shared<g2o::SparseOptimizer>();
  std::unique_ptr<TEBLinearSolver> linear_solver(new TEBLinearSolver()); // see typedef in optimization.h
  linear_solver->setBlockOrdering(true);
  std::unique_ptr<TEBBlockSolver> block_solver(new TEBBlockSolver(std::move(linear_solver)));
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

  optimizer->setAlgorithm(solver);

  optimizer->initMultiThreading(); // required for >Eigen 3.1

  return optimizer;
}

bool SocialTebOptimalPlanner::buildGraph(double weight_multiplier){
  if (!optimizer_->edges().empty() || !optimizer_->vertices().empty())
  {
    ROS_WARN("Cannot build graph, because it is not empty. Call graphClear()!");
    return false;
  }

  // add TEB vertices
  AddTEBVertices();

  // add Edges (local cost functions)
  if (cfg_->obstacles.legacy_obstacle_association)
    AddEdgesObstaclesLegacy(weight_multiplier);
  else
    AddEdgesObstacles(weight_multiplier);

  if (cfg_->obstacles.include_dynamic_obstacles)
    AddEdgesDynamicObstacles();

  AddEdgesViaPoints();

  AddEdgesVelocity();

  AddEdgesAcceleration();

  AddEdgesTimeOptimal();

  if (cfg_->robot.min_turning_radius == 0 || cfg_->optim.weight_kinematics_turning_radius == 0)
    AddEdgesKinematicsDiffDrive(); // we have a differential drive robot
  else
    AddEdgesKinematicsCarlike(); // we have a carlike robot since the turning radius is bounded from below.


  AddEdgesPreferRotDir();

  if(cfg_->socialTeb.use_proxemics){
    AddEdgesProxemics();
    // ROS_INFO("Proxemics edge added");
  }

  // if(cfg_->socialTeb.use_ttc){
  //   AddEdgesTTC();
  //   // ROS_INFO("TTC edge added");
  // }
  //
  // if(cfg_->socialTeb.use_directional){
  //   AddEdgesDirectional();
  //   // ROS_INFO("Directional edge added");
  // }
  //
  // if(cfg_->socialTeb.use_visibility){
  //   AddEdgesVisibility();
  //   // ROS_INFO("Visibility edge added");
  // }

  if(cfg_->socialTeb.use_lookathuman){
    AddEdgesLookatHuman();
    // ROS_INFO("Visibility edge added");
  }

  return true;
}

void SocialTebOptimalPlanner::AddEdgesProxemics(){
  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_social_proxemics);

  // iterate all teb points (skip first and last)
  for (int i=1; i < teb_.sizePoses()-1; ++i)
  {
    Human* human = nullptr;

    std::vector<Human*> relevant_humans;

    const Eigen::Vector2d pose_orient = teb_.Pose(i).orientationUnitVec();

    // iterate obstacles
    for (const HumanPtr& human : *humans_)
    {

      // calculate distance to current pose
      double dist = human->getMinimumDistance(teb_.Pose(i).position());

      // force considering obstacle if really close to the current pose
      if (dist < cfg_->socialTeb.min_robot_human_distance * cfg_->socialTeb.robot_human_distance_cutoff_factor)
      {
        relevant_humans.push_back(human.get());
        continue;
      }
    }

    for (const Human* human : relevant_humans)
    {
      EdgeProxemics* dist_bandpt_prox = new EdgeProxemics;
      dist_bandpt_prox->setVertex(0,teb_.PoseVertex(i));
      dist_bandpt_prox->setInformation(information);
      dist_bandpt_prox->setParameters(*cfg_, robot_model_.get(), human);
      optimizer_->addEdge(dist_bandpt_prox);
    }
  }
}

void SocialTebOptimalPlanner::AddEdgesTTC(){
  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_social_ttc);
  // information.fill(cfg_->optim.weight_social_ttc);

  // iterate all teb points (skip first and last)
  for (int i=1; i < teb_.sizePoses()-1; ++i)
  {
    Human* human = nullptr;
    std::vector<Human*> relevant_humans;

  // iterate obstacles
    for (const HumanPtr& human : *humans_)
    {
      // calculate distance to current pose
      double dist = human->getMinimumDistance(teb_.Pose(i).position());

      // force considering obstacle if really close to the current pose
      if (dist < 2.0)
      {
        relevant_humans.push_back(human.get());
        continue;
      }
    }

    for (const Human* human : relevant_humans)
    {
      EdgeTTC *ttc_edge = new EdgeTTC;
      // const double robot_radius_ = robot_model_->getInscribedRadius();
      ttc_edge->setVertex(0, teb_.PoseVertex(i));
      ttc_edge->setVertex(1, teb_.PoseVertex(i + 1));
      ttc_edge->setVertex(2, teb_.TimeDiffVertex(i));
      ttc_edge->setInformation(information);
      ttc_edge->setParameters(*cfg_, robot_model_->getInscribedRadius(), human);
      optimizer_->addEdge(ttc_edge);
    }
  }
}

void SocialTebOptimalPlanner::AddEdgesDirectional(){
  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_social_directional);

  // iterate all teb points (skip first and last)
  for (int i=1; i < teb_.sizePoses()-1; ++i)
  {
    Human* human = nullptr;

    std::vector<Human*> relevant_humans;

    const Eigen::Vector2d pose_orient = teb_.Pose(i).orientationUnitVec();

    // iterate obstacles
    for (const HumanPtr& human : *humans_)
    {

      // calculate distance to current pose
      double dist = human->getMinimumDistance(teb_.Pose(i).position());

      // force considering obstacle if really close to the current pose
      if (dist < cfg_->socialTeb.min_robot_human_distance * cfg_->socialTeb.robot_human_distance_cutoff_factor)
      {
        relevant_humans.push_back(human.get());
        continue;
      }
    }

    for (const Human* human : relevant_humans)
    {
      EdgeDirectional* edge_directional = new EdgeDirectional();
      edge_directional->setVertex(0, teb_.PoseVertex(i));
      edge_directional->setVertex(1, teb_.PoseVertex(i + 1));
      edge_directional->setVertex(2, teb_.TimeDiffVertex(i));
      edge_directional->setInformation(information);
      edge_directional->setParameters(*cfg_, human);
      optimizer_->addEdge(edge_directional);
    }
  }
}

void SocialTebOptimalPlanner::AddEdgesVisibility(){
  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_social_visibility);

  // iterate all teb points (skip first and last)
  for (int i=1; i < teb_.sizePoses()-1; ++i)
  {
    Human* human = nullptr;

    std::vector<Human*> relevant_humans;

    const Eigen::Vector2d pose_orient = teb_.Pose(i).orientationUnitVec();

    // iterate obstacles
    for (const HumanPtr& human : *humans_)
    {

      // calculate distance to current pose
      double dist = human->getMinimumDistance(teb_.Pose(i).position());

      // force considering obstacle if really close to the current pose
      if (dist < cfg_->socialTeb.min_robot_human_distance * cfg_->socialTeb.robot_human_distance_cutoff_factor)
      {
        relevant_humans.push_back(human.get());
        continue;
      }
    }

    for (const Human* human : relevant_humans)
    {
      EdgeVisibility *visibility_edge = new EdgeVisibility;
      visibility_edge->setVertex(0, teb_.PoseVertex(i));
      visibility_edge->setInformation(information);
      visibility_edge->setParameters(*cfg_, human);;
      optimizer_->addEdge(visibility_edge);
    }
  }
}

void SocialTebOptimalPlanner::AddEdgesLookatHuman(){
  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_social_lookathuman);
  int n = teb_.sizePoses()-1;
  // iterate all teb points (skip first and last)
  for (int i=1; i < teb_.sizePoses()-1; ++i)
  {
    Human* human = nullptr;

    std::vector<Human*> relevant_humans;

    const Eigen::Vector2d pose_orient = teb_.Pose(i).orientationUnitVec();

    // iterate obstacles
    for (const HumanPtr& human : *humans_)
    {
      // calculate distance to current pose
      double dist = human->getMinimumDistance(teb_.Pose(i).position());

      //calculate position of robot
      Eigen::Vector2d d_rtoh = human->position() - teb_.Pose(i).position();
      Eigen::Vector2d humanLookAt = {cos(human->getTheta()), sin(human->getTheta())};
      double hdeltaPsi = fabs(acos(humanLookAt.dot(d_rtoh) / (humanLookAt.norm() * d_rtoh.norm())));
      Eigen::Vector2d vec_goal = teb_.Pose(n).position() - teb_.Pose(1).position();
      double dist_goal = vec_goal.norm();
      // force considering obstacle if really close to the current pose
      if ((dist < cfg_->socialTeb.min_robot_human_distance * cfg_->socialTeb.robot_human_distance_cutoff_factor) && (hdeltaPsi >= cfg_->socialTeb.fov * M_PI/180))
      {
        if(dist_goal > 0.5)
        relevant_humans.push_back(human.get());
        continue;
      }
    }

    for (const Human* human : relevant_humans)
    {
      EdgeLookatHuman *lookathuman_edge = new EdgeLookatHuman;
      lookathuman_edge->setVertex(0, teb_.PoseVertex(i));
      lookathuman_edge->setInformation(information);
      lookathuman_edge->setParameters(*cfg_, human);;
      optimizer_->addEdge(lookathuman_edge);
    }
  }
}

void SocialTebOptimalPlanner::computeCurrentCostWithHumans(double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost)
{
  // check if graph is empty/exist  -> important if function is called between buildGraph and optimizeGraph/clearGraph
  bool graph_exist_flag(false);
  if (optimizer_->edges().empty() && optimizer_->vertices().empty())
  {
    // here the graph is build again, for time efficiency make sure to call this function
    // between buildGraph and Optimize (deleted), but it depends on the application
    buildGraph();
    optimizer_->initializeOptimization();
  }
  else
  {
    graph_exist_flag = true;
  }

  optimizer_->computeInitialGuess();

  cost_ = 0;

  if (alternative_time_cost)
  {
    cost_ += teb_.getSumOfAllTimeDiffs();
    // TEST we use SumOfAllTimeDiffs() here, because edge cost depends on number of samples, which is not always the same for similar TEBs,
    // since we are using an AutoResize Function with hysteresis.
  }

  // now we need pointers to all edges -> calculate error for each edge-type
  // since we aren't storing edge pointers, we need to check every edge
  for (std::vector<g2o::OptimizableGraph::Edge*>::const_iterator it = optimizer_->activeEdges().begin(); it!= optimizer_->activeEdges().end(); it++)
  {
    EdgeTimeOptimal* edge_time_optimal = dynamic_cast<EdgeTimeOptimal*>(*it);
    if (edge_time_optimal!=NULL && !alternative_time_cost)
    {
      cost_ += edge_time_optimal->getError().squaredNorm();
      cost_dict["optimal_time"] = edge_time_optimal->getError().squaredNorm();
      continue;
    }

    EdgeKinematicsDiffDrive* edge_kinematics_dd = dynamic_cast<EdgeKinematicsDiffDrive*>(*it);
    if (edge_kinematics_dd!=NULL)
    {
      cost_ += edge_kinematics_dd->getError().squaredNorm();
      continue;
    }

    EdgeKinematicsCarlike* edge_kinematics_cl = dynamic_cast<EdgeKinematicsCarlike*>(*it);
    if (edge_kinematics_cl!=NULL)
    {
      cost_ += edge_kinematics_cl->getError().squaredNorm();
      continue;
    }

    EdgeVelocity* edge_velocity = dynamic_cast<EdgeVelocity*>(*it);
    if (edge_velocity!=NULL)
    {
      cost_ += edge_velocity->getError().squaredNorm();
      cost_dict["velocity"] = edge_velocity->getError().squaredNorm();
      continue;
    }

    EdgeAcceleration* edge_acceleration = dynamic_cast<EdgeAcceleration*>(*it);
    if (edge_acceleration!=NULL)
    {
      cost_ += edge_acceleration->getError().squaredNorm();
      continue;
    }

    EdgeObstacle* edge_obstacle = dynamic_cast<EdgeObstacle*>(*it);
    if (edge_obstacle!=NULL)
    {
      cost_ += edge_obstacle->getError().squaredNorm() * obst_cost_scale;
      continue;
    }

    EdgeInflatedObstacle* edge_inflated_obstacle = dynamic_cast<EdgeInflatedObstacle*>(*it);
    if (edge_inflated_obstacle!=NULL)
    {
      cost_ += std::sqrt(std::pow(edge_inflated_obstacle->getError()[0],2) * obst_cost_scale
               + std::pow(edge_inflated_obstacle->getError()[1],2));
      continue;
    }

    EdgeDynamicObstacle* edge_dyn_obstacle = dynamic_cast<EdgeDynamicObstacle*>(*it);
    if (edge_dyn_obstacle!=NULL)
    {
      cost_ += edge_dyn_obstacle->getError().squaredNorm() * obst_cost_scale;
      continue;
    }

    EdgeViaPoint* edge_viapoint = dynamic_cast<EdgeViaPoint*>(*it);
    if (edge_viapoint!=NULL)
    {
      cost_ += edge_viapoint->getError().squaredNorm() * viapoint_cost_scale;
      continue;
    }

    EdgeProxemics* edge_proxemics = dynamic_cast<EdgeProxemics*>(*it);
    if (edge_proxemics!=NULL)
    {
      cost_ += edge_proxemics->getError().squaredNorm();
      continue;
    }

    EdgeLookatHuman* edge_lookathuman = dynamic_cast<EdgeLookatHuman*>(*it);
    if (edge_lookathuman!=NULL)
    {
      cost_ += edge_lookathuman->getError().squaredNorm();
      continue;
    }
  }

  // delete temporary created graph
  if (!graph_exist_flag)
    clearGraph();
}

bool SocialTebOptimalPlanner::optimizeTEB(int iterations_innerloop, int iterations_outerloop, bool compute_cost_afterwards,
                                    double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost)
{
  if (cfg_->optim.optimization_activate==false)
    return false;

  bool success = false;
  optimized_ = false;

  double weight_multiplier = 1.0;

  // TODO(roesmann): we introduced the non-fast mode with the support of dynamic obstacles
  //                (which leads to better results in terms of x-y-t homotopy planning).
  //                 however, we have not tested this mode intensively yet, so we keep
  //                 the legacy fast mode as default until we finish our tests.
  bool fast_mode = !cfg_->obstacles.include_dynamic_obstacles;

  for(int i=0; i<iterations_outerloop; ++i)
  {
    if (cfg_->trajectory.teb_autosize)
    {
      //teb_.autoResize(cfg_->trajectory.dt_ref, cfg_->trajectory.dt_hysteresis, cfg_->trajectory.min_samples, cfg_->trajectory.max_samples);
      teb_.autoResize(cfg_->trajectory.dt_ref, cfg_->trajectory.dt_hysteresis, cfg_->trajectory.min_samples, cfg_->trajectory.max_samples, fast_mode);

    }

    success = buildGraph(weight_multiplier);
    if (!success)
    {
        clearGraph();
        return false;
    }
    success = optimizeGraph(iterations_innerloop, false);
    if (!success)
    {
        clearGraph();
        return false;
    }
    optimized_ = true;

    if (compute_cost_afterwards && i==iterations_outerloop-1) // compute cost vec only in the last iteration
      computeCurrentCostWithHumans(obst_cost_scale, viapoint_cost_scale, alternative_time_cost);

    clearGraph();

    weight_multiplier *= cfg_->optim.weight_adapt_factor;
  }

  return true;
}

} // namespace teb_local_planner

//
// Created by Phani Teja
//
#ifndef TEB_LOCAL_PLANNER_EDGE_DIRECTIONAL_H
#define TEB_LOCAL_PLANNER_EDGE_DIRECTIONAL_H

#include <teb_local_planner/g2o_types/base_teb_edges.h>

namespace teb_local_planner{

class EdgeDirectional : public BaseTebMultiEdge<1, const Human*>
{
public:
  EdgeDirectional(){
    _measurement = NULL;
    this->resize(3);
    // _measurement = NULL;
  }

  void computeError()
  {
    ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setHuman() and setParameters() on EdgeDirectional()");

    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* bandpt_nxt = static_cast<const VertexPose*>(_vertices[1]);
    const VertexTimeDiff* dt_robot = static_cast<const VertexTimeDiff*>(_vertices[2]);



    Eigen::Vector2d robot_diff = bandpt_nxt->position() - bandpt->position();
    Eigen::Vector2d robot_vel = robot_diff / dt_robot->dt();
    Eigen::Vector2d d_htor = bandpt->position() -  _measurement->getCentroid();
    Eigen::Vector2d d_rtoh = _measurement->getCentroid() - bandpt->position();
    Eigen::Vector2d human_vel = _measurement->getCentroidVelocity();

    double dir_cost = (std::max(robot_vel.dot(d_rtoh), 0.0) +
                       std::max(human_vel.dot(d_htor), 0.0)) /
                      d_rtoh.dot(d_rtoh);
    // std::cout << dir_cost << '\n';

    if((human_vel.dot(human_vel))>0){
      _error[0] = penaltyBoundFromBelow(dir_cost, cfg_->socialTeb.directional_cost_threshold, cfg_->optim.penalty_epsilon);
    }
    else{
      _error[0] = 0.0;
    }
    ROS_DEBUG_THROTTLE(0.5, "dir_cost value : %f", dir_cost);
    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeHumanRobot::computeError() _error[0]=%f\n", _error[0]);
  }

  void setHuman(const Human* human)
  {
    _measurement = human;
  }

  void setParameters(const TebConfig& cfg, const Human* human)
  {
    cfg_ = &cfg;
    _measurement = human;
    // radius_sum_ = robot_radius + human->radius();
    // radius_sum_sq_ = radius_sum_ * radius_sum_;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

} // namespace teb_local_planner

#endif // TEB_LOCAL_PLANNER_EDGE_DIRECTIONAL_H

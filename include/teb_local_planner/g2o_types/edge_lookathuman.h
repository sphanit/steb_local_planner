//
// Created by Phani Teja
//
#ifndef TEB_LOCAL_PLANNER_EDGE_LOOKATHUMAN_H
#define TEB_LOCAL_PLANNER_EDGE_LOOKATHUMAN_H

#include <teb_local_planner/g2o_types/base_teb_edges.h>

namespace teb_local_planner{

class EdgeLookatHuman : public BaseTebUnaryEdge<1, const Human*, VertexPose>
{
public:
  EdgeLookatHuman(){
    _measurement = NULL;
  }

  void computeError()
  {
    ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setHuman() and setParameters() on EdgeLookatHuman()");

    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);

    Eigen::Vector2d d_rtoh = _measurement->position() - bandpt->position();
    Eigen::Vector2d d_htor = bandpt->position() - _measurement->position();
    Eigen::Vector2d robotLookAt = {cos(bandpt->theta()), sin(bandpt->theta())};
    double rdeltaPsi = fabs(acos(robotLookAt.dot(d_rtoh) / (robotLookAt.norm() * d_rtoh.norm())));

    double c_lookat;

    c_lookat = fabs(rdeltaPsi - cfg_->socialTeb.lookathuman_cost_threshold);

    _error[0] = penaltyBoundFromAbove(c_lookat, cfg_->socialTeb.lookathuman_cost_threshold,
                                  cfg_->optim.penalty_epsilon);

    ROS_ASSERT_MSG(std::isfinite(_error[0]),"EdgeLookatHuman::computeError() _error[0]=%f\n", _error[0]);
  }

  void setHuman(const Human* human)
  {
    _measurement = human;
  }

  void setParameters(const TebConfig& cfg, const Human* human)
  {
    cfg_ = &cfg;
    _measurement = human;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

} // namespace teb_local_planner

  #endif // TEB_LOCAL_PLANNER_EDGE_LOOKATHUMAN_H

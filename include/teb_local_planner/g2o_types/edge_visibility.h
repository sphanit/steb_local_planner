//
// Created by Phani Teja
//
#ifndef TEB_LOCAL_PLANNER_EDGE_VISIBILITY_H
#define TEB_LOCAL_PLANNER_EDGE_VISIBILITY_H

#include <teb_local_planner/g2o_types/base_teb_edges.h>

namespace teb_local_planner{

class EdgeVisibility : public BaseTebUnaryEdge<1, const Human*, VertexPose>
{
public:
  EdgeVisibility(){
    _measurement = NULL;
  }

  void computeError()
  {
    ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setHuman() and setParameters() on EdgeVisibility()");

    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);

    Eigen::Vector2d d_rtoh = _measurement->position() - bandpt->position();
    Eigen::Vector2d d_htor = bandpt->position() - _measurement->position();
    Eigen::Vector2d humanLookAt = {cos(_measurement->getTheta()), sin(_measurement->getTheta())};
    double deltaPsi = fabs(acos(humanLookAt.dot(d_htor) / (humanLookAt.norm() + d_htor.norm())));
    double c_visibility;

    if (deltaPsi >= cfg_->socialTeb.fov * M_PI / 180){
        c_visibility = deltaPsi * ((cos(d_rtoh.x()) + 1) * (cos(d_rtoh.y()) + 1));
    }else{
        c_visibility = 0.;
    }

    _error[0] = penaltyBoundFromAbove(c_visibility, cfg_->socialTeb.visibility_cost_threshold,
                                  cfg_->optim.penalty_epsilon);

    ROS_ASSERT_MSG(std::isfinite(_error[0]),
               "EdgeHumanRobotVisibility::computeError() _error[0]=%f\n",
               _error[0]);
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

#endif // TEB_LOCAL_PLANNER_EDGE_VISIBILITY_H

//
// Created by Phani Teja
//
#ifndef TEB_LOCAL_PLANNER_EDGE_TTC_H
#define TEB_LOCAL_PLANNER_EDGE_TTC_H

#include <teb_local_planner/g2o_types/base_teb_edges.h>
// #include <fstream>

namespace teb_local_planner{

class EdgeTTC : public BaseTebMultiEdge<1, const Human*>
{
public:
  EdgeTTC(){
    _measurement = NULL;
    this->resize(3);
    // _measurement = NULL;
  }

  void computeError()
  {
    ROS_ASSERT_MSG(cfg_ && _measurement && radius_sum_, "You must call setHuman() and setParameters() on EdgeTTC()");

    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* bandpt_nxt = static_cast<const VertexPose*>(_vertices[1]);
    const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);



    Eigen::Vector2d diff = bandpt_nxt->position() - bandpt->position();
    Eigen::Vector2d vel = diff / dt->dt();
    Eigen::Vector2d C = (_measurement->getCentroid() - bandpt->position());

    double ttc = std::numeric_limits<double>::infinity();
    double C_sq = C.dot(C);

    // std::cout << "C is " << C << '\n';
    // std::cout << "pos_human is " << _measurement->position() << '\n';


    if (C_sq <= radius_sum_sq_)
    {
      ttc = 0.0;
    }
    else
    {
      Eigen::Vector2d V = vel - _measurement->getCentroidVelocity();
      double C_dot_V = C.dot(V);
      double C_dot_Vh = C.dot(_measurement->getCentroidVelocity());
      if ((C_dot_V > 0) && (C_dot_Vh <= 0)) { // otherwise ttc is infinite
        double V_sq = V.dot(V);
        double f = (C_dot_V * C_dot_V)- (V_sq * (C_sq - radius_sum_sq_));
        if (f > 0) { // otherwise ttc is infinite
          ttc = (C_dot_V - std::sqrt(f)) / V_sq;
        }
      }
    }

    // std::cout << ttc << '\n';
    // std::cout <<"velocity of robot" <<vel << '\n';
    // std::cout <<"velocity of human" <<_measurement->getCentroidVelocity() << '\n';

    if (ttc < std::numeric_limits<double>::infinity()) {

      _error[0] = penaltyBoundFromBelow(ttc, cfg_->socialTeb.ttc_threshold, cfg_->optim.penalty_epsilon);
      if (cfg_->socialTeb.scale_ttc) {
        _error[0] = _error[0] * cfg_->optim.ttc_scale_alpha / C_sq;
      }

    } else {
      // no collsion possible
      _error[0] = 0.0;
    }

    //std::cout << "TTC error" << _error[0] << '\n';

    ROS_DEBUG_THROTTLE(0.5, "TTC value : %f", ttc);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeTTC::computeError() _error[0]=%f\n", _error[0]);
  }

  void setHuman(const Human* human)
  {
    _measurement = human;
  }

  void setParameters(const TebConfig& cfg, const double& robot_radius, const Human* human)
  {
    cfg_ = &cfg;
    _measurement = human;
    radius_sum_ = robot_radius + human->radius();
    radius_sum_sq_ = radius_sum_ * radius_sum_;
  }

protected:
  double radius_sum_ = std::numeric_limits<double>::infinity();
  double radius_sum_sq_ = std::numeric_limits<double>::infinity();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

} // namespace teb_local_planner

#endif // TEB_LOCAL_PLANNER_EDGE_TTC_H

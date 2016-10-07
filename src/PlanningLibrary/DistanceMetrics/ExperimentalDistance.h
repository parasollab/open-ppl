#ifndef EXPERIMENTAL_DISTANCE_H_
#define EXPERIMENTAL_DISTANCE_H_

#include "DistanceMetricMethod.h"
#include "WeightedEuclideanDistance.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup DistanceMetrics
/// @brief These aren't the droids you're looking for.
/// @tparam MPTraits Motion planning universe
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class ExperimentalDistance : public WeightedEuclideanDistance<MPTraits> {
  public:
    typedef typename MPTraits::CfgType StateType;
    typedef typename MPTraits::MPProblemType MPProblemType;

    ExperimentalDistance(double _posW = 0.25, double _rotW = 0.25,
        double _velW = 0.25, double _avlW = 0.25, double _ts = 30);
    ExperimentalDistance(MPProblemType* _problem, XMLNode& _node);

    virtual double Distance(const StateType& _s1, const StateType& _s2);

  private:

    double m_timestep; ///< The timestep to use.
};


template<class MPTraits>
ExperimentalDistance<MPTraits>::
ExperimentalDistance(double _posW, double _rotW, double _velW,
    double _avlW, double _ts) :
    WeightedEuclideanDistance<MPTraits>(_posW, _rotW, _velW, _avlW),
    m_timestep(_ts) {
  this->SetName("ExperimentalDistance");
}

template<class MPTraits>
ExperimentalDistance<MPTraits>::
ExperimentalDistance(MPProblemType* _problem, XMLNode& _node) :
    WeightedEuclideanDistance<MPTraits>(_problem, _node) {
  this->SetName("ExperimentalDistance");

  m_timestep = _node.Read("timestep", true, 0., 0.,
      numeric_limits<double>::max(), "The fixed timestep");
}

template<class MPTraits>
double
ExperimentalDistance<MPTraits>::
Distance(const StateType& _s1, const StateType& _s2) {
  // Assume that _s1 will coast toward _s2 at its present velocity for m_timestep.
  StateType s1 = _s1;
  double dt = m_timestep * this->GetEnvironment()->GetTimeRes();
  s1.Apply(s1.GetVelocity(), dt);
  return WeightedEuclideanDistance<MPTraits>::Distance(s1, _s2);
}

#endif

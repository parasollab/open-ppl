#ifndef EXPERIMENTAL_DISTANCE_H_
#define EXPERIMENTAL_DISTANCE_H_

#include "DistanceMetricMethod.h"
#include "WeightedEuclideanDistance.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup DistanceMetrics
/// @brief These aren't the droids you're looking for.
///
/// This metric applies only to nonholonomic robots. It works as a weighted
/// Euclidean distance between two states, except that it allows the first
/// state to coast under its present momentum for one timestep before checking
/// the distance. Thus, it measures how close the first state will be to the
/// second after drifting.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class ExperimentalDistance : public WeightedEuclideanDistance<MPTraits> {

  public:

    ///@name Local Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    ExperimentalDistance();
    ExperimentalDistance(XMLNode& _node);
    virtual ~ExperimentalDistance() = default;

    ///@}
    ///@name Distance Interface
    ///@{

    virtual double Distance(const CfgType& _s1, const CfgType& _s2) override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    double m_timestep{30}; ///< The timestep to use.

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
ExperimentalDistance<MPTraits>::
ExperimentalDistance() : WeightedEuclideanDistance<MPTraits>() {
  this->SetName("ExperimentalDistance");
}


template <typename MPTraits>
ExperimentalDistance<MPTraits>::
ExperimentalDistance(XMLNode& _node) :
    WeightedEuclideanDistance<MPTraits>(_node) {
  this->SetName("ExperimentalDistance");

  m_timestep = _node.Read("timestep", true, 0., 0.,
      numeric_limits<double>::max(), "The fixed timestep");
}

/*----------------------------- Distance Interface ---------------------------*/

template <typename MPTraits>
double
ExperimentalDistance<MPTraits>::
Distance(const CfgType& _s1, const CfgType& _s2) {
  // Assume that _s1 will coast toward _s2 at its present velocity for m_timestep.
  CfgType s1 = _s1;
  double dt = m_timestep * this->GetEnvironment()->GetTimeRes();
  s1.Apply(s1.GetVelocity(), dt);
  return WeightedEuclideanDistance<MPTraits>::Distance(s1, _s2);
}

/*----------------------------------------------------------------------------*/

#endif

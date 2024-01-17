#ifndef MINIMUM_DISTANCE_EVALUATOR_H_
#define MINIMUM_DISTANCE_EVALUATOR_H_

#include <limits>

#include "MapEvaluatorMethod.h"
#include "Utilities/MetricUtils.h"

////////////////////////////////////////////////////////////////////////////////
/// This evaluator returns true once a minimum distance (using the provided
/// distance metric) has been reached in any roadmap configuration, relative to
/// the first (root) configuration in the roadmap.
///
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
class MinimumDistanceEvaluator : virtual public MapEvaluatorMethod {
 public:
  ///@name Motion Planning Types
  ///@{

  typedef typename MPBaseObject::RoadmapType RoadmapType;
  typedef typename RoadmapType::VID VID;

  ///@}
  ///@name Construction
  ///@{

  MinimumDistanceEvaluator(const double _timeout = 10,
                           const string _dmLabel = "",
                           const double _minDist = 1.0);

  MinimumDistanceEvaluator(XMLNode& _node);

  virtual ~MinimumDistanceEvaluator() = default;

  ///@}
  ///@name MPBaseObject Interface
  ///@{

  virtual void Initialize() override;

  ///@}
  ///@name MapEvaluatorMethod Interface
  ///@{

  virtual bool operator()() override;

  ///@}

 private:
  ///@name Internal State
  ///@{

  string m_dmLabel;                ///< Distance metric label
  double m_minDist = 1.0;          ///< Minimum distance for maximum time
  bool m_minimumAchieved = false;  ///< flag for faster check
  VID m_lastNode = 0;  ///< last node of last operation, to save time

  ///@}
};

#endif

#ifndef ITERATION_COUNT_EVALUATOR_H_
#define ITERATION_COUNT_EVALUATOR_H_

#include "MapEvaluatorMethod.h"

#include <limits>


////////////////////////////////////////////////////////////////////////////////
/// Returns true once a minimum number of iterations has been achieved.
///
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class IterationCountEvaluator : public MapEvaluatorMethod<MPTraits> {

  public:
    ///@name Construction
    ///@{

    IterationCountEvaluator(const double _minCount = 5);

    IterationCountEvaluator(XMLNode& _node);

    virtual ~IterationCountEvaluator() = default;

    ///@}
    ///@name MPBaseObject Interface
    ///@{

    virtual void Initialize() override;

    void InitializeGlobal();

    ///@}
    ///@name MapEvaluatorMethod Interface
    ///@{

    virtual bool operator()() override;

    ///@}

  private:
    ///@name Internal State
    ///@{

    unsigned int m_minCount{0};
    unsigned int m_currentCount{0};
    bool m_minAchieved{false};
    bool m_neverAchieved{false};

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
IterationCountEvaluator<MPTraits>::
IterationCountEvaluator(const double _minCount) :
    MapEvaluatorMethod<MPTraits>(), m_minCount(_minCount) {
  this->SetName("IterationCountEvaluator");
}


template <typename MPTraits>
IterationCountEvaluator<MPTraits>::
IterationCountEvaluator(XMLNode& _node) : MapEvaluatorMethod<MPTraits>(_node) {
  this->SetName("IterationCountEvaluator");
  m_minCount = _node.Read("minCount", true, m_minCount, 1U,
                          std::numeric_limits<unsigned int>::max(),
                          "Number of ME calls before returning true.");
}

/*------------------------- MPBaseObject Interface ---------------------------*/

template <typename MPTraits>
void
IterationCountEvaluator<MPTraits>::
Initialize() {
  m_currentCount = 0;
  m_minAchieved = false;
  m_neverAchieved = false;
}

template <typename MPTraits>
void
IterationCountEvaluator<MPTraits>::
InitializeGlobal() {
  m_currentCount = 0;
  m_minAchieved = false;
  m_neverAchieved = true;
}

/*---------------------- MapEvaluatorMethod Interface ------------------------*/

template <typename MPTraits>
bool
IterationCountEvaluator<MPTraits>::
operator()() {
  if(m_neverAchieved)
    return false; 
  if(!m_minAchieved) {
    m_minAchieved = (++m_currentCount >= m_minCount);
  }
    

  return m_minAchieved;
}


/*----------------------------------------------------------------------------*/

#endif
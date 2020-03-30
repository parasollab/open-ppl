#ifndef PMPL_ITERATION_COUNT_EVALUATOR_H_
#define PMPL_ITERATION_COUNT_EVALUATOR_H_

#include "MapEvaluatorMethod.h"

#include <limits>


////////////////////////////////////////////////////////////////////////////////
/// Returns true once a minimum number of iterations has been achieved.
///
/// @warning This counts every time you call operator()() and won't work
///          properly with multiple sub-problems.
///
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class IterationCountEvaluator : public MapEvaluatorMethod<MPTraits> {

  public:

    ///@name Construction
    ///@{

    IterationCountEvaluator();

    IterationCountEvaluator(XMLNode& _node);

    virtual ~IterationCountEvaluator() = default;

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

    unsigned int m_minCount{0};      ///< Minimum iterations to return true.
    unsigned int m_currentCount{0};  ///< Current iteration count.

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
IterationCountEvaluator<MPTraits>::
IterationCountEvaluator() : MapEvaluatorMethod<MPTraits>() {
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
}

/*---------------------- MapEvaluatorMethod Interface ------------------------*/

template <typename MPTraits>
bool
IterationCountEvaluator<MPTraits>::
operator()() {
  return ++m_currentCount >= m_minCount;
}


/*----------------------------------------------------------------------------*/

#endif

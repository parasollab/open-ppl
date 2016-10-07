#ifndef MAP_EVALUATION_METHOD_H_
#define MAP_EVALUATION_METHOD_H_

#include "Utilities/MPUtils.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MapEvaluators
/// @brief Base algorithm abstraction for \ref MapEvaluators.
/// @tparam MPTraits Motion planning universe
///
/// MapEvaluatorMethod has one main function, @c operator(), which applies a
/// boolean pass/fail evaluation to a roadmap.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class MapEvaluatorMethod : public MPBaseObject<MPTraits> {
  public:

    MapEvaluatorMethod() = default;
    MapEvaluatorMethod(typename MPTraits::MPProblemType* _problem, XMLNode& _node)
      : MPBaseObject<MPTraits>(_problem, _node) {}
    virtual ~MapEvaluatorMethod() = default;

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Having state implies that a new roadmap needs to be loaded and
    ///        evaluated
    /// @return state/no state
    ///
    /// @c HasState() is called by strategies that start from an existing
    /// roadmap if HasState returns true, then the evaluator is called on the
    /// input roadmap to reset the state of the object. Note that most
    /// evaluators do not have state, so this is set to false by default.
    ////////////////////////////////////////////////////////////////////////////
    virtual bool HasState() const {return false;}

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Evaluate a roadmap
    /// @return pass/failed evaluation
    ///
    /// @usage
    /// @code
    /// MapEvaluatorPointer me = this->GetMPProblem()->GetMapEvaluator(m_meLabel);
    /// bool passed = (*me)(); //call as a function object
    /// bool passed2 = me->operator()(); //call with pointer notation
    /// @endcode
    ////////////////////////////////////////////////////////////////////////////
    virtual bool operator()() = 0;
};

#endif

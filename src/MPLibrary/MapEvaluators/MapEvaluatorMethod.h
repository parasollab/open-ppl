#ifndef MAP_EVALUATION_METHOD_H_
#define MAP_EVALUATION_METHOD_H_

#include "MPLibrary/MPBaseObject.h"
#include "Utilities/MPUtils.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MapEvaluators
/// @brief Base algorithm abstraction for \ref MapEvaluators.
///
/// MapEvaluatorMethod has one main function, @c operator(), which applies a
/// boolean pass/fail evaluation to a roadmap.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class MapEvaluatorMethod : public MPBaseObject<MPTraits> {

  public:

    ///@name Construction
    ///@{

    MapEvaluatorMethod() = default;
    MapEvaluatorMethod(XMLNode& _node) : MPBaseObject<MPTraits>(_node) {}
    virtual ~MapEvaluatorMethod() = default;

    ///@}
    ///@name Evaluation Interface
    ///@{

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
    /// MapEvaluatorPointer me = this->GetMapEvaluator(m_meLabel);
    /// bool passed = (*me)(); //call as a function object
    /// bool passed2 = me->operator()(); //call with pointer notation
    /// @endcode
    ////////////////////////////////////////////////////////////////////////////
    virtual bool operator()() = 0;

    ///@}
    ///@name Accessors and Modifiers
    ///@{

    /// Set the active robots.
    void SetActiveRobots(const std::vector<size_t> _activeRobots)
        { m_activeRobots = _activeRobots; }

    /// Get the active robots.
    std::vector<size_t> GetActiveRobots() const { return m_activeRobots; }

    ///@}


  protected:

    std::vector<size_t> m_activeRobots; ///< Active robots for group evaluators

};

#endif

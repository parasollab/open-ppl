#ifndef PPL_DRRT_QUERY_H_
#define PPL_DRRT_QUERY_H_

#include "MapEvaluatorMethod.h"

#include "MPLibrary/MPStrategies/GroupRRTStrategy.h"
#include "MPLibrary/MPStrategies/MPStrategyMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// Implements the dRRT algorithm. Searches the implicit tensor product roadmap
/// constructed from individual robot roadmaps.
///
/// Reference:
///   TODO
///
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class DRRT : public MapEvaluatorMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    ///@}
    ///@name Construction
    ///@{

    DRRT();
    
    DRRT(XMLNode& _node);

    virtual ~DRRT();

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Initialize() override;

    ///@}
    ///@name MapEvaluator Overrides
    ///@{

    virtual bool operator()() override;

    ///@}

  private:

    ///@name Helper Functions
    ///@{

    ///@}
    ///@name Internal State
    ///@{

    std::string m_rrtLabel;

    ///@}

};


/*----------------------------- Construction ---------------------------------*/

template <typename MPTraits>
DRRT<MPTraits>::
DRRT() {
  this->SetName("DRRT");
}

template <typename MPTraits>
DRRT<MPTraits>::
DRRT(XMLNode& _node) : MapEvaluatorMethod<MPTraits>(_node) {
  this->SetName("DRRT");

  m_rrtLabel = _node.Read("rrt", true, "", "Underlying RRT Strategy to use.");
}

template <typename MPTraits>
DRRT<MPTraits>::
~DRRT() { }

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
DRRT<MPTraits>::
Initialize() {

  // Check that the underlying RRT strategy is an rrt strategy and
  // that is has the Discrete Extender
  auto ms = this->GetMPStrategy(m_rrtLabel);
  auto rrt = dynamic_cast<GroupRRTStrategy<MPTraits>*>(ms);
  if(!rrt)
    throw RunTimeException(WHERE) << "dRRT Requires the underlying mp strategy "
                                     "to be a RRT variant.";

}

/*-------------------------- MapEvaluator Overrides --------------------------*/

template <typename MPTraits>
bool
DRRT<MPTraits>::
operator()() {

  auto ms = this->GetMPStrategy(m_rrtLabel);
  auto rrt = dynamic_cast<GroupRRTStrategy<MPTraits>*>(ms);

  (*rrt)();

  auto path = this->GetGroupPath();
  return !path->Empty();
}

/*----------------------------------------------------------------------------*/

#endif

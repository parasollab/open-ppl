#ifndef PMPL_REACHABILITY_UTIL_H_
#define PMPL_REACHABILITY_UTIL_H_

#include "MPLibrary/MPBaseObject.h"


////////////////////////////////////////////////////////////////////////////////
/// A utility that will generate the reachable set of a
///        given configuration.
///
/// @note A Reachable Set is a set of states that can reached with respect to
///       its current degrees of freedom
///
/// Reachability-Guided Sampling for Planning Under Differential Constraints
///     Alexander Shkolnik, Matthew Walter, and Russ Tedrake
///
/// @ingroup Utilities
////////////////////////////////////////////////////////////////////////////////
class ReachabilityUtil : public MPBaseObject {

  public:

    ///@name Motion Planning Types
    ///@{

    

    ///@}
    ///@name Local Types
    ///@{

    typedef std::vector<Cfg> ReachableSet;

    ///@}
    ///@name Construction
    ///@{

    ReachabilityUtil();

    ReachabilityUtil(XMLNode& _node);

    ///@}
    ///@name Overriden Base Methods
    ///@{

    virtual void Initialize() override;

    ///@}
    ///@name Utility Operator
    ///@{

    /// Generate a reachable set of a given configuration
    /// @param _cfg The extended configuration
    /// @return The set of cfgs which are reachable from _cfg.
    ReachableSet operator() (const Cfg& _cfg);

    ///@}

  private:

    ///@name Internal State
    ///@{

    std::string m_extenderLabel;

    // TODO: optimize with unordered map
    std::map<Cfg, ReachableSet> m_reachableSets; ///< computed reachable sets

    ///@}

};

#endif

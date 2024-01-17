#ifndef PMPL_EXTENDER_METHOD_H_
#define PMPL_EXTENDER_METHOD_H_

#include "MPLibrary/MPBaseObject.h"
#include "MPLibrary/LocalPlanners/LPOutput.h"
#include "MPLibrary/LocalPlanners/GroupLPOutput.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/CDInfo.h"
#include <limits>

////////////////////////////////////////////////////////////////////////////////
/// Base algorithm abstraction for \ref Extenders.
///
/// ExtenderMethod has one main method, @c Extend, to grow a simple path from a
/// starting node in some input direction - note that not all expansion
/// methods go in straight lines through @cspace.
///
/// @usage
/// @code
/// ExtenderPointer e = this->GetExtender(m_exLabel);
/// CfgType start, goal, new;
/// LPOutput lp;
/// bool pass = e->Extend(start, goal, new, lp);
/// @endcode
///
/// @todo Local planners and Extenders represent the same concepts and should be
///       merged into a single class with both an Extend and LocalPlan function.
///       This will help simplify several other objects within PMPL as well,
///       such as bi-directional RRT.
///
/// @ingroup Extenders
////////////////////////////////////////////////////////////////////////////////
class ExtenderMethod : public MPBaseObject {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPBaseObject::RoadmapType RoadmapType;
    typedef typename MPBaseObject::GroupCfgType GroupCfgType;
    typedef GroupLocalPlan<RoadmapType>      GroupWeightType;

    ///@}
    ///@name Construction
    ///@{

    ExtenderMethod() = default;

    ExtenderMethod(XMLNode& _node);

    virtual ~ExtenderMethod() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name Required Interface
    ///@{

    /// Get the minimum extension distance.
    virtual double GetMinDistance() const;

    /// Get the maximum extension distance.
    virtual double GetMaxDistance() const;

    /// Extends a local plan from a starting configuration towards a target
    /// configuration.
    /// @param _start Initial configuration to grow from.
    /// @param _end   Target configuration to grow towards.
    /// @param _new   Placeholder for resulting configuration.
    /// @param _lp    Placeholder for polygonal chain configurations for
    ///               non-straight-line extention operations and associated
    ///               weight.
    /// @return True if the extension produced a valid configuration that was
    ///         at least the minimum distance away from the starting point.
    virtual bool Extend(const Cfg& _start, const Cfg& _end,
        Cfg& _new, LPOutput& _lp) = 0;
    ///@example Extenders_UseCase.cpp
    /// This is an example of how to use the extender methods.

    /// An optional version if CDInfo is desired. Not required to implement.
    virtual bool Extend(const Cfg& _start, const Cfg& _end,
        Cfg& _new, LPOutput& _lp, CDInfo& _cdInfo);

    /// For groups.
    /// @override
    /// @param _robotIndexes The indexes of the robots which should move (empty
    ///                      for all).
    virtual bool Extend(const GroupCfgType& _start, const GroupCfgType& _end,
        GroupCfgType& _new, GroupLPOutput& _lp,
        const std::vector<size_t>& _robotIndexes = {});

    /// For groups, with CD info.
    /// @override
    /// @param _robotIndexes The indexes of the robots which should move (empty
    ///                      for all).
    virtual bool Extend(const GroupCfgType& _start, const GroupCfgType& _end,
        GroupCfgType& _new, GroupLPOutput& _lp, CDInfo& _cdInfo,
        const std::vector<size_t>& _robotIndexes = {});

    ///@}

  protected:

    ///@name Extender Properties
    ///@{

    double m_minDist{.1};  ///< The minimum valid extension distance.
    double m_maxDist{1.};  ///< The maximum valid extension distance.

    ///@}

};

#endif

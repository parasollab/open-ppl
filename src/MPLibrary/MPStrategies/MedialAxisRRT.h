#ifndef PMPL_MARRT_H
#define PMPL_MARRT_H

#include "MPLibrary/LocalPlanners/LPOutput.h"
#include "MPLibrary/NeighborhoodFinders/Neighbors.h"
#include "BasicRRTStrategy.h"
#include "Utilities/XMLNode.h"

// #include <iomanip>
// #include <iterator>
// #include <string>
// #include <unordered_set>
// #include <vector>

class MedialAxisRRT : public BasicRRTStrategy {

  public:
    typedef size_t VID;

    MedialAxisRRT();

    MedialAxisRRT(XMLNode& _node);

    virtual ~MedialAxisRRT();

    ///@}
    ///@name MPBaseObject overrides
    ///@{

    virtual void Print(std::ostream& _os) const;

    ///@}

  protected:
    /// Try to extend a new configuration toward a specific goal region. No-op
    /// if the goal is outside the extender's range.
    /// @param _newVID The VID of a newly extended configuration.
    /// @param _boundary The goal boundary.
    /// @note This only applies when not growing goals.
    virtual void TryGoalExtension(const VID _newVID, const Boundary* const _boundary) override;

    virtual VID ExtendWithGoalExtender(const VID _nearVID, const Cfg& _target,
    LPOutput& _lp, const bool _requireNew = true);

    /// @overload
    VID ExtendWithGoalExtender(const VID _nearVID, const Cfg& _target,
        const bool _requireNew = true);

  private:
    std::string m_goalExtender;  ///< label for extender that reaches to goal
};

#endif

#ifndef PMPL_SKELETON_CLEARANCE_UTILITY_H_
#define PMPL_SKELETON_CLEARANCE_UTILITY_H_

#include "MPLibrary/MPBaseObject.h"
#include "Utilities/XMLNode.h"
#include "Workspace/WorkspaceSkeleton.h"


////////////////////////////////////////////////////////////////////////////////
/// A utility for algorithms that improve the obstacle clearance of a workspace
/// skeleton.
////////////////////////////////////////////////////////////////////////////////
class SkeletonClearanceUtility final : public MPBaseObject {

  ///@name Motion Planning Types
  ///@{

  

  ///@}
  ///@name Internal State
  ///@{

  std::string m_mauLabel; ///< The label of the medial axis tool to use.

  ///@}

  public:

    ///@name Construction
    ///@{

    SkeletonClearanceUtility();

    SkeletonClearanceUtility(XMLNode& _node);

    virtual ~SkeletonClearanceUtility() = default;

    ///@}
    ///@name Skeleton Fix
    ///@{

    /// Fix the skeleton clearance by pushing vertices and edge points to the
    /// medial axis.
    /// @param _skeleton The skeleton to adjust.
    void operator()(WorkspaceSkeleton& _skeleton) const;

    /// Fix the skeleton clearance by attempting to push vertices and edge points
    /// away from nearby obstacles.
    /// @note This is a hacky way to fix clearance that we used in the WAFR'16
    ///       paper Dynamic Region RRT. A medial axis push is the preferred way to
    ///       do this - this version is retained for reference and reproduction of
    ///       prior work only. It should not be used in new methods.
    /// @param _skeleton The skeleton to adjust.
    void HackFix(WorkspaceSkeleton& _skeleton) const;

    ///@}

};

#endif

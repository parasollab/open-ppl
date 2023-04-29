#ifndef EDGE_VALIDITY_CHECKER_METHOD_H_
#define EDGE_VALIDITY_CHECKER_METHOD_H_

#include "Utilities/MPUtils.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup EdgeValidityCheckers
/// @Edge validity checkers validate an edge of the roadmap.
///
/// EdgeValidityCheckerMethod has one main method, @c ValidateEdge that takes
/// either two cfgs or to VIDs and validates the edge between them.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class EdgeValidityCheckerMethod : public MPBaseObject<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType                CfgType;
    typedef typename MPTraits::RoadmapType            RoadmapType;
    typedef typename RoadmapType::VID                 VID;
    typedef typename RoadmapType::EID                 EID;
    typedef typename RoadmapType::adj_edge_iterator   EI;

    ///@}
    ///@name Construction
    ///@{

    EdgeValidityCheckerMethod() = default;

    EdgeValidityCheckerMethod(XMLNode& _node);

    virtual ~EdgeValidityCheckerMethod() = default;

    void Initialize() = 0;
    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    ///@}
    ///@name EdgeValidityChecker Interface
    ///@{

    /// Checks the validity of an edge
    /// @param _u The source of the edge to validate
    /// @param _v The target of the edge to validate
    /// @param _collisions An empty container that will be populated with
    ///   a list of obstacles with which the edge collides.
    /// @return True if the edge between _u and _v does is valid.
    virtual bool ValidateEdge(VID _u, VID _v, vector<size_t>& _collisions) = 0;
    
    /// Cfg version of ValidateEdge 
    virtual bool ValidateEdge(CfgType& _c1, CfgType& _c2, vector<size_t>& _collisions) = 0;

    /// Setter for the report collision boolean
    /// @param _reportCollisions the new value of m_reportCollisions
    void SetReportCollisions(bool _reportCollisions){
      m_reportCollisions = _reportCollisions;
    }


  protected:
    ///@}
    ///@name Internal State
    ///@{

    bool m_reportCollisions{false};   ///< If true the list of obstacles in collision
                                      ///< will be reported

    ///@}
  private:



};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
EdgeValidityCheckerMethod<MPTraits>::
EdgeValidityCheckerMethod(XMLNode& _node) : MPBaseObject<MPTraits>(_node) {
}

/*------------------------- MPBaseObject Overrides ---------------------------*/


/*--------------------------- EdgeValidityChecker Interface -------------------------*/



/*----------------------------------------------------------------------------*/

#endif

#ifndef INTERMEDIATE_EDGE_VALIDITY_CHECKER_H_
#define INTERMEDIATE_EDGE_VALIDITY_CHECKER_H_

#include "EdgeValidityCheckerMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup EdgeValidityCheckers
/// @brief Validates an edge by calling a local planner and collision checking
/// all intermediates created by that local planner separately. 
/// 
/// @note Assumes that the validity checker is a collision detection validity
/// checker.
////////////////////////////////////////////////////////////////////////////////
class IntermediatesEdgeValidityChecker : public EdgeValidityCheckerMethod {

  public:

    typedef typename MPBaseObject::RoadmapType           RoadmapType;
    typedef typename RoadmapType::VID                VID;
    typedef typename RoadmapType::EID                EID;
    typedef typename RoadmapType::adj_edge_iterator  EI;

    IntermediatesEdgeValidityChecker();
    IntermediatesEdgeValidityChecker(XMLNode& _node);

    void Initialize() override;


    /// Checks the validity of an edge using a set of intermediate cfgs
    /// each validated independently.
    /// @param _u The source of the edge to validate
    /// @param _v The target of the edge to validate
    /// @param _collisions An empty container that will be populated with
    ///   a list of indices of obstacles with which the edge collides.
    /// @return True if all of the intermediate cfgs are in free space.
    bool ValidateEdge(VID _u, VID _v, std::vector<size_t>& _collisions) override;
    bool ValidateEdge(Cfg& _c1, Cfg& _c2, std::vector<size_t>& _collisions) override;

    /// Determines the clearance of the edge given by two VIDs.
    /// @param _u The source of the edge
    /// @param _v The target of the edge
    /// @return The clearance of the edge
    /// @note Takes into account weighted obstacles
    double EdgeWeightedClearance(VID _u, VID _v);
    double EdgeWeightedClearance(Cfg& _c1, Cfg& _c2);


  private:

    ///@name helper functions
    ///@{


    /// Calls a collision detector for a set of intermediate cfgs , and can 
    ///   populate the collisions vector with the indices of obstacles in collision
    ///   with these cfgs or report the clearance.
    /// @param _intermediates A vector of intermediate cfgs.
    /// @param _collisions A vector to be populated with the indices of obstacles in collision
    ///   with the cfgs.
    /// @return -/+ 1.0 to report invalid/valid if used for collision reporting, and
    ///   otherwise returns the weighted clearance of the edge.
    /// @note Clears the vector _collisions before populating it
    double HandleIntermediates(std::vector<Cfg>& _intermediates, std::vector<size_t>& _collisions, bool _reportClearance = false);

    /// @todo Maybe can get a nicer implementation using a clearance map of CDInfo
    /// Iterates over a set of intermediates in order to find the minimum weighted clearance
    /// @param _intermediates A vector of intermediate cfgs.
    /// @return The minimum weighted clearance
    double intermediateClearance(std::vector<Cfg>& _intermediates);

    ///@}
    ///@name member variables
    ///@{

    ///@}

    string m_lpLabel;                 ///< the local planner label

    string m_vcLabel;                 ///< the validity checker label. Must be a label of a
                                      ///<  collision detection validity checker.
    bool m_overrideLp{false};          ///< Flag for using a different local planner than the one
                                      ///<  stored in the roadmap edges
};

#endif

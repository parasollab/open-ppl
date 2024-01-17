#ifndef PMPL_BRUTE_FORCE_NF_H_
#define PMPL_BRUTE_FORCE_NF_H_

#include "NeighborhoodFinderMethod.h"

#include <queue>


////////////////////////////////////////////////////////////////////////////////
/// Determine the nearest neighbors with a brute force search.
///
/// This method does a direct distance check between the query configuration and
/// each input candidate.
///
/// @ingroup NeighborhoodFinders
////////////////////////////////////////////////////////////////////////////////
class BruteForceNF : virtual public NeighborhoodFinderMethod {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPBaseObject::RoadmapType   RoadmapType;
    typedef typename RoadmapType::VID            VID;
    typedef typename RoadmapType::VertexSet      VertexSet;
    typedef typename MPBaseObject::GroupRoadmapType  GroupRoadmapType;
    typedef typename MPBaseObject::GroupCfgType      GroupCfgType;

    ///@}
    ///@name Local Types
    ///@{

    using typename NeighborhoodFinderMethod::Type;
    using typename NeighborhoodFinderMethod::OutputIterator;

    ///@}
    ///@name Construction
    ///@{

    BruteForceNF();

    BruteForceNF(XMLNode& _node);

    virtual ~BruteForceNF() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name NeighborhoodFinderMethod Overrides
    ///@{

    virtual void FindNeighbors(RoadmapType* const _r, const Cfg& _cfg,
        const VertexSet& _candidates, OutputIterator _out) override;

    virtual void FindNeighbors(GroupRoadmapType* const _r,
        const GroupCfgType& _cfg, const VertexSet& _candidates,
        OutputIterator _out) override;

    ///@}

  protected:

    ///@name Helpers
    ///@{

    void FindNeighborsImpl(RoadmapType* const _r, const Cfg& _cfg,
        const VertexSet& _candidates, OutputIterator _out);

    void FindNeighborsImpl(GroupRoadmapType* const _r, const GroupCfgType& _cfg,
        const VertexSet& _candidates, OutputIterator _out);

    ///@}

};

#endif

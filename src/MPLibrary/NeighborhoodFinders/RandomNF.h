#ifndef PMPL_RANDOM_NF_H_
#define PMPL_RANDOM_NF_H_

#include "NeighborhoodFinderMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// Selects a set of random neighbors from the roadmap.
///
/// @note This assumes a fairly small k value compared to the candidate set
///       size, and will perform poorly otherwise.
///
/// @ingroup NeighborhoodFinders
////////////////////////////////////////////////////////////////////////////////
class RandomNF : virtual public NeighborhoodFinderMethod {
 public:
  ///@name Motion Planning Types
  ///@{

  typedef typename MPBaseObject::RoadmapType RoadmapType;
  typedef typename RoadmapType::VID VID;
  typedef typename RoadmapType::VertexSet VertexSet;
  typedef typename MPBaseObject::GroupRoadmapType GroupRoadmapType;
  typedef typename MPBaseObject::GroupCfgType GroupCfgType;

  ///@}
  ///@name Local Types
  ///@{

  using typename NeighborhoodFinderMethod::OutputIterator;
  using typename NeighborhoodFinderMethod::Type;

  ///@}
  ///@name Construction
  ///@{

  RandomNF();

  RandomNF(XMLNode& _node);

  virtual ~RandomNF() = default;

  ///@}
  ///@name MPBaseObject Overrides
  ///@{

  virtual void Print(std::ostream& _os) const override;

  ///@}
  ///@name NeighborhoodFinderMethod Overrides
  ///@{

  virtual void FindNeighbors(RoadmapType* const _r,
                             const Cfg& _cfg,
                             const VertexSet& _candidates,
                             OutputIterator _out) override;

  virtual void FindNeighbors(GroupRoadmapType* const _r,
                             const GroupCfgType& _cfg,
                             const VertexSet& _candidates,
                             OutputIterator _out) override;

  ///@}
};

#endif

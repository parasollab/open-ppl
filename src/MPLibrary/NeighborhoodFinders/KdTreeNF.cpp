#include "KdTreeNF.h"

#include "MPLibrary/MPLibrary.h"

/*------------------------------- Construction -------------------------------*/

KdTreeNF::
KdTreeNF() : NeighborhoodFinderMethod(Type::K) {
  this->SetName("KdTreeNF");
}


KdTreeNF::
KdTreeNF(XMLNode& _node) : NeighborhoodFinderMethod(_node, Type::K) {
  this->SetName("KdTreeNF");

  m_epsilon = _node.Read("epsilon", false, m_epsilon, 0., 100.,
      "Epsilon value for CGAL");
}

/*--------------------------- MPBaseObject Overrides -------------------------*/

void
KdTreeNF::
Initialize() {
  m_trees.clear();
  m_tmpTree.reset(new KdTree);

  // Force the dm to be a euclidean type because the kd tree will not be
  // consistent with it otherwise.
  if(this->GetMPLibrary()->GetDistanceMetric(this->m_dmLabel)->GetName() != "Euclidean")
    throw ParseException(WHERE) << "KdTreeNF requires a Euclidean distance "
                                << "metric to be consistent with the CGAL "
                                << "kd tree.";
}


void
KdTreeNF::
Print(std::ostream& _os) const {
  NeighborhoodFinderMethod::Print(_os);
  _os << "\tepsilon: " << m_epsilon
      << std::endl;
}

/*----------------------- NeighborhoodFinder Functions -----------------------*/

void
KdTreeNF::
FindNeighbors(RoadmapType* const _r, const Cfg& _cfg,
    const VertexSet& _candidates, OutputIterator _out) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::FindNeighbors");

  // Get the best Kd tree model.
  KdTree* const tree = GetModel(_r, _candidates);

  // Search for nearest neighbors.
  NeighborSearch search(*tree, _cfg, this->m_k, m_epsilon);

  auto dm = this->GetMPLibrary()->GetDistanceMetric(this->m_dmLabel);
  for(const auto& n : search) {
    // Get the VID.
    const VID vid = n.first.vid;

    // Check for self.
    const auto& node = _r->GetVertex(vid);
    if(node == _cfg)
      continue;

    // Compute distance and output neighbor.
    const double distance = dm->Distance(_cfg, node);
    _out = Neighbor(vid, distance);
  }
}


void
KdTreeNF::
FindNeighbors(GroupRoadmapType* const _r, const GroupCfgType& _cfg,
    const VertexSet& _candidates, OutputIterator _out) {
  throw NotImplementedException(WHERE);
}


/*---------------------------------- Helpers ---------------------------------*/

void
KdTreeNF::
SetupModels(RoadmapType* const _r) {
  // Create a kd tree for this roadmap.
  auto* tree = &(m_trees[ModelKey{_r, &_r->GetAllVIDs()}]);

  // Create a kd tree for each of its connected components.
  auto* ccTracker = _r->GetCCTracker();
  if(ccTracker) {
    for(const VertexSet* cc : *ccTracker)
      m_trees[ModelKey{_r, cc}];

    // Create a hook to add new CC tracker models.
    ccTracker->InstallCreatedHook(this->GetNameAndLabel(),
        [this, _r](const CCTrackerType* const, const VertexSet* const _cc) {
          this->m_trees[ModelKey{_r, _cc}];
        }
    );
    // Create a hook to remove CC tracker models.
    ccTracker->InstallDeletedHook(this->GetNameAndLabel(),
        [this, _r](const CCTrackerType* const, const VertexSet* const _cc) {
          this->m_trees.erase(ModelKey{_r, _cc});
        }
    );
    // Create a hook to merge CC tracker models.
    ccTracker->InstallMergedHook(this->GetNameAndLabel(),
        [this, _r](const CCTrackerType* const, const VertexSet* const _target,
            const VertexSet* const _source) {
          // Add all points in _source to the tree for _target.
          auto* targetTree = &this->m_trees.at(ModelKey{_r, _target});
          for(const VID vid : *_source)
            targetTree->insert(PointD(vid, _r->GetVertex(vid)));

          // Remove the tree for _source.
          this->m_trees.erase(ModelKey{_r, _source});
        }
    );
    // Create a hook to split CC tracker models.
    ccTracker->InstallBrokenHook(this->GetNameAndLabel(),
        [this, _r](const CCTrackerType* const, const VertexSet* const _source,
            const VertexSet* const _target) {
          // Create a new tree for the _target CC and move its points from the
          // _source tree to their new tree.
          auto* targetTree = &this->m_trees[ModelKey{_r, _target}],
              * sourceTree = &this->m_trees.at(ModelKey{_r, _source});
          for(const VID vid : *_target) {
            PointD p(vid, _r->GetVertex(vid));
            targetTree->insert(p);
            sourceTree->remove(p);
          }
        }
    );
  }

  // Create a function factory for adding vertices to the kd tree.
  typename RoadmapType::VertexHook adder = [tree, _r, this](const VI _vi) {
        const VID vid   = _vi->descriptor();
        const auto& cfg = _vi->property();
        PointD p(vid, cfg);

        // Add vertex to the full roadmap tree.
        tree->insert(PointD(vid, cfg));

        // Add vertex to its CC tree.
        auto* ccTracker = _r->GetCCTracker();
        if(ccTracker) {
          auto* cc     = ccTracker->GetCC(vid);
          auto* ccTree = &this->m_trees.at(ModelKey{_r, cc});
          ccTree->insert(p);
        }
      };
  // Create a function factory for deleting vertices from the kd tree.
  typename RoadmapType::VertexHook deleter = [tree, _r, this](const VI _vi) {
        const VID vid   = _vi->descriptor();
        const auto& cfg = _vi->property();
        PointD p(vid, cfg);

        // Remove vertex from the full roadmap tree.
        tree->remove(p);

        // Remove vertex from its CC tree.
        auto* ccTracker = _r->GetCCTracker();
        if(ccTracker) {
          auto* cc     = ccTracker->GetCC(vid);
          auto* ccTree = &this->m_trees.at(ModelKey{_r, cc});
          ccTree->remove(p);
        }
      };

  // Add each existing vertex to the set.
  for(auto vi = _r->begin(); vi != _r->end(); ++vi)
    adder(vi);

  // Each time we add a vertex, add it to the kd tree (the CGAL implementation
  // is internally buffered).
  _r->InstallHook(RoadmapType::HookType::AddVertex, this->GetNameAndLabel(),
      adder);
  // Each time we remove a vertex, remove it from the kd tree.
  _r->InstallHook(RoadmapType::HookType::DeleteVertex, this->GetNameAndLabel(),
      deleter);

}


typename KdTreeNF::KdTree*
KdTreeNF::
GetModel(RoadmapType* const _r, const VertexSet& _candidates) {
  // Set up models for this roadmap if needed.
  auto iter = m_trees.find(ModelKey{_r, &_r->GetAllVIDs()});
  if(iter == m_trees.end())
    SetupModels(_r);

  // If we already have a model for this roadmap/candidate set pair, use it.
  iter = m_trees.find(ModelKey{_r, &_candidates});
  if(iter != m_trees.end())
    return &iter->second;

  // Else we have no model. Build a temporary tree.
  /// @todo Questionable, why use O(n lg n) operation of building and
  ///       searching a new tree rather than brute force for O(n)? Somehow it
  ///       appears to provide better performance anyway, perhaps due to
  ///       CGAL's use of incremental orthogonal distance calculations?
  m_tmpTree->clear();

  for(const VID vid : _candidates)
    m_tmpTree->insert(PointD(vid, _r->GetVertex(vid)));

  return m_tmpTree.get();
}

/*----------------------------------------------------------------------------*/

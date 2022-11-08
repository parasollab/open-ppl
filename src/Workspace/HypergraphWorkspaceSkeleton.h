#ifndef PPL_HYPERGRAPH_WORKSPACE_SKELETON_H_
#define PPL_HYPERGRAPH_WORKSPACE_SKELETON_H_

#include "Utilities/Hypergraph.h"
#include "Workspace/WorkspaceSkeleton.h"
#include "ConfigurationSpace/CompositeGraph.h"

#include "Vector.h"

#include <queue>
#include <set>
#include <unordered_map>
#include <string>


////////////////////////////////////////////////////////////////////////////////
/// A HypergraphWorkspaceSkeleton is an aggregate of workspace skeletons
////////////////////////////////////////////////////////////////////////////////
template <typename Vertex, typename Hyperarc>
class HypergraphWorkspaceSkeleton : public Hypergraph<Vertex, Hyperarc> {

  public:

    ///@name Local Types
    ///@{

    typedef Hypergraph<Vertex, Hyperarc> BaseType;
    typedef size_t  VD;
    // typedef typename BaseType::VI   VI;
    // typedef typename BaseType::EI   EI;
    // typedef typename BaseType::adj_edge_iterator adj_edge_iterator;
    // typedef typename BaseType::vertex_iterator vertex_iterator;

    typedef std::vector<WorkspaceSkeleton*>      WorkspaceSkeletonSet;

    // typedef WorkspaceSkeleton::vertex_descriptor             VD;
    typedef WorkspaceSkeleton::edge_descriptor               ED;
    // typedef typename WorkspaceSkeleton::vertex_iterator      vertex_iterator;
    // typedef typename WorkspaceSkeleton::vertex_descriptor    vertex_descriptor;
    // typedef typename WorkspaceSkeleton::adj_edge_iterator    adj_edge_iterator;

    // typedef std::vector<vertex_iterator>        vertex_iterator_set;
    typedef std::vector<mathtool::Point3d>      PointSet;
    // typedef std::vector<VD>                     VDSet;

    ///@}
    ///@name Construction
    ///@{

    HypergraphWorkspaceSkeleton();

    HypergraphWorkspaceSkeleton(RobotGroup* const _g, WorkspaceSkeletonSet _skeletons);

    ///@}
    ///@name Locators
    ///@{

    /// Helper function to find the skeleton vertex that is closest to a given
    /// workspace point for each skeleton.
    /// @param _targets The workspace point for each skeleton.
    /// @return An vector of iterators to the nearest skeleton vertices.
    // vertex_iterator FindNearestVertex(const PointSet& _targets);

    ///@}
    ///@name Modifiers
    ///@{

    // Override this to add intermediates
    size_t AddHyperarc(std::set<size_t> _head, std::set<size_t> _tail,
                       Hyperarc _hyperarc, bool _overWrite = false) override;

    /// Overriding to suppress output
    virtual VD AddVertex(const Vertex& _v) noexcept override;

    void RefineEdges(double _maxLength);

    void DoubleEdges();

    ///@}

  private:
    ///@name Internal State
    ///@{

    RobotGroup* m_fullGroup{nullptr};

    std::map<Robot*, WorkspaceSkeleton> m_skeletons;

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename Vertex, typename Hyperarc>
HypergraphWorkspaceSkeleton<Vertex, Hyperarc>::
HypergraphWorkspaceSkeleton() :
  Hypergraph<Vertex, Hyperarc>() {}

template <typename Vertex, typename Hyperarc>
HypergraphWorkspaceSkeleton<Vertex, Hyperarc>::
HypergraphWorkspaceSkeleton(RobotGroup* const _g, WorkspaceSkeletonSet _skeletons) :
  m_fullGroup(_g), Hypergraph<Vertex, Hyperarc>() {

  auto robots = _g->GetRobots();
  for(size_t i = 0; i < robots.size(); i++) {
    m_skeletons.emplace(std::make_pair(robots[i], _skeletons[i]));
  }
}

/*--------------------------------- Locators ---------------------------------*/

///@todo need merging rules for this
// template <typename Vertex, typename Hyperarc>
// typename HypergraphWorkspaceSkeleton<Vertex, Hyperarc>::vertex_iterator
// HypergraphWorkspaceSkeleton<Vertex, Hyperarc>::
// FindNearestVertex(const PointSet& _targets) {
//   double closestDistance = std::numeric_limits<double>::max();
//   vertex_iterator closestVI;

//   for(auto vit = this->begin(); vit != this->end(); ++vit) {
//     double distance = 0;
//     for(size_t i = 0; i < _targets.size(); ++i)
//       distance += (vit->property().GetRobotCfg(i) - _targets[i]).norm();

//     if(distance < closestDistance) {
//       closestDistance = distance;
//       closestVI = vit;
//     }
//   }
//   return closestVI;
// }

/*-------------------------------- Modifiers ---------------------------------*/

template <typename Vertex, typename Hyperarc>
void
HypergraphWorkspaceSkeleton<Vertex, Hyperarc>::
RefineEdges(double _maxLength) {

  for (auto skeleton : m_skeletons) {
    skeleton.second.RefineEdges(_maxLength);
  }
}


template <typename Vertex, typename Hyperarc>
void
HypergraphWorkspaceSkeleton<Vertex, Hyperarc>::
DoubleEdges() {

  for (auto skeleton : m_skeletons) {
    skeleton.second.DoubleEdges();
  }
}

/*----------------------------------------------------------------------------*/

template <typename Vertex, typename Hyperarc>
size_t 
HypergraphWorkspaceSkeleton<Vertex,Hyperarc>::
AddHyperarc(std::set<size_t> _head, std::set<size_t> _tail,
            Hyperarc _hyperarc, bool _overWrite) {
  if(_head == _tail)
    throw RunTimeException(WHERE) << "Tried to add hyperarc between same head/tail "
                                  << _head << ".";

  // We need to adjust hyperarc, but we still want to override the base class
  // function, so make a local copy of the edge.
  Hyperarc edge = _hyperarc;

  // working on this -- need the hyperarcs to be composite edges, vertices to be composite vertices
  // Vector of edge descriptors, which are edges already in individual roadmaps
  auto& edgeDescriptors = _hyperarc.GetEdgeDescriptors();

  size_t numInactiveRobots = 0;

  std::map<Robot*, size_t> startVIDs;
  std::map<Robot*, size_t> targetVIDs;

  for(auto hid : _head) {
    auto cState = this->GetVertexType(hid);
    auto hRobots = cState.GetGroup();
    for(auto robot : hRobots) {
      if(startVIDs.count(robot))
        throw RunTimeException(WHERE) << "Tried to add multiple edges for " 
                                      << robot->GetLabel();
      startVIDs.emplace(std::make_pair(robot, cState.GetVID(robot)));
    }
  }


  for(auto hid : _tail) {
    auto cState = this->GetVertexType(hid);
    auto hRobots = cState.GetGroup();
    for(auto robot : hRobots) {
      if(targetVIDs.count(robot))
        throw RunTimeException(WHERE) << "Tried to add multiple edges for " 
                                      << robot->GetLabel();
      targetVIDs.emplace(std::make_pair(robot, cState.GetVID(robot)));
    }
  }

  // First, make sure all the local edges are in the individual roadmaps.
  ///@todo rework this to get start and targets from composite vids in head/tail then loop through and check
  auto robots = edge.GetGroup()->GetRobots();
  for(size_t i = 0; i < robots.size(); ++i) {
    auto roadmap = m_skeletons.at(robots[i]);

    const VD individualSourceVID = startVIDs.at(robots[i]),
             individualTargetVID = targetVIDs.at(robots[i]);

    // If they are the same, it means this is an inactive robot. Record the
    // number of these that occur so that we can ensure SOME robot(s) moved.
    if(individualSourceVID == individualTargetVID) {
      ++numInactiveRobots;
    }

    // Assert that the individual vertices exist.
    const bool verticesExist =
        roadmap->find_vertex(individualSourceVID) != roadmap->end() and
        roadmap->find_vertex(individualTargetVID) != roadmap->end();
    if(!verticesExist)
      throw RunTimeException(WHERE, "Cannot add edge for non-existent vertices.");

    // If we received a valid edge descriptor, it must agree with the individual
    // VIDs, and the individual edge must exist.
    const bool edgeExists = roadmap->IsEdge(individualSourceVID,
                                            individualTargetVID);
    if(edgeDescriptors[i] != INVALID_ED) {
      if(!edgeExists)
        throw RunTimeException(WHERE) << "Expected edge (" << individualSourceVID
                                      << ", " << individualTargetVID << ") does "
                                      << "not exist for robot "
                                      << roadmap->GetRobot()->GetLabel() << ".";

      const bool consistent = edgeDescriptors[i].source() == individualSourceVID
                          and edgeDescriptors[i].target() == individualTargetVID;
      if(!consistent)
        throw RunTimeException(WHERE) << "Edge descriptors are not consistent. "
                                      << "Fetched from group cfg: ("
                                      << individualSourceVID << ", "
                                      << individualTargetVID << "), fetched from "
                                      << "group edge: ("
                                      << edgeDescriptors[i].source() << ", "
                                      << edgeDescriptors[i].target() << ").";
    }
    // If not, assert that the edge to be added does not already exist and then
    // add it.
    else 
      throw RunTimeException(WHERE) << "Expected edge (" << individualSourceVID
                                      << ", " << individualTargetVID << ") does "
                                      << "not exist for robot "
                                      << roadmap->GetRobot()->GetLabel() << ".";
  }

  if(numInactiveRobots >= this->GetNumRobots())
    throw RunTimeException(WHERE) << "No robots were moved in this edge!";

  // Add Intermediates
  // Find the length of each individual edge and the maximum length
  // Intermediate length is five times the average robot diameter
  double diam = 0;
  for(auto robot : robots)
    diam += robot->GetMultiBody()->GetBoundingSphereRadius() * 2;
  double intLength = (diam / robots.size()) * 5;

  std::vector<Point3d> starts;
  std::vector<Point3d> displacements;
  double maxDist = 0;
  for(size_t i = 0; i < robots.size(); ++i) {
    auto start = m_skeletons.at(robots[i])->find_vertex(edgeDescriptors[i].source());
    auto target = m_skeletons.at(robots[i])->find_vertex(edgeDescriptors[i].target());
    starts.push_back(start->property());

    auto disp = target->property() - start->property();
    const auto dist = (disp).norm();

    if(dist < intLength)
      displacements.push_back({0., 0., 0.});
    else
      displacements.push_back(disp);
    
    if(dist > maxDist)
      maxDist = dist;
  }

  // Find the number of composite edge intermediates
  std::vector<Vertex> inters;
  int numInter = (int)ceil(maxDist/intLength);

  // Set the intermediate values along the edge
  for(int i = 0; i <= numInter; ++i){
    Vertex v = Vertex(this);

    for(size_t r = 0; r < robots.size(); r++) {
      Point3d d = {i * displacements[r][0]/numInter, 
                   i * displacements[r][1]/numInter,
                   i * displacements[r][2]/numInter};
      v.SetRobotCfg(r, starts[r] + d);
    }

    inters.push_back(v);
  }

  // Set the intermediates
  edge.SetIntermediates(inters);

  return BaseType::AddHyperarc(_head, _tail, edge, _overWrite);
}

/*----------------------------------------------------------------------------*/

#endif

#include "ReebGraphConstruction.h"

#include <queue>
#include <unordered_map>

#include <containers/sequential/graph/algorithms/dijkstra.h>

#include "TetGenDecomposition.h"

ReebGraphConstruction::
ReebGraphConstruction(TetGenDecomposition* _tetgen) {
  //Fill vertices vector
  size_t numPoints = _tetgen->GetNumPoints();
  const double* const points = _tetgen->GetPoints();
  m_vertices.reserve(numPoints);
  for(size_t i = 0; i < numPoints; ++i)
    m_vertices.emplace_back(&points[3*i]);

  //Fill triangle vector
  size_t numCorners = _tetgen->GetNumCorners();
  size_t numTetras = _tetgen->GetNumTetras();
  const int* const tetra = _tetgen->GetTetras();
  map<Triangle, unordered_set<size_t>> triangles;

  for(size_t i = 0; i < numTetras; ++i) {
    auto AddTriangle = [&](size_t _i, size_t _j, size_t _k) {
      int v[3] = {tetra[i*numCorners + _i], tetra[i*numCorners + _j], tetra[i*numCorners + _k]};
      sort(v, v+3);
      triangles[Triangle(v[0], v[1], v[2])].insert(i);
    };

    AddTriangle(0, 2, 1);
    AddTriangle(0, 3, 2);
    AddTriangle(0, 1, 3);
    AddTriangle(1, 2, 3);
  }

  m_triangles.reserve(triangles.size());
  copy(triangles.begin(), triangles.end(), back_inserter(m_triangles));

  //Compute ReebGraph
  Construct();
  Embed(_tetgen);
}

pair<ReebGraphConstruction::FlowGraph, size_t>
ReebGraphConstruction::
GetFlowGraph(const Vector3d& _p, double _posRes) {
  typedef FlowGraph::vertex_descriptor FVD;
  FlowGraph f;

  enum Color {White, Gray, Black};
  unordered_map<FVD, Color> visited;

  //add vertices of reeb graph and find closest
  double closestDist = numeric_limits<double>::max();
  FVD closestID = -1;
  for(auto vit = m_reebGraph.begin(); vit != m_reebGraph.end(); ++vit) {
    Vector3d v = vit->property().m_vertex;
    FVD vd = vit->descriptor();
    f.add_vertex(vd, v);
    visited[vd] = White;
    double dist = (v - _p).norm();
    if(dist < closestDist) {
      closestDist = dist;
      closestID = vd;
    }
  }

  //Specialized BFS to make flow network
  //
  //Differs from regular BFS because:
  //  - Treats ReebGraph as undirected graph even though it is directed
  //  - Computes a graph instead of BFS tree, i.e., cross edges are added
  queue<FVD> q;
  q.push(closestID);
  visited[closestID] = Gray;
  while(!q.empty()) {
    FVD u = q.front();
    q.pop();
    auto uit = m_reebGraph.find_vertex(u);

    //process outgoing edges
    for(auto eit = uit->begin(); eit != uit->end(); ++eit) {
      FVD v = eit->target();
      switch(visited[v]) {
        case White:
          visited[v] = Gray;
          q.push(v);
        case Gray:
          f.add_edge(eit->descriptor(), eit->property().m_path);
          break;
        default:
          break;
      }
    }

    //process incoming edges
    set<FVD> processed;
    for(auto pit = uit->predecessors().begin();
        pit != uit->predecessors().end(); ++pit) {
      FVD v = *pit;
      if(processed.count(v) == 0) {
        auto vit = m_reebGraph.find_vertex(v);
        for(auto eit = vit->begin(); eit != vit->end(); ++eit) {
          if(eit->target() == u) {
            switch(visited[v]) {
              case White:
                visited[v] = Gray;
                q.push(v);
              case Gray:
                {
                  vector<Vector3d>& opath = eit->property().m_path;
                  vector<Vector3d> path(opath.rbegin(), opath.rend());
                  f.add_edge(ReebGraph::edge_descriptor(u, v, eit->descriptor().id()), path);
                  break;
                }
              default:
                break;
            }
          }
        }
      }
      processed.emplace(v);
    }
    visited[u] = Black;
  }

  return make_pair(f, closestID);
}

void
ReebGraphConstruction::
Construct() {
  //////////////////////////////////////////////////////////////////////////////
  /// @brief Morse function over model (currently height)
  /// @param _v Vertex
  /// @todo Generalize to allow something other than height
  auto f = [&](const Vector3d& _v) {
    return _v[1];
  };

  //Add all vertices of tetrahedralization
  for(size_t i = 0; i < m_vertices.size(); ++i) {
    double w = f(m_vertices[i]);
    CreateNode(i, m_vertices[i], w);
  }

  //Precompute absolute ordering of vertices to optimize vertex comparison
  vector<size_t> order;
  for(size_t i = 0; i < m_vertices.size(); ++i)
    order.emplace_back(i);
  ReebNodeComp rnc;
  sort(order.begin(), order.end(), [&](size_t _a, size_t _b) {
        return rnc(m_reebGraph.find_vertex(_a)->property(), m_reebGraph.find_vertex(_b)->property());
      });
  for(size_t i = 0; i < order.size(); ++i)
    m_reebGraph.find_vertex(order[i])->property().m_order = i;

  //Precompute ordering of triangles proper usage of MergePaths function
  for(auto& t : m_triangles) {
    size_t v[3] = {get<0>(t.first), get<1>(t.first), get<2>(t.first)};
    sort(v, v+3, [&](size_t _a, size_t _b) {
        return m_reebGraph.find_vertex(_a)->property().m_order <
            m_reebGraph.find_vertex(_b)->property().m_order;
        });

    t.first = make_tuple(v[0], v[1], v[2]);
  }

  //Add each edge to ReebGraph as separate ReebArc
  for(auto& t : m_triangles) {
    size_t v[3] = {get<0>(t.first), get<1>(t.first), get<2>(t.first)};

    MeshEdge* e0 = CreateArc(v[0], v[2], t.second);
    MeshEdge* e1 = CreateArc(v[0], v[1], t.second);
    MeshEdge* e2 = CreateArc(v[1], v[2], t.second);

    ++e0->m_numTris;
    ++e1->m_numTris;
    ++e2->m_numTris;
  }

  //Call MergePaths on each triangle
  for(auto& t : m_triangles) {
    size_t v[3] = {get<0>(t.first), get<1>(t.first), get<2>(t.first)};

    MeshEdge* e0 = CreateArc(v[0], v[2]);
    MeshEdge* e1 = CreateArc(v[0], v[1]);
    MeshEdge* e2 = CreateArc(v[1], v[2]);

    MergePaths(e0, e1, e2);

    --e0->m_numTris;
    --e1->m_numTris;
    --e2->m_numTris;

    //If edge is done being processed - delete. This is an optimization
    if(e0->m_numTris == 0)
      DeleteMeshEdge(e0);
    if(e1->m_numTris == 0)
      DeleteMeshEdge(e1);
    if(e2->m_numTris == 0)
      DeleteMeshEdge(e2);
  }

  //Remove all 2nodes from graph
  Remove2Nodes();
}

void
ReebGraphConstruction::
CreateNode(size_t _i, const Vector3d& _v, double _w) {
  m_reebGraph.add_vertex(_i, ReebNode(_i, _v, _w));
}

ReebGraphConstruction::MeshEdge*
ReebGraphConstruction::
CreateArc(size_t _s, size_t _t, const unordered_set<size_t>& _tetra) {
  //If edge does not exist add an edge in both tetrahedron edges set and a
  //corresponding reeb arc.
  MeshEdge m(_s, _t, &m_reebGraph);
  if(!m_edges.count(&m)) {
    MeshEdge* m2 = new MeshEdge(_s, _t, &m_reebGraph);
    m_edges.insert(m2);
    RGEID eid = m_reebGraph.add_edge(_s, _t, ReebArc(_s, _t, m2));
    m2->m_arcs.insert(eid);
  }
  //Associate tetrahedrons with edge's reeb arcs
  MeshEdge* e = *m_edges.find(&m);
  for(auto& a : e->m_arcs) {
    ReebArc& ra = GetReebArc(a);
    ra.m_tetra.insert(_tetra.begin(), _tetra.end());
  }
  return e;
}

void
ReebGraphConstruction::
MergePaths(MeshEdge* _e0, MeshEdge* _e1, MeshEdge* _e2) {
  //Merge edges _e0 and _e1 of triangle
  ArcSet& a0 = _e0->m_arcs;
  ArcSet& a1 = _e1->m_arcs;
  GlueByMergeSorting(a0, _e0, a1, _e1);

  //Merge edges _e0 and _e2 of triangle
  ArcSet& a2 = _e0->m_arcs;
  ArcSet& a3 = _e2->m_arcs;
  GlueByMergeSorting(a2, _e0, a3, _e2);
}

void
ReebGraphConstruction::
GlueByMergeSorting(ArcSet& _a0, MeshEdge* _e0, ArcSet& _a1, MeshEdge* _e1) {
  ReebArcComp rac(&m_reebGraph);
  for(auto asit0 = _a0.begin(), asit1 = _a1.begin();
      asit0 != _a0.end() && asit1 != _a1.end();) {

    //Skip same edge
    if(*asit0 == *asit1) {
      ++asit0; ++asit1;
      continue;
    }

    //Not valid edges to merge continue
    if(asit0->source() != asit1->source()) {
      if(rac(*asit0, *asit1))
        ++asit0;
      else
        ++asit1;
      continue;
    }

    ReebNode& n0 = m_reebGraph.find_vertex(asit0->target())->property();
    ReebNode& n1 = m_reebGraph.find_vertex(asit1->target())->property();

    //Merge based on order of target of edges
    if(n0.m_order < n1.m_order) {
      MergeArcs(*asit0, *asit1);
      ++asit0;
      asit1 = _a1.begin();
    }
    else {
      MergeArcs(*asit1, *asit0);
      ++asit1;
      asit0 = _a0.begin();
    }
  }
}

void
ReebGraphConstruction::
MergeArcs(RGEID _a0, RGEID _a1) {
  ReebArc& a0 = GetReebArc(_a0);
  ReebArc& a1 = GetReebArc(_a1);

  //Merge data into a0
  for(auto& edge : a1.m_edges) {
    edge->m_arcs.insert(_a0);
    edge->m_arcs.erase(_a1);
    a0.m_edges.insert(edge);
  }
  a0.m_tetra.insert(a1.m_tetra.begin(), a1.m_tetra.end());

  //Case: targets are not equal. Insert new edge from a0's target to a1's target
  if(_a0.target() != _a1.target()) {
    ReebArc a = a1;
    a.m_source = a0.m_target;
    RGEID neweid = m_reebGraph.add_edge(a.m_source, a.m_target, a);
    for(auto& edge : GetReebArc(neweid).m_edges)
      edge->m_arcs.insert(neweid);
  }

  //Delete a1
  m_reebGraph.delete_edge(_a1);
}

ReebGraphConstruction::ReebArc&
ReebGraphConstruction::
GetReebArc(RGEID _a) {
  ReebGraph::adj_edge_iterator ei;
  ReebGraph::vertex_iterator vi;
  m_reebGraph.find_edge(_a, vi, ei);
  return ei->property();
}

void
ReebGraphConstruction::
DeleteMeshEdge(MeshEdge* _e) {
  for(auto& a : _e->m_arcs)
    GetReebArc(a).m_edges.erase(_e);
}

void
ReebGraphConstruction::
Remove2Nodes() {
  bool removed;
  do {
    removed = false;
    for(auto vit = m_reebGraph.begin(); vit != m_reebGraph.end(); ++vit) {
      //detect 2-node
      if(vit->predecessors().size() == 1 && vit->size() == 1) {
        //get in and out edge
        auto pred = m_reebGraph.find_vertex(vit->predecessors()[0]);
        RGEID in, out;
        for(auto eit : *pred) {
          if(eit.target() == vit->descriptor())
            in = eit.descriptor();
        }
        out = (*vit->begin()).descriptor();

        ReebArc& ain = GetReebArc(in);
        ReebArc& aout = GetReebArc(out);

        //Merge arcs
        RGEID neweid = m_reebGraph.add_edge(ain.m_source, aout.m_target, ReebArc(ain.m_source, aout.m_target));

        ReebArc& newa = GetReebArc(neweid);
        for(auto& edge : ain.m_edges) {
          edge->m_arcs.insert(neweid);
          edge->m_arcs.erase(in);
          newa.m_edges.insert(edge);
        }
        newa.m_tetra.insert(ain.m_tetra.begin(), ain.m_tetra.end());
        for(auto& edge : aout.m_edges) {
          edge->m_arcs.insert(neweid);
          edge->m_arcs.erase(out);
          newa.m_edges.insert(edge);
        }
        newa.m_tetra.insert(aout.m_tetra.begin(), aout.m_tetra.end());

        //Delete old edges and delete old vertex
        m_reebGraph.delete_edge(in);
        m_reebGraph.delete_edge(out);
        m_reebGraph.delete_vertex(vit->descriptor());
        --vit;
        removed = true;
      }
    }
  } while(removed);
}

void
ReebGraphConstruction::
Embed(TetGenDecomposition* _tetgen) {
  size_t numCorners = _tetgen->GetNumCorners();
  const int* const tetras = _tetgen->GetTetras();
  TetGenDecomposition::DualGraph& tetraGraph = _tetgen->GetDualGraph();

  //embed ReebNodes in tetrahedralization by finding its closest tetrahedron
  for(auto nit = m_reebGraph.begin(); nit != m_reebGraph.end(); ++nit) {
    Vector3d& vn = nit->property().m_vertex;
    double minDist = numeric_limits<double>::max();
    Vector3d closest;
    size_t closestID = -1;
    for(auto tit = tetraGraph.begin(); tit != tetraGraph.end(); ++tit) {
      Vector3d& vt = tit->property();
      double dist = (vt - vn).norm();
      if(dist < minDist) {
        minDist = dist;
        closest = vt;
        closestID = tit->descriptor();
      }
    }
    nit->property().m_vertex = closest;
    nit->property().m_tetra = closestID;
  }

  //embed ReebArcs in tetrahedralization
  for(auto eit = m_reebGraph.edges_begin(); eit != m_reebGraph.edges_end(); ++eit) {
    ReebArc& ra = eit->property();

    auto WeightGraph = [&](double _factor) {
      for(auto& tetraid : ra.m_tetra) {
        auto vit = tetraGraph.find_vertex(tetraid);
        for(auto eit = vit->begin(); eit != vit->end(); ++eit) {
          auto targetit = tetraGraph.find_vertex(eit->target());
          eit->property() = _factor * (vit->property() - targetit->property()).norm();
        }
      }
    };

    //weight all tetrahedron on reeb arc to bias search
    WeightGraph(0.01);

    //find path
    auto sourceit = m_reebGraph.find_vertex(eit->source());
    auto targetit = m_reebGraph.find_vertex(eit->target());

    vector<size_t> pathVID;
    stapl::sequential::find_path_dijkstra(tetraGraph,
        sourceit->property().m_tetra, targetit->property().m_tetra,
        pathVID, numeric_limits<double>::max());

    ra.m_path.clear();

    // for each tetrahedron pair in path, find common triangle and insert middle
    // of triangle into path for proper transition between the two tetrahedron
    for(auto vit1 = pathVID.begin(), vit2 = vit1 + 1;
        vit2 != pathVID.end(); ++vit1, ++vit2) {
      Vector3d& v1 = tetraGraph.find_vertex(*vit1)->property();
      ra.m_path.push_back(v1);

      int t1[4];
      copy(&tetras[(*vit1)*numCorners], &tetras[(*vit1)*numCorners + 4], t1);
      int t2[4];
      copy(&tetras[(*vit2)*numCorners], &tetras[(*vit2)*numCorners + 4], t2);

      int tcommon[3];
      size_t j = 0;
      for(size_t i = 0; i < 4; ++i) {
        int* f = find(t2, t2+4, t1[i]);
        if(f != t2 + 4)
          tcommon[j++] = *f;
      }

      Vector3d c;
      for(size_t i = 0; i < 3; ++i)
        c += m_vertices[tcommon[i]];
      c /= 3;

      ra.m_path.push_back(c);
    }
    ra.m_path.push_back(tetraGraph.find_vertex(pathVID.back())->property());

    //unweight all
    WeightGraph(1);
  }
}


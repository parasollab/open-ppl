#include "TetGenDecomposition.h"

#include "PMPLExceptions.h"

#include "Environment/BoundingBox.h"
#include "Environment/BoundingSphere.h"
#include "Environment/Environment.h"
#include "Environment/FixedBody.h"
#include "Environment/StaticMultiBody.h"

TetGenDecomposition::
TetGenDecomposition(string _switches,
    bool _writeFreeModel, bool _writeDecompModel) :
  m_env(nullptr),
  m_freeModel(new tetgenio()),
  m_decompModel(new tetgenio()),
  m_switches(_switches),
  m_writeFreeModel(_writeFreeModel),
  m_writeDecompModel(_writeDecompModel) {
  }

TetGenDecomposition::
TetGenDecomposition(XMLNode& _node) :
  m_env(nullptr),
  m_freeModel(new tetgenio()),
  m_decompModel(new tetgenio()) {
    _node.Read("switches", false, "pqnQ",
        "TetGen input parameters. See TetGen manual. Need 'pn' at a minimum");
    _node.Read("writeFreeModel", false, false,
        "Output TetGen model of workspace");
    _node.Read("writeDecompModel", false, false,
        "Output TetGen model of tetrahedralization");
  }

TetGenDecomposition::
~TetGenDecomposition() {
  delete m_freeModel;
  delete m_decompModel;
}

void
TetGenDecomposition::
Decompose(Environment* _env) {
  m_env = _env;

  //make in tetgenio - this is a model of free workspace to decompose
  InitializeFreeModel();
  MakeFreeModel();
  if(m_writeFreeModel)
    SaveFreeModel();

  //decompose with tetgen
  tetrahedralize(const_cast<char*>(m_switches.c_str()),
      m_freeModel, m_decompModel);
  if(m_writeDecompModel)
    SaveDecompModel();

  MakeDualGraph();

  m_env = nullptr;
}

void
TetGenDecomposition::
InitializeFreeModel() {
  m_freeModel->numberofpoints = GetNumVertices();
  m_freeModel->pointlist = new double[m_freeModel->numberofpoints * 3];

  m_freeModel->numberoffacets = GetNumFacets();
  m_freeModel->facetlist = new tetgenio::facet[m_freeModel->numberoffacets];
  m_freeModel->facetmarkerlist = new int[m_freeModel->numberoffacets];

  m_freeModel->numberofholes = GetNumHoles();
  m_freeModel->holelist = new double[m_freeModel->numberofholes * 3];
}

size_t
TetGenDecomposition::
GetNumVertices() const {
  size_t numVerts = GetNumVertices(m_env->GetBoundary());
  for(size_t i = 0; i < m_env->NumObstacles(); ++i)
    if(!m_env->GetObstacle(i)->IsInternal())
      numVerts += GetNumVertices(m_env->GetObstacle(i));
  return numVerts;
}

size_t
TetGenDecomposition::
GetNumVertices(const shared_ptr<StaticMultiBody>& _obst) const {
  return _obst->GetFixedBody(0)->GetPolyhedron().m_vertexList.size();
}

size_t
TetGenDecomposition::
GetNumVertices(const shared_ptr<Boundary>& _boundary) const {
  if(_boundary->Type() == "Box")
    return 8;
  else if(_boundary->Type() == "Sphere")
    throw RunTimeException(WHERE, "BoundingSphere not supported yet in TetGen Decomposition");
  else
    throw RunTimeException(WHERE, "Impossibility. Give up.");
}

size_t
TetGenDecomposition::
GetNumFacets() const {
  size_t numFacets = GetNumFacets(m_env->GetBoundary());
  for(size_t i = 0; i < m_env->NumObstacles(); ++i)
    if(!m_env->GetObstacle(i)->IsInternal())
      numFacets += GetNumFacets(m_env->GetObstacle(i));
  return numFacets;
}

size_t
TetGenDecomposition::
GetNumFacets(const shared_ptr<StaticMultiBody>& _obst) const {
  return _obst->GetFixedBody(0)->GetPolyhedron().m_polygonList.size();
}

size_t
TetGenDecomposition::
GetNumFacets(const shared_ptr<Boundary>& _boundary) const {
  if(_boundary->Type() == "Box")
    return 6;
  else if(_boundary->Type() == "Sphere")
    throw RunTimeException(WHERE, "BoundingSphere not supported yet in TetGen Decomposition");
  else
    throw RunTimeException(WHERE, "Impossibility. Give up.");
}

size_t
TetGenDecomposition::
GetNumHoles() const {
  size_t numHoles = 0;
  for(size_t i = 0; i < m_env->NumObstacles(); ++i)
    if(!m_env->GetObstacle(i)->IsInternal())
      ++numHoles;
  return numHoles;
}

void
TetGenDecomposition::
MakeFreeModel() {
  size_t pointOffset = 0;
  size_t facetOffset = 0;
  size_t holeOffset = 0;
  for(size_t i = 0; i < m_env->NumObstacles(); ++i) {
    shared_ptr<StaticMultiBody> obst = m_env->GetObstacle(i);
    if(!obst->IsInternal()) {
      AddToFreeModel(obst, pointOffset, facetOffset, holeOffset);
      pointOffset += GetNumVertices(obst);
      facetOffset += GetNumFacets(obst);
      ++holeOffset;
    }
  }
  AddToFreeModel(m_env->GetBoundary(), pointOffset, facetOffset);
}

void
TetGenDecomposition::
AddToFreeModel(const shared_ptr<StaticMultiBody>& _obst,
    size_t _pOff, size_t _fOff, size_t _hOff) {

  shared_ptr<FixedBody> body = _obst->GetFixedBody(0);
  GMSPolyhedron& poly = body->GetWorldPolyhedron();

  //Add vertices to tetModel
  size_t n = 0;
  for(auto& v : poly.m_vertexList) {
    m_freeModel->pointlist[3*_pOff + 3*n + 0] = v[0];
    m_freeModel->pointlist[3*_pOff + 3*n + 1] = v[1];
    m_freeModel->pointlist[3*_pOff + 3*n + 2] = v[2];
    ++n;
  }

  //Add triagles as facets to tetModel
  size_t m = 0;
  for(auto& t : poly.m_polygonList) {
    tetgenio::facet* f = &m_freeModel->facetlist[_fOff + m];
    m_freeModel->facetmarkerlist[_fOff + m] = _hOff + 1;
    f->numberofpolygons = 1;
    f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
    f->numberofholes = 0;
    f->holelist = NULL;
    tetgenio::polygon* p = &f->polygonlist[0];
    p->numberofvertices = t.m_vertexList.size();
    p->vertexlist = new int[p->numberofvertices];
    size_t i = 0;
    for(auto& v : t.m_vertexList)
      p->vertexlist[i++] = _pOff + v;
    ++m;
  }

  //Add hole to tetModel
  //
  //Use the furthest vertex from the center of mass purturbed by an epsilon
  //distance towards center
  Vector3d com = body->GetCenterOfMass();
  Vector3d hole = com;
  GMSPolyhedron& modpoly = body->GetPolyhedron();
  for(auto& v : modpoly.m_vertexList) {
    if(body->IsConvexHullVertex(v)) {
      if((v - com).norm() > (hole - com).norm())
        hole = v;
    }
  }
  body->ComputeCenterOfMass();
  hole = body->GetWorldTransformation() * hole;
  hole = hole + (com - hole).normalize() * 0.0001;
  m_freeModel->holelist[3*_hOff + 0] = hole[0];
  m_freeModel->holelist[3*_hOff + 1] = hole[1];
  m_freeModel->holelist[3*_hOff + 2] = hole[2];
}

void
TetGenDecomposition::
AddToFreeModel(const shared_ptr<Boundary>& _boundary,
    size_t _pOff, size_t _fOff) {
  if(_boundary->Type() == "Box")
    AddToFreeModel(dynamic_pointer_cast<BoundingBox>(_boundary), _pOff, _fOff);
  else if(_boundary->Type() == "Sphere")
    AddToFreeModel(dynamic_pointer_cast<BoundingSphere>(_boundary), _pOff, _fOff);
  else
    throw RunTimeException(WHERE, "Impossibility. Give up.");
}

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/centroid.h>
typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel2;
typedef Kernel2::Point_2                                  Point2;
typedef CGAL::Polygon_2<Kernel2>                          Polygon2;
typedef CGAL::Polygon_with_holes_2<Kernel2>               PolygonWithHoles2;
typedef std::vector<PolygonWithHoles2>                    PWH2Set;

////////////////////////////////////////////////////////////////////////////////
/// @brief Isolated helper function for cutting hole into boundary facet
/// @param _tetModel Free workspace model
/// @param _verts Vertex indices of boundary facet
/// @param _holes Coplanar facets to cut from boundary facet
/// @return New polygons and holes to add as boundary facets
///
/// Use CGAL to compute facet differences
vector<pair<vector<size_t>, vector<pair<vector<size_t>, Vector3d>>>>
Merge(tetgenio* _tetModel, vector<size_t>& _verts,
    vector<pair<vector<size_t>, Vector3d>> _holes) {

  Vector3d v0(&_tetModel->pointlist[3*_verts[0]]);
  Vector3d v1(&_tetModel->pointlist[3*_verts[1]]);
  Vector3d v2(&_tetModel->pointlist[3*_verts[2]]);
  Vector3d vx = (v0 - v1).normalize();
  Vector3d vy = (v2 - v1).normalize();

  //////////////////////////////////////////////////////////////////////////////
  /// @param _p Vertex
  /// @return 2D CGAL point of vertex
  auto planeProj = [&](Vector3d& _p) {
    double x = (_p - v1).proj(vx).norm();
    double y = (_p - v1).proj(vy).norm();
    return Point2(x, y);
  };

  //////////////////////////////////////////////////////////////////////////////
  /// @param _vs Facet
  /// @return CGAL polygon of facet
  auto makePolygon2 = [&](vector<size_t>& _vs) {
    Polygon2 p;
    for(auto& x : _vs) {
      Vector3d v(&_tetModel->pointlist[3*x]);
      p.push_back(planeProj(v));
    }
    return p;
  };

  //////////////////////////////////////////////////////////////////////////////
  /// @param _p 2D CGAL point
  /// @return Unprojected 3D point
  auto planeUnproj = [&](const Point2& _p) {
    return v1 + vx*to_double(_p[0]) + vy*to_double(_p[1]);
  };

  //////////////////////////////////////////////////////////////////////////////
  /// @param _p Unprojected 3D point
  /// @return Closest vertex in TetGen model
  auto getVert = [&](const Vector3d& _p) {
    size_t closest = -1;
    double dist = 1e6;
    for(size_t i = 0; i < (size_t)_tetModel->numberofpoints; ++i) {
      Vector3d p2(&_tetModel->pointlist[3*i]);
      double d = (_p-p2).norm();
      if(d < dist) {
        closest = i;
        dist = d;
      }
    }
    return closest;
  };

  //Progessively make a polygon from coplanar facet and use CGAL to compute
  //polygon difference with overall boundary facet
  Polygon2 bounds = makePolygon2(_verts);

  PWH2Set diff;
  diff.push_back(PolygonWithHoles2(bounds));

  PWH2Set diff2;
  for(size_t i = 0; i < _holes.size(); ++i) {
    Polygon2 hole = makePolygon2(_holes[i].first);
    for(auto& p : diff)
      CGAL::difference(p, hole, back_inserter(diff2));
    diff.swap(diff2);
    diff2.clear();
  }

  //Aggregate results into final set of polygons and holes for tetgenio
  vector<pair<vector<size_t>, vector<pair<vector<size_t>, Vector3d>>>> polys;
  for(auto& p : diff) {
    vector<size_t> ext;
    vector<pair<vector<size_t>, Vector3d>> holes;
    Polygon2 outer = p.outer_boundary();
    for(auto pit = outer.vertices_begin(); pit != outer.vertices_end(); ++pit) {
      ext.push_back(getVert(planeUnproj(*pit)));
    }
    for(auto hit = p.holes_begin(); hit != p.holes_end(); ++hit) {
      vector<size_t> hole;
      for(auto pit = hit->vertices_begin(); pit != hit->vertices_end(); ++pit) {
        hole.push_back(getVert(planeUnproj(*pit)));
      }
      Point2 cent = CGAL::centroid(hit->vertices_begin(), hit->vertices_end());
      Vector3d c = planeUnproj(cent);
      holes.push_back(make_pair(hole, c));
    }
    polys.push_back(make_pair(ext, holes));
  }

  return polys;
}

void
TetGenDecomposition::
AddToFreeModel(const shared_ptr<BoundingBox>& _boundary,
    size_t _pOff, size_t _fOff) {

  const pair<double, double>* const ranges = _boundary->GetBox();

  //Add points of boundary
  m_freeModel->pointlist[3*_pOff +  0] = ranges[0].first;
  m_freeModel->pointlist[3*_pOff +  1] = ranges[1].first;
  m_freeModel->pointlist[3*_pOff +  2] = ranges[2].first;
  m_freeModel->pointlist[3*_pOff +  3] = ranges[0].first;
  m_freeModel->pointlist[3*_pOff +  4] = ranges[1].first;
  m_freeModel->pointlist[3*_pOff +  5] = ranges[2].second;
  m_freeModel->pointlist[3*_pOff +  6] = ranges[0].first;
  m_freeModel->pointlist[3*_pOff +  7] = ranges[1].second;
  m_freeModel->pointlist[3*_pOff +  8] = ranges[2].first;
  m_freeModel->pointlist[3*_pOff +  9] = ranges[0].first;
  m_freeModel->pointlist[3*_pOff + 10] = ranges[1].second;
  m_freeModel->pointlist[3*_pOff + 11] = ranges[2].second;
  m_freeModel->pointlist[3*_pOff + 12] = ranges[0].second;
  m_freeModel->pointlist[3*_pOff + 13] = ranges[1].first;
  m_freeModel->pointlist[3*_pOff + 14] = ranges[2].first;
  m_freeModel->pointlist[3*_pOff + 15] = ranges[0].second;
  m_freeModel->pointlist[3*_pOff + 16] = ranges[1].first;
  m_freeModel->pointlist[3*_pOff + 17] = ranges[2].second;
  m_freeModel->pointlist[3*_pOff + 18] = ranges[0].second;
  m_freeModel->pointlist[3*_pOff + 19] = ranges[1].second;
  m_freeModel->pointlist[3*_pOff + 20] = ranges[2].first;
  m_freeModel->pointlist[3*_pOff + 21] = ranges[0].second;
  m_freeModel->pointlist[3*_pOff + 22] = ranges[1].second;
  m_freeModel->pointlist[3*_pOff + 23] = ranges[2].second;

  //////////////////////////////////////////////////////////////////////////////
  /// @brief Add facet of bounding box to free model
  /// @param _fIndx Facet index
  /// @param _v0 Vertex 0 of facet
  /// @param _v1 Vertex 1 of facet
  /// @param _v2 Vertex 2 of facet
  /// @param _v3 Vertex 3 of facet
  auto addFacet = [&](size_t _fIndx, size_t _v0, size_t _v1, size_t _v2, size_t _v3) {
    tetgenio::facet* f;
    tetgenio::polygon* p;
    f = &m_freeModel->facetlist[_fIndx];
    m_freeModel->facetmarkerlist[_fIndx] = 0;

    //for each facet of free model so far determine if lies on this facet of
    //boundary. Collect coplanar facet as hole.
    vector<pair<vector<size_t>, Vector3d>> holes;
    vector<size_t> verts = {_pOff + _v0, _pOff + _v1, _pOff + _v2, _pOff + _v3};
    Vector3d x3(&m_freeModel->pointlist[3*verts[0]]);
    Vector3d x4(&m_freeModel->pointlist[3*verts[1]]);
    Vector3d x5(&m_freeModel->pointlist[3*verts[2]]);
    Vector3d x6(&m_freeModel->pointlist[3*verts[3]]);
    for(size_t i = 0; i < _fOff; ++i) {
      vector<size_t> tri(3);
      tri[0] = m_freeModel->facetlist[i].polygonlist[0].vertexlist[0];
      tri[1] = m_freeModel->facetlist[i].polygonlist[0].vertexlist[1];
      tri[2] = m_freeModel->facetlist[i].polygonlist[0].vertexlist[2];
      Vector3d x0(&m_freeModel->pointlist[3*tri[0]]);
      Vector3d x1(&m_freeModel->pointlist[3*tri[1]]);
      Vector3d x2(&m_freeModel->pointlist[3*tri[2]]);
      Vector3d x20 = (x2-x0).normalize();
      Vector3d x10 = (x1-x0).normalize();
      //Coplanar test
      if(
          fabs(x20*(x10%(x3-x2).normalize())) < 0.00000001 &&
          fabs(x20*(x10%(x4-x2).normalize())) < 0.00000001 &&
          fabs(x20*(x10%(x5-x2).normalize())) < 0.00000001 &&
          fabs(x20*(x10%(x6-x2).normalize())) < 0.00000001
          ) {
        holes.push_back(make_pair(tri, (x0 + x1 + x2)/3));
      }
    }

    //Merge facet holes into aggregate polygon
    auto polys = Merge(m_freeModel, verts, holes);

    //Add corrected boundary facet to free model
    size_t numHoles = 0;
    for(auto& poly : polys)
      numHoles += poly.second.size();
    size_t numPoly = polys.size() + numHoles;

    f->numberofpolygons = numPoly;
    f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
    f->numberofholes = numHoles;
    f->holelist = new double[3*f->numberofholes];

    size_t polyIndx = 0;
    size_t holeIndx = 0;
    for(auto& poly : polys) {
      p = &f->polygonlist[polyIndx++];
      p->numberofvertices = poly.first.size();
      p->vertexlist = new int[p->numberofvertices];
      for(size_t i = 0; i < poly.first.size(); ++i)
        p->vertexlist[i] = poly.first[i];

      //add each hole in facet
      for(auto& h : poly.second) {
        p = &f->polygonlist[polyIndx++];
        p->numberofvertices = h.first.size();
        p->vertexlist = new int[p->numberofvertices];
        for(size_t i = 0; i < h.first.size(); ++i)
          p->vertexlist[i] = h.first[i];
        f->holelist[3*holeIndx + 0] = h.second[0];
        f->holelist[3*holeIndx + 1] = h.second[1];
        f->holelist[3*holeIndx + 2] = h.second[2];
        ++holeIndx;
      }
    }
  };

  addFacet(_fOff + 0, 0, 1, 3, 2);
  addFacet(_fOff + 1, 0, 2, 6, 4);
  addFacet(_fOff + 2, 4, 5, 7, 6);
  addFacet(_fOff + 3, 5, 1, 3, 7);
  addFacet(_fOff + 4, 7, 3, 2, 6);
  addFacet(_fOff + 5, 0, 4, 5, 1);
}

void
TetGenDecomposition::
AddToFreeModel(const shared_ptr<BoundingSphere>& _boundary,
    size_t _pOff, size_t _fOff) {
  throw RunTimeException(WHERE, "BoundingSphere not supported yet in TetGen Decomposition");
}

void
TetGenDecomposition::
SaveFreeModel() {
  m_freeModel->save_nodes((char*)"freespace");
  m_freeModel->save_poly((char*)"freespace");
}

void
TetGenDecomposition::
SaveDecompModel() {
  m_decompModel->save_nodes((char*)"decomposed");
  m_decompModel->save_elements((char*)"decomposed");
  m_decompModel->save_faces((char*)"decomposed");
}

void
TetGenDecomposition::
MakeDualGraph() {
  m_dualGraph.clear();
  size_t numTetras = m_decompModel->numberoftetrahedra;
  size_t numCorners = m_decompModel->numberofcorners;
  const double* const points = m_decompModel->pointlist;
  const int* const tetra = m_decompModel->tetrahedronlist;
  const int* const neighbors = m_decompModel->neighborlist;

  //Computer center of tetrahedron as vertex of dual
  for(size_t i = 0; i < numTetras; ++i) {
    Vector3d com;
    for(size_t j = 0; j < numCorners; ++j)
      com += Vector3d(&points[3*tetra[i*numCorners + j]]);
    com /= numCorners;
    m_dualGraph.add_vertex(i, com);
  }

  //Tetrahedron adjacency implies edge in dual
  for(size_t i = 0; i < numTetras; ++i) {
    for(size_t j = 0; j < 4; ++j) {
      size_t neigh = neighbors[4*i + j];
      if(neigh != size_t(-1)) {
        Vector3d v1 = m_dualGraph.find_vertex(neigh)->property();
        Vector3d v2 = m_dualGraph.find_vertex(i)->property();
        m_dualGraph.add_edge(i, neigh, (v1-v2).norm());
      }
    }
  }
}


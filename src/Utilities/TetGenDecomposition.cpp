#include "TetGenDecomposition.h"

#include "PMPLExceptions.h"

#include "Environment/BoundingBox.h"
#include "Environment/BoundingSphere.h"
#include "Environment/Environment.h"
#include "Environment/FixedBody.h"
#include "Environment/StaticMultiBody.h"
#include "MPProblem/MPProblemBase.h"

#include <CGAL/IO/io.h>

/*------------------------------- Construction -------------------------------*/

TetGenDecomposition::
TetGenDecomposition(const string& _switches,
    bool _writeFreeModel, bool _writeDecompModel) :
    m_freeModel(new tetgenio()),
    m_decompModel(new tetgenio()),
    m_switches(_switches),
    m_writeFreeModel(_writeFreeModel),
    m_writeDecompModel(_writeDecompModel) { }


TetGenDecomposition::
TetGenDecomposition(XMLNode& _node) :
    m_freeModel(new tetgenio()),
    m_decompModel(new tetgenio()) {
  m_tetGenFilename = _node.Read("tetGenFilename", false, "",
      "Input Filename for reading TetGen models from file");
  if(m_tetGenFilename.empty()) {
    m_switches = _node.Read("switches", false, "pnqQ",
        "TetGen input parameters. See TetGen manual. Need 'pn' at a minimum");
    m_writeFreeModel = _node.Read("writeFreeModel", false, false,
        "Output TetGen model of workspace");
    m_writeDecompModel = _node.Read("writeDecompModel", false, false,
        "Output TetGen model of tetrahedralization");
    if(!m_switches.find('p'))
      m_switches += 'p';
    if(!m_switches.find('n'))
      m_switches += 'n';
  }
  m_debug = _node.Read("debug", false, false, "Show debug messages");
}

TetGenDecomposition::
~TetGenDecomposition() {
  delete m_freeModel;
  delete m_decompModel;
}

/*----------------------------- Decomposition --------------------------------*/

void
TetGenDecomposition::
Decompose(Environment* _env, const string& _baseFilename) {
  if(m_debug)
    cout << "Decomposing environment with tetgen..." << endl;

  m_env = _env;
  m_baseFilename = _baseFilename;

  if(m_tetGenFilename.empty()) {
    //make in tetgenio - this is a model of free workspace to decompose
    MakeFreeModel();
    if(m_writeFreeModel)
      SaveFreeModel();

    if(m_debug)
      cout << "\tRunning tetgen with switches '" << m_switches << "'..." << endl;

    //decompose with tetgen
    tetrahedralize(const_cast<char*>(m_switches.c_str()),
        m_freeModel, m_decompModel);
    if(m_writeDecompModel)
      SaveDecompModel();
  }
  else {
    LoadDecompModel();

    if(m_debug)
      cout << "\tRunning tetgen with switches 'rn'..." << endl;

    tetrahedralize(const_cast<char*>("rn"), m_decompModel, m_decompModel);
  }

  MakeDualGraph();

  m_env = nullptr;
  m_baseFilename = "";

  if(m_debug)
    cout << "Decomposition complete." << endl;
}

/*------------------------ Freespace Model Creation --------------------------*/

void
TetGenDecomposition::
MakeFreeModel() {
  if(m_debug)
    cout << "Creating free model..." << endl
         << "Adding boundary..." << endl;

  // Initialize the freespace model as an outward-facing boundary.
  auto cp = m_env->GetBoundary()->CGAL();
  NefPolyhedron freespace(cp);

  // Subtract each obstacle from the freespace.
  for(size_t i = 0; i < m_env->NumObstacles(); ++i) {
    if(m_debug)
      cout << "Adding obstacle " << i << "..." << endl;
    shared_ptr<StaticMultiBody> obst = m_env->GetObstacle(i);
    if(!obst->IsInternal()) {
      auto ocp = obst->GetFixedBody(0)->GetWorldPolyhedron().CGAL();
      if(m_debug)
        cout << "\tobstacle is " << (ocp.is_closed() ? "" : "not ") << "closed"
             << endl;
      freespace -= NefPolyhedron(ocp);
    }
  }

  // Add free model to tetgen structure.
  AddVertices(freespace);
  AddFacets(freespace);
  AddHoles(freespace);
}


void
TetGenDecomposition::
AddVertices(const NefPolyhedron& _freespace) {
  if(m_debug)
    cout << "Adding vertices..." << endl;

  m_freeModel->numberofpoints = _freespace.number_of_vertices();
  m_freeModel->pointlist = new double[m_freeModel->numberofpoints * 3];

  size_t n = 0;
  for(auto v = _freespace.vertices_begin(); v != _freespace.vertices_end(); ++v) {
    using CGAL::to_double;
    m_freeModel->pointlist[3 * n + 0] = to_double(v->point()[0]);
    m_freeModel->pointlist[3 * n + 1] = to_double(v->point()[1]);
    m_freeModel->pointlist[3 * n + 2] = to_double(v->point()[2]);
    ++n;
  }
}


void
TetGenDecomposition::
AddFacets(const NefPolyhedron& _freespace) {
  if(m_debug)
    cout << "Adding facets..." << endl;

  auto facets = ExtractFacets(_freespace);

  m_freeModel->numberoffacets = facets.size();
  m_freeModel->facetlist = new tetgenio::facet[m_freeModel->numberoffacets];
  m_freeModel->facetmarkerlist = new int[m_freeModel->numberoffacets];

  for(size_t fn = 0; fn < facets.size(); ++fn) {
    const auto& facetData = facets[fn];
    tetgenio::facet* f = &m_freeModel->facetlist[fn];

    // Add this facet to the free model.
    f->numberofpolygons = facetData.size();
    f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
    f->numberofholes = 0;
    f->holelist = nullptr;

    m_freeModel->facetmarkerlist[fn] = 1;

    for(size_t pn = 0; pn < facetData.size(); ++pn) {
      const auto& polygonData = facetData[pn];

      // Add this polygon to the facet
      tetgenio::polygon* p = &f->polygonlist[pn];
      p->numberofvertices = polygonData.size();
      p->vertexlist = new int[p->numberofvertices];

      for(size_t vn = 0; vn < polygonData.size(); ++vn) {
        const auto& vertex = polygonData[vn];

        // Add this vertex to the polygon.
        p->vertexlist[vn] = vertex;
      }
    }
  }
}


void
TetGenDecomposition::
AddHoles(const NefPolyhedron& _freespace) {
  if(m_debug)
    cout << "Adding holes..." << endl;

  // Initialize the boundary as inward-facing.
  auto cp = m_env->GetBoundary()->CGAL();
  NefPolyhedron boundary(cp);
  boundary = boundary.complement();

  vector<shared_ptr<StaticMultiBody>> holes;
  for(size_t i = 0; i < m_env->NumObstacles(); ++i) {
    shared_ptr<StaticMultiBody> obst = m_env->GetObstacle(i);
    if(!obst->IsInternal())
      holes.push_back(obst);
  }

  // Add a hole to the tetgen structure for each obstacle. Use the furthest
  // convex hull point perturbed slightly towards the center of mass.
  m_freeModel->numberofholes = holes.size();
  m_freeModel->holelist = new double[m_freeModel->numberofholes * 3];

  size_t num = 0;
  for(const auto obst : holes) {
    const auto& body = obst->GetFixedBody(0);
    Vector3d com = body->GetCenterOfMass();
    Vector3d hole = com;
    GMSPolyhedron& poly = body->GetPolyhedron();
    for(auto& v : poly.m_vertexList)
      if(body->IsConvexHullVertex(v) && (v - com).norm() > (hole - com).norm())
        hole = v;
    hole = hole + (com - hole).normalize() * 0.0001;
    hole = body->GetWorldTransformation() * hole;
    m_freeModel->holelist[3 * num + 0] = hole[0];
    m_freeModel->holelist[3 * num + 1] = hole[1];
    m_freeModel->holelist[3 * num + 2] = hole[2];
    ++num;
  }
}


size_t
TetGenDecomposition::
PointIndex(const NefPolyhedron& _freespace, const CGALPoint& _p) const {
  size_t index = 0;
  auto it = _freespace.vertices_begin();
  for(; it != _freespace.vertices_end(); ++it, ++index)
    if(_p == it->point())
      break;
  if(it == _freespace.vertices_end())
    throw RunTimeException(WHERE, "Point not found.");
  return index;
}


vector<vector<vector<size_t>>>
TetGenDecomposition::
ExtractFacets(const NefPolyhedron& _freespace) {
  using SHalfloop_const_handle = NefPolyhedron::SHalfloop_const_handle;
  using SHalfedge_const_handle = NefPolyhedron::SHalfedge_const_handle;
  using SHalfedge_around_facet_const_circulator =
      NefPolyhedron::SHalfedge_around_facet_const_circulator;

  typedef vector<size_t> Polygon;
  typedef vector<Polygon> Facet;

  vector<Facet> facets;

  // NefPolyhedron represents each facet as two half-facets. Each has one
  // outward-facing half-facet and one inward-facing twin. Start by iterating
  // over all half-facets.
  for(auto hf = _freespace.halffacets_begin(); hf != _freespace.halffacets_end();
      ++hf) {

    // Assume that our objects are well-formed and skip inward-facing twins.
    if(hf->is_twin())
      continue;

    Facet f;

    // Each facet is composed of one or more polygons. Each polygon is described
    // by a 'half-facet cycle'. Extract the points from these cycles to describe
    // the facet's polygons.
    for(auto it = hf->facet_cycles_begin(); it != hf->facet_cycles_end(); ++it) {
      Polygon poly;

      // Each cycle represents either a single, isolated vertex or a set of
      // vertices connected by edges.
      if(it.is_shalfloop()) {
        // This is an isolated vertex.
        SHalfloop_const_handle sl(it);
        if(sl == 0)
          throw RunTimeException(WHERE, "TetGenDecomposition error: could not "
              "get shalfloop handle while adding facets to freespace model.");

        const auto& point = sl->incident_sface()->center_vertex()->point();
        poly.push_back(PointIndex(_freespace, point));
      }
      else {
        // This is a set of vertices joined by edges.
        SHalfedge_const_handle se(it);
        if(se == 0)
          throw RunTimeException(WHERE, "TetGenDecomposition error: could not "
              "get shalfedge handle while adding facets to freespace model.");

        SHalfedge_around_facet_const_circulator hc_start(se);
        SHalfedge_around_facet_const_circulator hc_end(hc_start);
        CGAL_For_all(hc_start, hc_end) {
          const auto& point = hc_start->source()->center_vertex()->point();
          poly.push_back(PointIndex(_freespace, point));
        }
      }

      // Add the polygon to the current facet.
      f.push_back(poly);
    }

    // Add the facet to the facet list.
    facets.push_back(f);
  }

  return facets;
}

/*------------------------------- IO Helpers ---------------------------------*/

void
TetGenDecomposition::
SaveFreeModel() {
  string basename = m_baseFilename + ".freespace";
  char* b = const_cast<char*>(basename.c_str());
  m_freeModel->save_nodes(b);
  m_freeModel->save_poly(b);
}


void
TetGenDecomposition::
SaveDecompModel() {
  string basename = m_baseFilename + ".decomposed";
  char* b = const_cast<char*>(basename.c_str());
  m_decompModel->save_nodes(b);
  m_decompModel->save_elements(b);
  //m_decompModel->save_neighbors(b);
}


void
TetGenDecomposition::
LoadDecompModel() {
  string basename = MPProblemBase::GetPath(m_tetGenFilename);
  char* b = const_cast<char*>(basename.c_str());
  m_decompModel->load_node(b);
  m_decompModel->load_tet(b);
  //m_decompModel->load_neighbors(b);
}

/*------------------------- Dual Graph Helpers -------------------------------*/

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

/*----------------------------------------------------------------------------*/

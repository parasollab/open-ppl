#include "TetGenDecomposition.h"

#include "Geometry/Bodies/FixedBody.h"
#include "Geometry/Bodies/StaticMultiBody.h"
#include "MPProblem/Environment/Environment.h"
#include "MPProblem/MPProblem.h"
#include "Workspace/WorkspaceDecomposition.h"

#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"

#define TETLIBRARY
#undef PI
#include "tetgen.h"

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/IO/io.h>
#include <CGAL/Nef_polyhedron_3.h>

using CGALKernel    = CGAL::Exact_predicates_exact_constructions_kernel;
using NefPolyhedron = CGAL::Nef_polyhedron_3<CGALKernel>;


/*---------------------------- Local Functions -------------------------------*/
// These are implemented here to avoid including CGAL nef poly stuff in the
// header file. This is needed because the nef poly stuff requires us to
// undefine PI, and there is no way to sand-box that inside of a header.
// Instead, every function that has a CGAL nef poly as part of its object is
// implemented as a 'manual external member' in this section.


/// Get the index of a point in the freespace nef poly.
/// @param _polyhedron The nef polyhedron of interest.
/// @param _p The point to locate.
/// @return The index of _p in _polyhedron's vertex list. If it is not found,
///         an exception will be thrown.
size_t
PointIndex(const NefPolyhedron& _polyhedron,
    const NefPolyhedron::Point_3& _p) {
  size_t index = 0;
  auto it = _polyhedron.vertices_begin();
  for(; it != _polyhedron.vertices_end(); ++it, ++index)
    if(_p == it->point())
      break;
  if(it == _polyhedron.vertices_end())
    throw RunTimeException(WHERE, "Point not found.");
  return index;
}


/// Add the vertices to the free model.
/// @param _freeModel The tetgen model under construction.
/// @param _freespace The free space polyhedra.
/// @param _debug Show debug messages?
void
AddVertices(tetgenio* const _freeModel, const NefPolyhedron& _freespace,
    const bool _debug) {
  if(_debug)
    cout << "Adding " << _freespace.number_of_vertices() << " vertices..."
         << endl;

  _freeModel->numberofpoints = _freespace.number_of_vertices();
  _freeModel->pointlist = new double[_freeModel->numberofpoints * 3];

  size_t n = 0;
  for(auto v = _freespace.vertices_begin(); v != _freespace.vertices_end(); ++v) {
    using CGAL::to_double;
    _freeModel->pointlist[3 * n + 0] = to_double(v->point()[0]);
    _freeModel->pointlist[3 * n + 1] = to_double(v->point()[1]);
    _freeModel->pointlist[3 * n + 2] = to_double(v->point()[2]);
    ++n;
  }
}


/// Extract facet info from the freespace nef poly.
/// @param _freespace The freespace nef poly.
/// @return A set of facet descriptors.
vector<vector<vector<size_t>>>
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


/// Add the facets to the free model.
/// @param _freeModel The tetgen model under construction.
/// @param _freespace The free space polyhedra.
/// @param _debug Show debug messages?
void
AddFacets(tetgenio* const _freeModel, const NefPolyhedron& _freespace,
    const bool _debug) {
  auto facets = ExtractFacets(_freespace);

  if(_debug)
    cout << "Adding " << facets.size() << " facets..." << endl;

  _freeModel->numberoffacets = facets.size();
  _freeModel->facetlist = new tetgenio::facet[_freeModel->numberoffacets];
  _freeModel->facetmarkerlist = new int[_freeModel->numberoffacets];

  for(size_t fn = 0; fn < facets.size(); ++fn) {
    const auto& facetData = facets[fn];
    tetgenio::facet* f = &_freeModel->facetlist[fn];

    // Add this facet to the free model.
    f->numberofpolygons = facetData.size();
    f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
    f->numberofholes = 0;
    f->holelist = nullptr;

    _freeModel->facetmarkerlist[fn] = 1;

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


/// Add holes to the free model for each obstacle touching the final'
///        boundary.
/// @param _freeModel The tetgen model under construction.
/// @param _freespace The free space polyhedra.
/// @param _env The environment object.
/// @param _debug Show debug messages?
void
AddHoles(tetgenio* const _freeModel, const NefPolyhedron& _freespace,
    const Environment* const _env, const bool _debug) {
  if(_debug)
    cout << "Adding holes..." << endl;

  // Initialize the boundary as inward-facing.
  auto cp = _env->GetBoundary()->CGAL();
  NefPolyhedron boundary(cp);
  boundary = boundary.complement();

  vector<StaticMultiBody*> holes;
  for(size_t i = 0; i < _env->NumObstacles(); ++i) {
    StaticMultiBody* obst = _env->GetObstacle(i);
    if(!obst->IsInternal())
      holes.push_back(obst);
  }

  // Add a hole to the tetgen structure for each obstacle. Use the furthest
  // convex hull point perturbed slightly towards the center of mass.
  _freeModel->numberofholes = holes.size();
  _freeModel->holelist = new double[_freeModel->numberofholes * 3];

  size_t num = 0;
  for(const auto obst : holes) {
    const auto& body = obst->GetFixedBody(0);
    const auto& com = body->GetWorldPolyhedron().GetCentroid();
    Vector3d hole = com;
    const GMSPolyhedron& poly = body->GetPolyhedron();
    for(auto& v : poly.m_vertexList)
      if(body->IsConvexHullVertex(v) && (v - com).norm() > (hole - com).norm())
        hole = v;
    hole = hole + (com - hole).normalize() * 0.0001;
    hole = body->GetWorldTransformation() * hole;
    _freeModel->holelist[3 * num + 0] = hole[0];
    _freeModel->holelist[3 * num + 1] = hole[1];
    _freeModel->holelist[3 * num + 2] = hole[2];
    ++num;
  }
}

/*-------------------------- Static Initializers -----------------------------*/

TetGenDecomposition::Parameters TetGenDecomposition::m_defaultParams;

/*------------------------------- Construction -------------------------------*/

TetGenDecomposition::
TetGenDecomposition(const string& _baseFilename)
    : m_params(m_defaultParams), m_baseFilename(_baseFilename) {}


TetGenDecomposition::
TetGenDecomposition(const string& _baseFilename, const string& _switches,
    const bool _writeFreeModel, const bool _writeDecompModel,
    const string& _inputFilename)
    : TetGenDecomposition(_baseFilename) {

  m_params.inputFilename    = _inputFilename;
  m_params.switches         = _switches;
  m_params.writeFreeModel   = _writeFreeModel;
  m_params.writeDecompModel = _writeDecompModel;

  ValidateSwitches(m_params.switches);
}


void
TetGenDecomposition::
SetDefaultParameters(XMLNode& _node) {
  m_defaultParams.inputFilename = _node.Read("decompModelFilename", false,
      m_defaultParams.inputFilename,
      "Input filename for reading TetGen models from file");

  m_defaultParams.switches = _node.Read("switches", false,
      m_defaultParams.switches,
      "TetGen input parameters. See TetGen manual. Need 'pn' at a minimum");

  m_defaultParams.writeFreeModel = _node.Read("writeFreeModel", false,
      m_defaultParams.writeFreeModel,
      "Output TetGen model of workspace");

  m_defaultParams.writeDecompModel = _node.Read("writeDecompModel", false,
      m_defaultParams.writeDecompModel,
      "Output TetGen model of tetrahedralization");

  m_defaultParams.debug = _node.Read("debug", false,
      m_defaultParams.debug,
      "Show debugging messages");

  ValidateSwitches(m_defaultParams.switches);
}


TetGenDecomposition::
~TetGenDecomposition() {
  delete m_freeModel;
  delete m_decompModel;
}

/*----------------------------- Decomposition --------------------------------*/

shared_ptr<WorkspaceDecomposition>
TetGenDecomposition::
operator()(const Environment* _env) {
  Initialize();
  m_env = _env;

  if(!m_params.inputFilename.empty()) {
    LoadDecompModel();

    if(m_params.debug)
      cout << "\tRunning tetgen with switches 'rnQ'..." << endl;

    tetrahedralize(const_cast<char*>("rnQ"), m_decompModel, m_decompModel);
  }
  else {
    if(m_params.debug)
      cout << "Decomposing environment with tetgen..."
           << endl;

    MakeFreeModel();
    if(m_params.writeFreeModel)
      SaveFreeModel();

    if(m_params.debug)
      std::cout << "\tRunning tetgen with switches '" << m_params.switches
                << "'..." << std::endl;

    tetrahedralize(const_cast<char*>(m_params.switches.c_str()),
        m_freeModel, m_decompModel);
    if(m_params.writeDecompModel)
      SaveDecompModel();
  }

  m_env = nullptr;

  if(m_params.debug)
    cout << "Decomposition complete." << endl;

  return MakeDecomposition();
}


shared_ptr<WorkspaceDecomposition>
TetGenDecomposition::
MakeDecomposition() {
  // Assert that tetgen actually produced a tetrahedralization and not something
  // else.
  const size_t numCorners = m_decompModel->numberofcorners;
  if(numCorners != 4)
    throw PMPLException("TetGen error", WHERE, "The decomposition is not "
        "tetrahedral. Expected 4 points per tetraherdon, but got " +
        to_string(numCorners) + ".");

  // Make decomposition object.
  shared_ptr<WorkspaceDecomposition> decomposition(new WorkspaceDecomposition());

  // Add points.
  const size_t numPoints = m_decompModel->numberofpoints;
  const double* const points = m_decompModel->pointlist;
  for(size_t i = 0; i < numPoints; ++i)
    decomposition->AddPoint(Point3d(&points[3 * i]));

  // Make region for each tetrahedron.
  const size_t numTetras = m_decompModel->numberoftetrahedra;
  const int* const tetra = m_decompModel->tetrahedronlist;
  for(size_t i = 0; i < numTetras; ++i)
    decomposition->AddTetrahedralRegion(&tetra[i * numCorners]);

  // Make edges between adjacent tetrahedra.
  const int* const neighbors = m_decompModel->neighborlist;
  for(size_t i = 0; i < numTetras; ++i) {
    // Each tetrahedron has up to 4 neighbors. Empty slots are marked with an
    // index of -1.
    for(size_t j = 0; j < 4; ++j) {
      size_t neighborIndex = neighbors[4 * i + j];
      if(neighborIndex != size_t(-1))
        decomposition->AddPortal(i, neighborIndex);
    }
  }

  if(m_params.debug)
    std::cout << "\tNumber of points: " << numPoints << std::endl
              << "\tNumber of tetras: " << numTetras << std::endl;

  decomposition->Finalize();
  return decomposition;
}

/*------------------------ Freespace Model Creation --------------------------*/

void
TetGenDecomposition::
Initialize() {
  delete m_freeModel;
  delete m_decompModel;
  m_freeModel = new tetgenio();
  m_decompModel = new tetgenio();
}


void
TetGenDecomposition::
MakeFreeModel() {
  if(m_params.debug)
    cout << "Creating free model..." << endl
         << "\tAdding boundary..." << endl;

  // Initialize the freespace model as an outward-facing boundary.
  auto cp = m_env->GetBoundary()->CGAL();
  NefPolyhedron freespace(cp);

  // Subtract each obstacle from the freespace.
  for(size_t i = 0; i < m_env->NumObstacles(); ++i) {
    if(m_params.debug)
      cout << "\tAdding obstacle " << i << "..." << endl;

    StaticMultiBody* obst = m_env->GetObstacle(i);
    if(!obst->IsInternal()) {
      // Make CGAL representation of this obstacle.
      auto ocp = obst->GetFixedBody(0)->GetWorldPolyhedron().CGAL();

      if(m_params.debug)
        cout << "\t\tobstacle is " << (ocp.is_closed() ? "" : "not ")
             << "closed" << endl;

      // Subtract it from the freespace.
      freespace -= NefPolyhedron(ocp);
    }
  }

  // Add free model to tetgen structure.
  AddVertices(m_freeModel, freespace, m_params.debug);
  AddFacets(m_freeModel, freespace, m_params.debug);
  AddHoles(m_freeModel, freespace, m_env, m_params.debug);
}


/*------------------------------- IO Helpers ---------------------------------*/

void
TetGenDecomposition::
ValidateSwitches(std::string& _switches) {
  if(_switches.find('p') == string::npos)
    _switches += 'p';
  if(_switches.find('n') == string::npos)
    _switches += 'n';
}


void
TetGenDecomposition::
SaveFreeModel() {
  string basename = m_baseFilename + ".freespace";

  if(m_params.debug)
    cout << "Saving tetgen free space model with base name '" << basename << "'"
         << endl;

  char* b = const_cast<char*>(basename.c_str());
  m_freeModel->save_nodes(b);
  m_freeModel->save_poly(b);
}


void
TetGenDecomposition::
SaveDecompModel() {
  string basename = m_baseFilename + ".decomposed";

  if(m_params.debug)
    cout << "Saving tetgen decomposition model with base name '" << basename
         << "'" << endl;

  char* b = const_cast<char*>(basename.c_str());
  m_decompModel->save_nodes(b);
  m_decompModel->save_elements(b);
  //m_decompModel->save_neighbors(b);
}


void
TetGenDecomposition::
LoadDecompModel() {
  string basename = MPProblem::GetPath(m_params.inputFilename);

  if(m_params.debug)
    cout << "Loading tetgen files with base name '" << basename << "'"
         << endl;

  char* b = const_cast<char*>(basename.c_str());
  m_decompModel->load_node(b);
  m_decompModel->load_tet(b);
  //m_decompModel->load_neighbors(b);
}

/*----------------------------------------------------------------------------*/

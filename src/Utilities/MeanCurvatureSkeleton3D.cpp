#include "MeanCurvatureSkeleton3D.h"
#include "MPProblem/Environment/Environment.h"
#include "Geometry/GMSPolyhedron.h"
#include "Geometry/Bodies/Body.h"
#include "Geometry/Bodies/MultiBody.h"
#include "Geometry/Boundaries/WorkspaceBoundingBox.h"
#include "Workspace/WorkspaceSkeleton.h"
#include <cmath>
#include <map>
#include <set>
#include <algorithm>
#include <sstream>

#undef PI

#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/boost/graph/split_graph_into_polylines.h>
#include <CGAL/boost/graph/convert_nef_polyhedron_to_polygon_mesh.h>
#include <CGAL/subdivision_method_3.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/refine.h>
#include <CGAL/Polygon_mesh_processing/fair.h>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/squared_distance_3.h>
#include <CGAL/Bbox_3.h>
#include <boost/foreach.hpp>
#include <boost/function_output_iterator.hpp>

/*--------------------------- I/O ------------------------------------*/

void
MeanCurvature3D::
Read(string _filename) {
  m_skeleton.Read("skeleton_" + _filename);
  m_skeleton.Write("verify-skeleton_" + _filename);

  m_annotation = AnnotationType(&m_skeleton);
  m_annotation.Read("annotation_" + _filename);
  m_annotation.Write("verify-annotation_" + _filename);
}

void
MeanCurvature3D::
Write(string _filename) {
  m_skeleton.Write("skeleton_" + _filename);
  m_annotation.Write("annotation_" + _filename);
}

// Helper function to change CGAL Point3d to PMPL Point3d
Point3d ToPMPLPoint3d(const CGAL::Simple_cartesian<double>::Point_3& _p) {
	return Point3d(_p[0], _p[1], _p[2]);
}

/*------------------------- Constructors ----------------------------------*/
MeanCurvature3D::
MeanCurvature3D(const Environment* _env, size_t _space, bool _d) : m_debug(_d) {
  if(_space == 1)
    InitializeFreeSpace(_env);
  else
    InitializeObstacleSpace(_env);
}

MeanCurvature3D::
~MeanCurvature3D()	{
  Reset();
}

/*--------------------------- Modifiers ------------------------------*/

void
MeanCurvature3D::
BuildSkeleton(bool _curve) {
  if(m_params.ioType == "read") {
    Read(m_params.filename);
    return;
  }

  // validate whether the input mesh is triangulated
  if(!is_triangle_mesh(m_input)) {
    if(m_debug)
      cerr <<"Input mesh needs to be triangulated" << endl;
    // Triangulate mesh if not
    if(!CGAL::Polygon_mesh_processing::triangulate_faces(m_input)){
      if(m_debug)
        cerr << "Mesh cannot be fully triangulated "<<endl;
      return;
    }
  }

  if(m_debug)
    cout<<"Input mesh has " << m_input.number_of_vertices() <<" vertices "
      << m_input.number_of_edges() <<" edges "
      << m_input.number_of_faces() <<" faces "<<endl;

  // build the skeleton only if the mesh is triangulated

  Skeletonization mcs(m_input);
  mcs.set_quality_speed_tradeoff(m_params.m_wH);//10
  //mcs.set_min_edge_length(1);
  mcs.set_medially_centered_speed_tradeoff(m_params.m_wM);//100
  mcs.set_max_iterations(m_params.m_iterations);

  //if(m_debug)
  cout<<"Parameters: WH "<< mcs.quality_speed_tradeoff() << "\n"
    << " WM "<< mcs.medially_centered_speed_tradeoff() << "\n"
    << " max triangle angle "<< mcs.max_triangle_angle() << "\n"
    << " min edge length " << mcs.min_edge_length() << "\n"
    << " iterations "<< mcs.max_iterations() << "\n"
    << " area variation " << mcs.area_variation_factor() << "\n"
    << "is medially centered: " << mcs.is_medially_centered() << endl;

  // Contract the mesh by mean curvature flow.
  mcs.contract_until_convergence();

  // Get the output as curve skeleton / meso-skeleton surface mesh
  if(_curve) {
    m_meso = mcs.meso_skeleton();
    cout<<"Number of faces in meso skeleton: "<<num_faces(m_meso)<<endl;
    cout<<"Number of vertices in meso skeleton: "<<num_vertices(m_meso)<<endl;
    cout<<"Number of edges in meso skeleton: "<<num_edges(m_meso)<<endl;
    // Convert the contracted mesh into a curve skeleton and
    // get the correspondent surface points
    mcs.convert_to_skeleton(m_mcs);
    // Validation
    if(m_debug) {
      cout << "Number of vertices of the skeleton: " << boost::num_vertices(m_mcs) << "\n";
      cout << "Number of edges of the skeleton: " << boost::num_edges(m_mcs) << "\n";
    }
  }
  else
    m_meso = mcs.meso_skeleton();
}

/*--------------------------- Accessors ------------------------------*/

pair<WorkspaceSkeleton, MeanCurvature3D::AnnotationType>
MeanCurvature3D::
GetSkeleton() {
  if(m_params.ioType != "read") {
    typedef WorkspaceSkeleton::GraphType Graph;
    Graph g;
    // Map stores already inserted vertices in the graph
    map<MCVD, typename Graph::vertex_descriptor> vertexMap;
    // Segment the graph as set of polylines
    vector<vector<MCVD>> plines;
    ToPolyLines(plines);
    vector<pair<typename Graph::vertex_descriptor, WitnessType>> vertexWitnesses;
    vector<pair<typename Graph::edge_descriptor, vector<WitnessType>>> edgeWitnesses;
    // For each polyline - create an edge
    for(auto pline : plines){
      // Get the two end points
      MCVD epts[2];
      epts[0] = pline.front();
      epts[1] = pline.back();
      // vertex descriptors of end points
      Graph::vertex_descriptor vd[2];
      // For each end point create a vertex if not already in the graph
      for(size_t i = 0; i < 2; ++i) {
        auto findPt = vertexMap.find(epts[i]);
        // If end point not already in the graph - add vertex
        if(findPt == vertexMap.end()) {
          size_t deg = degree(epts[i], m_mcs); if(deg > 6 ) cout<<"degree"<<deg<<endl;
          vd[i] = g.add_vertex(ToPMPLPoint3d(Point(epts[i])));
          vertexMap.insert(make_pair(epts[i],vd[i]));
          // Store the witness points for the vertex
          vertexWitnesses.push_back(make_pair(vd[i], WitnessType()));
          GetWitnesses(epts[i], vertexWitnesses.back().second);
        }
        else
          vd[i] = findPt->second;
      }
      //Add edge
      vector<Point3d> intermediates;
      vector<WitnessType> intermediateWitnesses(pline.size());
      for(size_t i = 0; i < pline.size(); i++) {
        intermediates.push_back(ToPMPLPoint3d(Point(pline[i])));
        // Get the witnesses for the intermediate points in the edge
        GetWitnesses(pline[i], intermediateWitnesses[i]);
      }
      auto ed = g.add_edge(vd[0], vd[1], intermediates);
      // Store the witnesses of the edge points
      edgeWitnesses.push_back(make_pair(ed, intermediateWitnesses));
    }
    m_skeleton.SetGraph(g);
    // Set witnesses as annotation to the skeleton
    m_annotation = AnnotationType(&m_skeleton);
    for(auto vwitness : vertexWitnesses)
      m_annotation.SetVertexProperty(vwitness.first, vwitness.second);
    for(auto ewitness : edgeWitnesses)
      m_annotation.SetEdgeProperty(ewitness.first, ewitness.second);
  }

  if(m_params.ioType == "write")
    Write(m_params.filename);

  return make_pair(m_skeleton, m_annotation);
}

pair<WorkspaceSkeleton, MeanCurvature3D::AnnotationType>
MeanCurvature3D::
GetMesoSkeleton() {
  typedef WorkspaceSkeleton::GraphType Graph;
  Graph g;
  vector<pair<typename Graph::vertex_descriptor, WitnessType>> vertexWitnesses;
  vector<pair<typename Graph::edge_descriptor, vector<WitnessType>>> edgeWitnesses;
  // Map stores already inserted vertices in the graph
  typedef Skeletonization::vertex_descriptor VD;
  map<VD, typename Graph::vertex_descriptor> vertexMap;

  // Get all vertices
  BOOST_FOREACH(VD vd, vertices(m_meso)){
    auto svd = g.add_vertex(ToPMPLPoint3d(vd->point()));
    vertexMap.insert(make_pair(vd,svd));
    // Store the witness points for the vertex
    vertexWitnesses.push_back(make_pair(svd, WitnessType()));
    GetWitnesses(vd, vertexWitnesses.back().second);
  }

  // Get all edges
  typedef Skeletonization::edge_descriptor ED;
  typedef Skeletonization::face_descriptor FD;
  typedef Skeletonization::halfedge_descriptor HD;

  BOOST_FOREACH(ED ed, edges(m_meso)) {
    Graph::vertex_descriptor vd[2];
    VD src = source(ed, m_meso);
    auto findvd = vertexMap.find(src);
    if(findvd == vertexMap.end())
      continue;
    else
      vd[0] = findvd->second;
    VD tgt = target(ed, m_meso);
    findvd = vertexMap.find(tgt);
    if(findvd == vertexMap.end())
      continue;
    else
      vd[1] = findvd->second;
    //Add edge
    vector<Point3d> intermediates;
    vector<WitnessType> intermediateWitnesses(2);
    intermediates.push_back(ToPMPLPoint3d(src->point()));
    // Get the witnesses for the intermediate points in the edge
    GetWitnesses(src, intermediateWitnesses[0]);
    intermediates.push_back(ToPMPLPoint3d(tgt->point()));
    // Get the witnesses for the intermediate points in the edge
    GetWitnesses(tgt, intermediateWitnesses[1]);

    auto sed = g.add_edge(vd[0], vd[1], intermediates);
    // Store the witnesses of the edge points
    edgeWitnesses.push_back(make_pair(sed, intermediateWitnesses));
  }

  // Return the annotated skeleton
  WorkspaceSkeleton skeleton;
  skeleton.SetGraph(g);
  // Set witnesses as annotation to the skeleton
  AnnotationType annotation(&skeleton);
  for(auto vwitness : vertexWitnesses)
    annotation.SetVertexProperty(vwitness.first, vwitness.second);
  for(auto ewitness : edgeWitnesses)
    annotation.SetEdgeProperty(ewitness.first, ewitness.second);
  return make_pair(skeleton, annotation);
}

/*--------------------------- Helpers ------------------------------*/

void
MeanCurvature3D::
InitializeFreeSpace(const Environment* _env){
  // Initialize the freespace model as an outward-facing boundary.
  auto cp = _env->GetBoundary()->CGAL();
  double len = 0, minr = numeric_limits<double>::max();
  for(size_t i = 0; i < 3; ++i) {
    len += pow( _env->GetBoundary()->GetRange(i).Length(), 2);
    minr = min(minr , _env->GetBoundary()->GetRange(i).Length());
  }
  len = sqrt(len);
  typedef CGAL::Nef_polyhedron_3<typename GMSPolyhedron::CGALKernel> NefPolyhedron;
  NefPolyhedron freespace(cp);

  // Subtract each obstacle from the freespace.
  for(size_t i = 0; i < _env->NumObstacles(); ++i) {
    if(m_debug)
      cout << "\tAdding obstacle " << i << "..." << endl;

    auto obst = _env->GetObstacle(i);
    if(!obst->IsInternal()) {
      // Make CGAL representation of this obstacle.
      auto ocp = obst->GetBody(0)->GetWorldPolyhedron().CGAL();

      if(m_debug)
        cout << "\t\tobstacle is " << (ocp.is_closed() ? "" : "not ")
             << "closed" << endl;

      // Subtract it from the freespace.
      freespace -= NefPolyhedron(ocp);
    }
  }

  // Check whether the input is 2-manifold. Mean Curvature Skeleton cannot handle
  // degenaries
  if(!freespace.is_simple())
    cerr<<"Mesh is not 2-manifold. Cannot construct Mean Curvature Skeleton for it."<<endl;

  // Convert the nef polyhedron to polyhedron with same point kernel as nef polyhedron
  if(m_debug)
    cout <<" NefPolhedron: Vertices "<<freespace.number_of_vertices()
      <<" Edges "<<freespace.number_of_edges()
      <<" Faces "<<freespace.number_of_facets()<< endl;
  GMSPolyhedron::CGALPolyhedron smesh;
  freespace.convert_to_polyhedron(smesh);

  // Change the surface mesh to the surface with simple cartesian point kernel
  // otherwise skeletonization does not work with exact kernel
  AddPolyhedron(smesh,0.01*len);
}

void
MeanCurvature3D::
InitializeObstacleSpace(const Environment* _env){
  // Initialize the obstaclespace model as empty.
  auto cp = _env->GetBoundary()->CGAL();
  double len = 0, minr = numeric_limits<double>::max();
  for(size_t i = 0; i < 3; ++i) {
    len += pow( _env->GetBoundary()->GetRange(i).Length(), 2);
    minr = min(minr , _env->GetBoundary()->GetRange(i).Length());
  }
  len = sqrt(len);
  cout<<"Diagonal length"<<len<<" "<<minr<<endl;
  typedef CGAL::Nef_polyhedron_3<typename GMSPolyhedron::CGALKernel> NefPolyhedron;
  NefPolyhedron obstspace(NefPolyhedron::EMPTY);

  // Add all obstacle
  for(size_t i = 0; i < _env->NumObstacles(); ++i) {
    if(m_debug)
      cout << "\tAdding obstacle " << i << "..." << endl;

    auto obst = _env->GetObstacle(i);
    if(!obst->IsInternal()) {
      // Make CGAL representation of this obstacle.
      auto ocp = obst->GetBody(0)->GetWorldPolyhedron().CGAL();
      if(m_debug)
        cout << "\t\tobstacle is " << (ocp.is_closed() ? "" : "not ")
             << "closed" << endl;

      // Find the intersection of the obstacle with the boundary
      NefPolyhedron obst(cp);
      obst *= NefPolyhedron(ocp);
      // Add it to the obstaclespace.
      obstspace += obst;//NefPolyhedron(ocp);
    }
  }

  // Check whether the input is 2-manifold. Mean Curvature Skeleton cannot handle
  // degenaries
  if(!obstspace.is_simple())
    cerr<<"Mesh is not 2-manifold. Cannot construct Mean Curvature Skeleton for it."<<endl;

  // Convert the nef polyhedron to polyhedron with same point kernel as nef polyhedron
  if(m_debug)
    cout <<" NefPolhedron: Vertices "<<obstspace.number_of_vertices()
      <<" Edges "<<obstspace.number_of_edges()
      <<" Faces "<<obstspace.number_of_facets()<< endl;
  GMSPolyhedron::CGALPolyhedron smesh;
  obstspace.convert_to_polyhedron(smesh);

  // Change the surface mesh to the surface with simple cartesian point kernel
  // otherwise skeletonization does not work with exact kernel
  AddPolyhedron(smesh,0.01*len);
}


void
MeanCurvature3D::
GetWitnesses(MCVD& _v, vector<Point3d>& _witness) {
  typedef boost::graph_traits<SurfaceMesh>::vertex_descriptor SMVD;
  BOOST_FOREACH(SMVD vd, m_mcs[_v].vertices){
    _witness.push_back(ToPMPLPoint3d(get(CGAL::vertex_point, m_input, vd)));
  }
}

void
MeanCurvature3D::
GetWitnesses(MESO::Vertex& _v, vector<Point3d>& _witness) {
 typedef boost::graph_traits<SurfaceMesh>::vertex_descriptor SMVD;
 BOOST_FOREACH(SMVD vd, _v.vertices)
    _witness.push_back(ToPMPLPoint3d(get(CGAL::vertex_point, m_input, vd)));
}

void
MeanCurvature3D::
GetWitnesses(Skeletonization::vertex_descriptor& _v, vector<Point3d>& _witness){
  typedef boost::graph_traits<SurfaceMesh>::vertex_descriptor SMVD;
  BOOST_FOREACH(SMVD vd, _v->vertices)
    _witness.push_back(ToPMPLPoint3d(get(CGAL::vertex_point, m_input, vd)));
}

void MeanCurvature3D::Refine(SurfaceMesh& _m, double _f)  {
  CGAL::Polygon_mesh_processing::isotropic_remeshing(_m.faces(), _f,
    _m,
    CGAL::Polygon_mesh_processing::parameters::number_of_iterations(1));
}

void MeanCurvature3D::Reset() {
  m_input.clear();
  m_mcs.clear();
}

void
MeanCurvature3D::
ToPolyLines(vector<vector<MCVD>>& _plines, bool _isSimplify)	{
  if(!_isSimplify) {
    // Putting all the edges
    BOOST_FOREACH(MCED e, edges(m_mcs)) {
      _plines.push_back(vector<MCVD>());
      _plines.back().push_back(source(e, m_mcs));
		  _plines.back().push_back(target(e, m_mcs));
    }
		return;
  }
	// Helper structure that breaks any boost graph edges into polylines
  // It works like a visitor structure in graph algos
  // The function signatures are provided by CGAL
  struct SplitPolyline{
    const MCSkeleton& skeleton;
    vector<vector<MCVD>> lines;

    // Functions that needs to be defined that are used in CGAL function
    // Signature provided by CGAL (Do not change the signature)
    void start_new_polyline(){
      lines.push_back(vector<MCVD>());
    }
    void add_node(MCVD v){
      lines.back().push_back(v);
    }
    void end_polyline(){ }
    // User defined functions
    // This functions including the signature can be changed by the user
    SplitPolyline(const MCSkeleton& _sk): skeleton(_sk) {
      lines.clear();
    }
  };
  SplitPolyline sp(m_mcs);
  // Split the graph into polylines
  CGAL::split_graph_into_polylines(m_mcs, sp);
  _plines.insert(_plines.end(), sp.lines.begin(), sp.lines.end());
}




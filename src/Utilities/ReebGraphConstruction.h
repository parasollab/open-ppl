#ifndef REEB_GRAPH_CONSTRUCTION_H_
#define REEB_GRAPH_CONSTRUCTION_H_

#include <vector>
#include <unordered_set>
using namespace std;

#include <boost/functional/hash.hpp>

#include <containers/sequential/graph/directed_preds_graph.h>

#include <Vector.h>
using namespace mathtool;

class TetGenDecomposition;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// @brief Compute an embedded reeb graph of free workspace.
///
/// Computation requires a proper tetrahedralization for the computation to
/// work. ReebGraph comprises of ReebNodes and ReebArcs. Nodes represent the
/// critical values of a Morse function, i.e., min, max, saddle, and Arcs are
/// homotopy classes to get between these nodes.
///
/// For details of algorithm see: V. Pascucci, G. Scorzelli, P-T. Bremer, A.
/// Mascarenhas, "Robust On-line Computation of Reeb Graphs: Simplicity and
/// Speed," ACM Trans. on Graphics, 26.3.58, July 2007, and In. Proc. of ACM
/// SIGGRAPH, 2007.
////////////////////////////////////////////////////////////////////////////////
class ReebGraphConstruction {

  public:

    ////////////////////////////////////////////////////////////////////////////
    /// @name Types
    /// {

    typedef tuple<size_t, size_t, size_t> Triangle; ///< Triangle: 3 indices for
                                                    ///< vertex array.

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Vertex property of ReebGraph
    ////////////////////////////////////////////////////////////////////////////
    struct ReebNode {
      //////////////////////////////////////////////////////////////////////////
      /// @param _vIndx Vertex Index
      /// @param _v Vertex
      /// @param _w Morse function value
      ReebNode(size_t _vIndx = -1, const Vector3d& _v = Vector3d(),
          double _w = numeric_limits<double>::max()) :
        m_vertexIndex(_vIndx), m_vertex(_v), m_w(_w) {}

      size_t m_vertexIndex; ///< Vertex Index
      Vector3d m_vertex;    ///< Vertex
      double m_w;           ///< Morse function value
      size_t m_order;       ///< Total ordering of vertex
      size_t m_tetra;       ///< Tetrahedron index from embedding process
    };

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Comparator of ReebNodes. Ordering like tuple: (1) morse function
    ///        value (2-4) vertex[x-z] (5) vertex index
    ////////////////////////////////////////////////////////////////////////////
    struct ReebNodeComp {
      //////////////////////////////////////////////////////////////////////////
      /// @param _a Node 1
      /// @param _b Node 2
      /// @return _a < _b
      bool operator()(const ReebNode& _a, const ReebNode& _b) {
        ////////////////////////////////////////////////////////////////////////
        /// @brief Vertex x, y, z ordering
        /// @param _a Vertex 1
        /// @param _b Vertex 2
        /// @return _a < _b
        auto vertComp = [](const Vector3d _a, const Vector3d _b) {
          return _a[0] - _b[0] > 0.000001 ||
            (fabs(_a[0] - _b[0]) < 0.000001 && (_a[1] - _b[1] > 0.000001 ||
            (fabs(_a[1] - _b[1]) < 0.000001 && _a[2] - _b[2] > 0.000001)));
        };

        const Vector3d& a = _a.m_vertex;
        const Vector3d& b = _b.m_vertex;
        bool ret = _b.m_vertexIndex != _a.m_vertexIndex &&
          (_a.m_w - _b.m_w > 0.000001 ||
           (fabs(_a.m_w - _b.m_w) < 0.000001 &&
            (vertComp(a, b) || (a == b &&
            _a.m_vertexIndex < _b.m_vertexIndex))));
        return ret;
      }
    };

    struct MeshEdge;

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Edge property of ReebGraph. Arc stores set of MeshEdges and
    ///        tetrahedron correspondingly.
    ////////////////////////////////////////////////////////////////////////////
    struct ReebArc {
      //////////////////////////////////////////////////////////////////////////
      /// @param _s Source vertex index
      /// @param _t Target vertex index
      /// @param _m Corresponding mesh edge for initializing
      ReebArc(size_t _s = -1, size_t _t = -1, MeshEdge* _m = nullptr) :
        m_source(_s), m_target(_t) {
          if(_m)
            m_edges.insert(_m);
        }

      size_t m_source;                  ///< Source vertex index
      size_t m_target;                  ///< Target vertex index
      unordered_set<MeshEdge*> m_edges; ///< Related mesh edges
      unordered_set<size_t> m_tetra;    ///< Related tetrahedron
      vector<Vector3d> m_path;          ///< Embedded ReebArc
    };

    typedef stapl::sequential::directed_preds_graph<
      stapl::MULTIEDGES, ReebNode, ReebArc>
      ReebGraph; ///< ReebGraph is a directed, multiedge graph between ReebNode
                 ///< and ReebArc
    typedef ReebGraph::edge_descriptor RGEID; ///< ReebGraph edge descriptor

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Comparator of reeb arcs. Provides total ordering based on (1)
    ///        Source vertex ordering (2) target vertex ordering (3) Edge id
    ////////////////////////////////////////////////////////////////////////////
    struct ReebArcComp {
      //////////////////////////////////////////////////////////////////////////
      /// @param _rg Pointer to ReebGraph
      ReebArcComp(ReebGraph* _rg) : m_rg(_rg) {}

      //////////////////////////////////////////////////////////////////////////
      /// @param _a Edge descriptor 1
      /// @param _b Edge descriptor 2
      /// @return _a < _b
      bool operator()(const RGEID& _a, const RGEID& _b) {
        bool ret = false;
        ReebNode& s0 = m_rg->find_vertex(_a.source())->property();
        ReebNode& s1 = m_rg->find_vertex(_b.source())->property();
        if(s0.m_order < s1.m_order)
          ret = true;
        else if(s0.m_order == s1.m_order) {
          ReebNode& t0 = m_rg->find_vertex(_a.target())->property();
          ReebNode& t1 = m_rg->find_vertex(_b.target())->property();
          if(t0.m_order < t1.m_order)
            ret = true;
          else if(t0.m_order == t1.m_order)
            if(_a.id() < _b.id())
              ret = true;
        }
        return ret;
      }

      ReebGraph* m_rg; ///< ReebGraph pointer
    };

    typedef set<RGEID, ReebArcComp> ArcSet; ///< Set of ReebArcs

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Tetrahedralization edge. Stores associates ReebEdges
    ////////////////////////////////////////////////////////////////////////////
    struct MeshEdge {
      //////////////////////////////////////////////////////////////////////////
      /// @param _s Source vertex index
      /// @param _t Target vertex index
      /// @param _rg ReebGraph pointer
      MeshEdge(size_t _s, size_t _t, ReebGraph* _rg) :
        m_source(_s), m_target(_t), m_numTris(0), m_arcs(ReebArcComp(_rg)) {
        }

      size_t m_source;  ///< Source vertex index
      size_t m_target;  ///< Target vertex index
      size_t m_numTris; ///< Number of triangles which it is incident on
      ArcSet m_arcs;    ///< Set of Reeb Arcs
    };

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Hash function for MeshEdge. Hash based on source/target pair.
    ////////////////////////////////////////////////////////////////////////////
    struct MeshEdgeHash {
      //////////////////////////////////////////////////////////////////////////
      /// @param _m MeshEdge pointer
      /// @return Hash value
      size_t operator()(const MeshEdge* const _m) const {
        return h(make_pair(_m->m_source, _m->m_target));
      }

      boost::hash<pair<size_t, size_t>> h; ///Hash function for evaluation
    };

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Mesh edge equivalence. Based on source/target pair.
    ////////////////////////////////////////////////////////////////////////////
    struct MeshEdgeEq {
      //////////////////////////////////////////////////////////////////////////
      /// @param _a MeshEdge 1
      /// @param _b MeshEdge 2
      /// @return _a == _b
      bool operator()(const MeshEdge* const _a, const MeshEdge* const _b) const {
        return _a->m_source == _b->m_source && _a->m_target == _b->m_target;
      }
    };

    typedef stapl::sequential::graph<
      stapl::DIRECTED, stapl::MULTIEDGES,
      Vector3d, vector<Vector3d>
        > FlowGraph; ///< Flow graph is a directed multiedge graph of points and
                     ///< paths. Is a flow of the embedded ReebGraph.

    /// @}
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    /// @name Constructors
    /// @{

    ////////////////////////////////////////////////////////////////////////////
    /// @param _tetgen TetGen tetrahedralization
    ReebGraphConstruction(TetGenDecomposition* _tetgen);

    /// @}
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    /// @name Accessors
    /// @{

    ////////////////////////////////////////////////////////////////////////////
    /// @return Reeb Graph
    ReebGraph& GetReebGraph() {return m_reebGraph;}

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Compute Flow graph (directed) of ReebGraph from source point
    /// @param _p Start point
    /// @param _posRes Position resolution of edge paths
    /// @return Flow graph and source vertex of Flow Graph related to \c _p
    ///
    /// Computed with modified BFS on Reeb Graph. First, finds closest Reeb Node
    /// to \c p. Second, uses BFS to get directed flow. Note BFS considers cross
    /// edges in the flow, it is not just a BFS tree.
    pair<FlowGraph, size_t> GetFlowGraph(const Vector3d& _p, double _posRes);

    /// @}
    ////////////////////////////////////////////////////////////////////////////


  private:

    ////////////////////////////////////////////////////////////////////////////
    /// @name Construction Helpers
    /// @{

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Construct Reeb Graph based on algorithm presented in class
    ///        description
    ///
    /// Construction Algorithm
    /// - Input: Tetrahedralization t
    /// - Output: Reeb Graph
    /// - 1. Create ReebNode for each vertex of t
    /// - 2. Create ReebArc for each edge of t
    /// - 3. For each triangle in t
    /// - 4.   MergePaths(t)
    /// - 5.   if any edge of t is finished, remove edge from Reeb Graph
    /// - 6. Return Reeb Graph
    void Construct();

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Add ReebNode for vertex \c _v
    /// @param _i Index of vertex
    /// @param _v Vertex point
    /// @param _w Morse function value of vertex
    void CreateNode(size_t _i, const Vector3d& _v, double _w);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Add ReebArc to graph for mesh edge
    /// @param _s Source vertex index
    /// @param _t Target vertex index
    /// @param _tetra Subset of tetrahedrons adjecent to edge
    /// @return MeshEdge between vertices \c _s and \c _t
    ///
    /// If edge between \c _s and \c _t has not been encountered, a ReebArc will
    /// be created first before returning appropriate edge of
    /// tetrahedralization. All tetrahedron in \c _tetra will be associated to
    /// proper ReebArcs.
    MeshEdge* CreateArc(size_t _s, size_t _t,
        const unordered_set<size_t>& _tetra = unordered_set<size_t>());

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Merge triangle of tetrahedralization
    /// @param _e0 Edge 1
    /// @param _e1 Edge 2
    /// @param _e2 Edge 3
    ///
    /// \c _e0, \c _e1, \c _e2 must have proper order. \c _e0 and \c _e1 must
    /// share maximum vertex of triangle. \c _e0 and \c _e2 must share minimum.
    ///
    /// Algorithm from paper
    /// - 1. a0 = Arc(e0); a1 = Arc(e1); a2 = Arc(e2)
    /// - 2. GlueByMergeSorting(a0, a1, e0, e1)
    /// - 3. GlueByMergeSorting(a0, a1, e0, e2)
    void MergePaths(MeshEdge* _e0, MeshEdge* _e1, MeshEdge* _e2);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Merge two ReebArcs. Similar process to merging two arrays in
    ///        MergeSort
    /// @param _a0 ArcSet 1
    /// @param _e0 Edge 1
    /// @param _a1 ArcSet 2
    /// @param _e1 Edge 2
    ///
    /// Algorithm From paper
    /// - while(a0 and a1 are "valid" arcs) do
    /// -   n0 <- BottomNode(a0)
    /// -   n1 <- BottomNode(a1)
    /// -   if f(n0) > f(n1) then
    /// -     MergeArcs(a0, a1)
    /// -   else
    /// -     MergeArcs(a1, a0)
    /// -   a0 <- NextArcMappedToEdge(a0, e0)
    /// -   a1 <- NextArcMappedToEdge(a1, e1)
    void GlueByMergeSorting(ArcSet& _a0, MeshEdge* _e0,
        ArcSet& _a1, MeshEdge* _e1);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Merge second reeb arc into first
    /// @param _a0 ReebArc 1
    /// @param _a1 ReebArc 2
    ///
    /// Merge data of \c _a1 into \c _a0. Sources of \c _a0 and \c _a1 must be
    /// equal. Two cases occur with targets of edges. If targets are equivalent
    /// \c _a1 is entirely removed from the ReebGraph. Otherwise, \c _a1 is
    /// replaced by an edge which spans the target of \c _a0 to the target of
    /// \c _a1.
    void MergeArcs(RGEID _a0, RGEID _a1);

    ////////////////////////////////////////////////////////////////////////////
    /// @param _a ReebGraph edge descriptor
    /// @return ReebArc
    ReebArc& GetReebArc(RGEID _a);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Remove edge from all associated ReebArcs
    /// @param _e Edge of tetrahedralization
    void DeleteMeshEdge(MeshEdge* _e);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Remove 2-nodes from ReebGraph to reduce size of graph
    void Remove2Nodes();

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Spatially embed reeb graph based on algorithm presented in class
    ///        description
    /// @param _tetgen TetGen decomposition of workspace which construction was
    ///                based upon
    ///
    /// Embedding algorithm relates each Reeb Node to its closest tetrahedron
    /// and finds an embedded ReebArc by biasing a path search in the
    /// tetrahedralization's dual graph.
    void Embed(TetGenDecomposition* _tetgen);

    /// @}
    ////////////////////////////////////////////////////////////////////////////

    vector<Vector3d> m_vertices; ///< Vertices of tetrahedralization
    unordered_set<MeshEdge*, MeshEdgeHash, MeshEdgeEq>
      m_edges; ///< Edges of Tetrahedralization
    vector<pair<Triangle, unordered_set<size_t>>>
      m_triangles; ///< Triangles of Tetrahedralization and their corresponding
                   ///< tetrahedrons

    ReebGraph m_reebGraph; ///< Reeb Graph
};

#endif

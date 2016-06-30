#ifndef TET_GEN_DECOMPOSITION_H_
#define TET_GEN_DECOMPOSITION_H_

#include <memory>
using namespace std;

#include <containers/sequential/graph/graph.h>

#define TETLIBRARY
#undef PI
#include "tetgen.h"

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Nef_polyhedron_3.h>

#include <Vector.h>
using namespace mathtool;

#include "IOUtils.h"

class Environment;
class Boundary;
class BoundingBox;
class BoundingSphere;
class StaticMultiBody;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// @brief Tetrahedralization of workspace using TetGen library
////////////////////////////////////////////////////////////////////////////////
class TetGenDecomposition {

  public:

    ///@name Types
    ///@{

    using CGALKernel    = CGAL::Exact_predicates_exact_constructions_kernel;
    using NefPolyhedron = CGAL::Nef_polyhedron_3<CGALKernel>;
    using CGALPoint     = NefPolyhedron::Point_3;

    typedef stapl::sequential::graph<
      stapl::UNDIRECTED, stapl::NONMULTIEDGES,
      Vector3d, double
        > DualGraph; ///< Dual Graph of tetrahedralization

    ///@}
    ///@name Constructors
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @param _switches Switches for TetGen. See TetGen manual. Need 'pn' at a
    ///                  minimum.
    /// @param _writeFreeModel Output TetGen model of workspace
    /// @param _writeDecompModel Output TetGen model of tetrahedralization
    TetGenDecomposition(const string& _switches = "pnqQ",
        bool _writeFreeModel = false, bool _writeDecompModel = false);

    ////////////////////////////////////////////////////////////////////////////
    /// @param _node XML node for input parameters
    TetGenDecomposition(XMLNode& _node);

    ~TetGenDecomposition();

    ///@}
    ///@name Decomposition
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @param _env PMPL Environment as workspace
    /// @param _baseFilename Base filename used for saving models
    void Decompose(Environment* _env, const string& _baseFilename = "");

    ///@}
    ///@name Accessors
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @return Number of points in tetrahedralization
    size_t GetNumPoints() const {return m_decompModel->numberofpoints;}

    ////////////////////////////////////////////////////////////////////////////
    /// @return TetGen points of tetrahedralization. Three double per point.
    const double* const GetPoints() const {return m_decompModel->pointlist;}

    ////////////////////////////////////////////////////////////////////////////
    /// @return Number of points in tetrahedron. Usually 4, but allowed to be 10
    ///         in TetGen. See TetGen manual for details.
    size_t GetNumCorners() const {return m_decompModel->numberofcorners;}

    ////////////////////////////////////////////////////////////////////////////
    /// @return Number of tetrahedron in tetrahedralization
    size_t GetNumTetras() const {return m_decompModel->numberoftetrahedra;}

    ////////////////////////////////////////////////////////////////////////////
    /// @return Tetrahedrons of tetrahedralization. Sets of number of corners of
    ///         indices to points array, e.g., 4 points make a tetrahedron.
    const int* const GetTetras() const {return m_decompModel->tetrahedronlist;}

    ////////////////////////////////////////////////////////////////////////////
    /// @return Neighbor list of tetrahedralization.
    const int* const GetNeighbors() const {return m_decompModel->neighborlist;}

    ////////////////////////////////////////////////////////////////////////////
    /// @return Dual graph of tetrahedralization
    DualGraph& GetDualGraph() {return m_dualGraph;}

    ///@}

  private:

    ///@name Make workspace model
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Add all vertices and facets to free workspace model
    ///
    /// Add obstacles at holes in free workspace model and boundary as the
    /// actual polyhedron.
    void MakeFreeModel();

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Add the vertices to the free model.
    void AddVertices(const NefPolyhedron& _freespace);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Add the facets to the free model.
    void AddFacets(const NefPolyhedron& _freespace);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Add holes to the free model for each obstacle touching the final'
    ///        boundary.
    void AddHoles(const NefPolyhedron& _freespace);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Get the index of a point in the freespace nef poly.
    size_t PointIndex(const NefPolyhedron& _freespace, const CGALPoint& _p) const;

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Extract facet info from the freespace nef poly.
    vector<vector<vector<size_t>>> ExtractFacets(const NefPolyhedron& _freespace);

    ///@}
    ///@name Output helpers
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Output free workspace TetGen model
    void SaveFreeModel();

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Output tetrahedralization TetGen model
    void SaveDecompModel();

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Input tetrahedralization TetGen model
    void LoadDecompModel();

    ///@}
    ///@name Dual Graph Helpers
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Create Dual of tetrahedralization
    void MakeDualGraph();

    ///@}
    ///@name Internal State
    ///@{

    Environment* m_env;      ///< PMPL Environment
    string m_baseFilename;   ///< PMPL base filename

    tetgenio* m_freeModel;   ///< TetGen model of free workspace
    tetgenio* m_decompModel; ///< TetGen model of tetrahedralization

    string m_tetGenFilename; ///< TetGen model input base filename

    string m_switches;       ///< Switches for TetGen. See TetGen manual for
                             ///< details. Need 'pn' at a minimum.
    bool m_writeFreeModel;   ///< Output TetGen model of free workspace
    bool m_writeDecompModel; ///< Output TetGen model of tetrahedralization

    DualGraph m_dualGraph;   ///< Dual of tetrahedralization

    bool m_debug{true};      ///< Toggle debug messages.

    ///@}
};

#endif

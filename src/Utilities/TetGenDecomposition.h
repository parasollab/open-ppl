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
class WorkspaceDecomposition;

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

    ///@}
    ///@name Construction
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @param _switches Switches for TetGen. See TetGen manual. Need 'pn' at a
    ///                  minimum.
    /// @param _baseFilename Base filename used for saving models
    /// @param _writeFreeModel Output TetGen model of workspace
    /// @param _writeDecompModel Output TetGen model of tetrahedralization
    TetGenDecomposition(const string& _baseFilename = "",
        const string& _switches = "pnqQ",
        bool _writeFreeModel = false, bool _writeDecompModel = false);

    TetGenDecomposition(XMLNode& _node); ///< @TODO Add XML parsing.

    ~TetGenDecomposition();

    ///@}
    ///@name Decomposition
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Use tetgen to decompose a given environment.
    /// @param _env PMPL Environment as workspace.
    shared_ptr<WorkspaceDecomposition> operator()(const Environment* _env);

  private:

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Make a workspace decomposition object from the completed tetgen
    ///        model.
    shared_ptr<WorkspaceDecomposition> MakeDecomposition();

    ///@}
    ///@name Freespace Model Creation
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Reset the internal structures.
    void Initialize();

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
    ///@name IO Helpers
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
    ///@name Internal State
    ///@{

    const Environment* m_env{nullptr}; ///< PMPL Environment.
    string m_baseFilename;             ///< PMPL base filename.

    tetgenio* m_freeModel{nullptr};    ///< TetGen model of free workspace.
    tetgenio* m_decompModel{nullptr};  ///< TetGen model of tetrahedralization.

    string m_tetGenFilename;     ///< TetGen model input base filename.

    string m_switches;           ///< Switches for TetGen. See TetGen manual for
                                 ///< details. Need 'pn' at a minimum.
    bool m_writeFreeModel;       ///< Output TetGen model of free workspace.
    bool m_writeDecompModel;     ///< Output TetGen model of tetrahedralization.

    bool m_debug{false};         ///< Toggle debug messages.

    ///@}
};

#endif

#ifndef TET_GEN_DECOMPOSITION_H_
#define TET_GEN_DECOMPOSITION_H_

#include <memory>
using namespace std;

#include <containers/sequential/graph/graph.h>

#define TETLIBRARY
#undef PI
#include "tetgen.h"

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

    ////////////////////////////////////////////////////////////////////////////
    /// @name Types
    /// @{
    typedef stapl::sequential::graph<
      stapl::UNDIRECTED, stapl::NONMULTIEDGES,
      Vector3d, double
        > DualGraph; ///< Dual Graph of tetrahedralization
    /// @}
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    /// @name Constructors
    /// @{
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
    /// @}
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    /// @name Decomposition
    /// @{
    ////////////////////////////////////////////////////////////////////////////
    /// @param _env PMPL Environment as workspace
    /// @param _baseFilename Base filename used for saving models
    void Decompose(Environment* _env, const string& _baseFilename = "");

    /// @}
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    /// @name Accessors
    /// @{
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
    /// @return Dual graph of tetrahedralization
    DualGraph& GetDualGraph() {return m_dualGraph;}
    /// @{
    ////////////////////////////////////////////////////////////////////////////

  private:

    ////////////////////////////////////////////////////////////////////////////
    /// @name Initialize workspace model parameters
    /// @{
    ////////////////////////////////////////////////////////////////////////////
    /// @brief Initialize arrays in tetgenio structure of workspace model
    void InitializeFreeModel();
    ////////////////////////////////////////////////////////////////////////////
    /// @return Number of vertices in free workspace model
    ///
    /// One for each obstacle vertex and one for each boundary vertex
    size_t GetNumVertices() const;
    ////////////////////////////////////////////////////////////////////////////
    /// @param _obst Obstacle
    /// @return Number of vertices in obstacle model
    size_t GetNumVertices(const shared_ptr<StaticMultiBody>& _obst) const;
    ////////////////////////////////////////////////////////////////////////////
    /// @param _boundary Boundary
    /// @return Number of vertices in boundary
    /// @todo Implement for BoundingSphere
    size_t GetNumVertices(const shared_ptr<Boundary>& _boundary) const;
    ////////////////////////////////////////////////////////////////////////////
    /// @return Number of facets in free workspace model
    ///
    /// One for each obstacle facet and one for each boundary facet
    size_t GetNumFacets() const;
    ////////////////////////////////////////////////////////////////////////////
    /// @param _obst Obstacle
    /// @return Number of facets in obstacle model
    size_t GetNumFacets(const shared_ptr<StaticMultiBody>& _obst) const;
    ////////////////////////////////////////////////////////////////////////////
    /// @param _boundary Boundary
    /// @return Number of facets in boundary
    /// @todo Implement for BoundingSphere
    size_t GetNumFacets(const shared_ptr<Boundary>& _boundary) const;
    ////////////////////////////////////////////////////////////////////////////
    /// @return Number of holes in workspace model.
    ///
    /// One for each obstacle.
    size_t GetNumHoles() const;
    /// @}
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    /// @name Make workspace model
    /// @{
    ////////////////////////////////////////////////////////////////////////////
    /// @brief Add all vertices and facets to free workspace model
    ///
    /// Add obstacles at holes in free workspace model and boundary as the
    /// actual polyhedron.
    void MakeFreeModel();
    ////////////////////////////////////////////////////////////////////////////
    /// @brief Add obstacle as hole in free workspace model
    /// @param _obst Obstacle
    /// @param _pOff Offset in tetgenio pointlist
    /// @param _fOff Offset in tetgenio facetlist
    /// @param _hOff Offset in tetgenio holelist
    void AddToFreeModel(const shared_ptr<StaticMultiBody>& _obst,
        size_t _pOff, size_t _fOff, size_t _hOff);
    ////////////////////////////////////////////////////////////////////////////
    /// @brief Add boundary as polyhedron of free workspace model
    /// @param _boundary Boundary
    /// @param _pOff Offset in tetgenio pointlist
    /// @param _fOff Offset in tetgenio facetlist
    void AddToFreeModel(const shared_ptr<Boundary>& _boundary,
        size_t _pOff, size_t _fOff);
    ////////////////////////////////////////////////////////////////////////////
    /// @brief Add bounding box as polyhedron of free workspace model
    /// @param _boundary Bounding box
    /// @param _pOff Offset in tetgenio pointlist
    /// @param _fOff Offset in tetgenio facetlist
    ///
    /// This function's complextiy comes from the case of an obstacle facet
    /// perfectly aligning with the outer boundary facets. In this case, we must
    /// appropriately subtract the obstacle facet from the boundary facet.
    void AddToFreeModel(const shared_ptr<BoundingBox>& _boundary,
        size_t _pOff, size_t _fOff);
    ////////////////////////////////////////////////////////////////////////////
    /// @brief Add bounding sphere as polyhedron of free workspace model
    /// @param _boundary Bounding sphere
    /// @param _pOff Offset in tetgenio pointlist
    /// @param _fOff Offset in tetgenio facetlist
    /// @todo Implement for BoundindSphere
    void AddToFreeModel(const shared_ptr<BoundingSphere>& _boundary,
        size_t _pOff, size_t _fOff);
    /// @}
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    /// @name Output helpers
    /// @{

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Output free workspace TetGen model
    void SaveFreeModel();
    ////////////////////////////////////////////////////////////////////////////
    /// @brief Output tetrahedralization TetGen model
    void SaveDecompModel();
    ////////////////////////////////////////////////////////////////////////////
    /// @brief Input tetrahedralization TetGen model
    void LoadDecompModel();

    /// @}
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    /// @name Dual Graph Helpers
    /// @{

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Create Dual of tetrahedralization
    void MakeDualGraph();

    /// @}
    ////////////////////////////////////////////////////////////////////////////

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
};

#endif

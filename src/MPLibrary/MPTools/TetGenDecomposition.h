#ifndef TET_GEN_DECOMPOSITION_H_
#define TET_GEN_DECOMPOSITION_H_

using namespace std;

#include <containers/sequential/graph/graph.h>

#include <Vector.h>
using namespace mathtool;

#include "Utilities/IOUtils.h"

class Environment;
class StaticMultiBody;
class WorkspaceDecomposition;
class XMLNode;

class tetgenio;


////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// Tetrahedralization of workspace using TetGen library
///
/// TetGen offers two levels of courseness - maximally course (default) and
/// 'quality' (use 'q' in switches).
////////////////////////////////////////////////////////////////////////////////
class TetGenDecomposition {

  public:

    ///@}
    ///@name Construction
    ///@{

    TetGenDecomposition() = default;

    /// Create a TetGen decomposer manually. If a base file name is provided,
    /// output the decomposed model.
    /// @param _switches The switches to use.
    /// @param _baseFilename Base filename for output files.
    TetGenDecomposition(const std::string& _switches,
        const std::string& _baseFilename = "");

    /// Construct a TetGen decomposer from an XML node.
    /// @param _node The XML node to parse.
    TetGenDecomposition(XMLNode& _node);

    ///@}
    ///@name Decomposition
    ///@{

    /// Use tetgen to decompose a given environment.
    /// @param _env PMPL Environment as workspace.
    const WorkspaceDecomposition* operator()(const Environment* _env);

  private:

    /// Make a workspace decomposition object from the completed tetgen model.
    const WorkspaceDecomposition* MakeDecomposition();

    ///@}
    ///@name Freespace Model Creation
    ///@{

    /// Add all vertices and facets to free workspace model
    ///
    /// Add obstacles at holes in free workspace model and boundary as the
    /// actual polyhedron.
    void MakeFreeModel(const Environment* _env);

    ///@}
    ///@name IO Helpers
    ///@{

    /// Ensure that a switch string contains at least 'p' and 'n'. Add them if
    /// not found.
    /// @param _switches The string to validate.
    static void ValidateSwitches(std::string& _switches);

    /// Output free workspace TetGen model
    void SaveFreeModel();

    /// Output tetrahedralization TetGen model
    void SaveDecompModel();

    /// Input tetrahedralization TetGen model
    void LoadDecompModel();

    ///@}
    ///@name Internal State
    ///@{

    std::string m_switches{"pnQ"};  ///< See TetGen manual, need at least 'pn'.

    enum IOOperation {Read, Write, None}; ///< Types of I/O operation.
    IOOperation m_ioType{None};           ///< Read or write model to file?
    std::string m_baseFilename;           ///< Base file name for I/O.

    bool m_debug{false};               ///< Toggle debug messages.

    tetgenio* m_freeModel{nullptr};    ///< TetGen model of free workspace.
    tetgenio* m_decompModel{nullptr};  ///< TetGen model of tetrahedralization.

    ///@}
};

#endif

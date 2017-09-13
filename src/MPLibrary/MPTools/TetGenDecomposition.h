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
/// 'quality' (use 'q' in switches.
////////////////////////////////////////////////////////////////////////////////
class TetGenDecomposition {

  public:

    ///@name Types
    ///@{

    /// The input parameters for this object.
    struct Parameters {
      std::string inputFilename;    ///< Input file name, if any.
      std::string switches{"pnQ"};  ///< See TetGen manual, need at least 'pn'.
      bool writeFreeModel{false};   ///< Output model of free workspace.
      bool writeDecompModel{false}; ///< Output tetrahedralization.
      bool debug{false};            ///< Toggle debug messages.
    };

    ///@}
    ///@name Construction
    ///@{

    TetGenDecomposition() = default;

    /// @param _baseFilename Base filename used for all pmpl outputs.
    TetGenDecomposition(const std::string& _baseFilename);

    /// Construct a TetGen decomposer with designated parameters.
    /// @param _baseFilename Base filename used for output files.
    /// @param _params The input parameters for tetgen.
    TetGenDecomposition(const std::string& _baseFilename, Parameters&& _params);

    /// Construct a TetGen decomposer from an XML node.
    /// @param _node The XML node to parse.
    TetGenDecomposition(XMLNode& _node);

    ~TetGenDecomposition();

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
    void MakeFreeModel();

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

    Parameters m_params;               ///< Input parameters for this instance.

    const Environment* m_env{nullptr}; ///< PMPL Environment.
    std::string m_baseFilename;        ///< PMPL base filename.

    tetgenio* m_freeModel{nullptr};    ///< TetGen model of free workspace.
    tetgenio* m_decompModel{nullptr};  ///< TetGen model of tetrahedralization.

    ///@}
};

#endif

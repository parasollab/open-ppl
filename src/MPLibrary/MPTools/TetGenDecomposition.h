#ifndef TET_GEN_DECOMPOSITION_H_
#define TET_GEN_DECOMPOSITION_H_

#include <memory>
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

    static Parameters m_defaultParams; ///< The default parameters.

    ///@}
    ///@name Construction
    ///@{

    /// @param _baseFilename Base filename used for all pmpl outputs.
    TetGenDecomposition(const std::string& _baseFilename);

    /// @param _baseFilename Base filename used for all pmpl outputs.
    /// @param _switches Switches for TetGen. See TetGen manual. Need 'pn' at a
    ///                  minimum.
    /// @param _writeFreeModel Output TetGen model of workspace
    /// @param _writeDecompModel Output TetGen model of tetrahedralization
    /// @param _inputFilename Input filename for the decomposition model. Empty
    ///                       means do not read input.
    TetGenDecomposition(const string& _baseFilename,
        const std::string& _switches,
        const bool _writeFreeModel, const bool _writeDecompModel,
        const std::string& _inputFilename = "");

    /// Set the default parameters from an XML node.
    /// @param _node The XML node to parse.
    static void SetDefaultParameters(XMLNode& _node);

    ~TetGenDecomposition();

    ///@}
    ///@name Decomposition
    ///@{

    /// Use tetgen to decompose a given environment.
    /// @param _env PMPL Environment as workspace.
    shared_ptr<WorkspaceDecomposition> operator()(const Environment* _env);

  private:

    /// Make a workspace decomposition object from the completed tetgen model.
    shared_ptr<WorkspaceDecomposition> MakeDecomposition();

    ///@}
    ///@name Freespace Model Creation
    ///@{

    /// Reset the internal structures.
    void Initialize();

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

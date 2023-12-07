#ifndef PATH_MODIFIER_METHOD_H_
#define PATH_MODIFIER_METHOD_H_

#include "MPLibrary/MPBaseObject.h"
#include "Utilities/MPUtils.h"
#include "MPLibrary/LocalPlanners/LPOutput.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup PathModifiers
/// @brief Base algorithm abstraction for \ref PathModifiers.
///
/// PathModifierMethod has one main method, @c Modify, which takes an input path
/// and produces a valid output path.
////////////////////////////////////////////////////////////////////////////////
class PathModifierMethod : public MPBaseObject {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPBaseObject::RoadmapType RoadmapType;
    typedef typename RoadmapType::VID       VID;

    ///@}
    ///@name Construction
    ///@{

    PathModifierMethod() = default;

    PathModifierMethod(XMLNode& _node);

    virtual ~PathModifierMethod() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const override;

    ///@}
    ///@name PathModifier Interface
    ///@{

    /// Modifies the input path to a new valid path
    /// @param _path A path of configurations within a resolution
    ///        distance of each other
    /// @param _newPath An empty vector to place the resulting modified path
    ///
    /// @usage
    /// @code
    /// PathModifierPointer pm = this->GetPathModifier(m_pmLabel);
    /// vector<Cfg> inputPath, outputPath;
    /// pm->Modify(inputPath, outputPath);
    /// @endcode
    virtual void Modify(vector<Cfg>& _path,
        vector<Cfg>& _newPath);
    ///@example PathModifiers_UseCase.cpp
    /// This is an example of how to use the path modifier methods.

    virtual void Modify(RoadmapType* _graph, vector<Cfg>& _path,
                        vector<Cfg>& _newPath);

  protected:

    /// Modifies the input path to a new valid path
    /// @param _path A path of configurations within a resolution
    ///        distance of each other
    /// @param _newPath An empty vector to place the resulting modified path
    /// @return Success/failed modification
    virtual bool ModifyImpl(RoadmapType* _graph, vector<Cfg>& _path,
        vector<Cfg>& _newPath) = 0;

    /// Appends local plan to path
    /// @param _path Path to append local plan to
    /// @param _lpOutput Local plan output
    /// @param _end End Cfg of local plan
    void AddToPath(vector<Cfg>& _path, LPOutput* _lpOutput,
        Cfg& _end);

    /// Extract path VIDs in roadmap from path
    /// @param _path Path to extract VIDs from
    /// @param _graph RoadmapGraph containing path nodes
    /// @return Path VIDs
    vector<VID> GetPathVIDs(vector<Cfg>& _path, RoadmapType* _graph);

    /// @TODO Figure out what this does and document it.
    void RemoveBranches(const string& _dmLabel, vector<Cfg>& _path,
        vector<Cfg>& _newPath);

  private:

    /// Write path to file
    /// @param _path Path
    void OutputPath(vector<Cfg>& _path);

    ///@}
    ///@name Internal State
    ///@{

    string m_pathFile; ///< Where to write the smoothed path.

    ///@}
};

#endif

#ifndef MP_TOOLS_H_
#define MP_TOOLS_H_

#include <string>
#include <unordered_map>

#include "Utilities/XMLNode.h"

#include "SafeIntervalTool.h"

class WorkspaceDecomposition;


////////////////////////////////////////////////////////////////////////////////
/// This class is a general tool box for stateful objects that don't belong
/// elsewhere. It gives us a way to do XML parsing and uniform access for these
/// odd-balls like medial axis tools.
///
/// For most utilities, this object creates and maintains a map of string ->
/// instance. One instance will be generated for each MedialAxisUtility node in
/// the XML file (with corresponding label). Instances can also be added
/// manually with the Set functions.
///
/// For the other tools, this object parses a single XML node to set default
/// parameters for the tool classes. Default-constructed tools then use those
/// values. This effectively uses the XML to set default parameters so that we
/// don't have to create an instance of the tool right away (or at all) in order
/// to parse the input file. For these, there is no label attribute, and using
/// multiple XML nodes will throw a parse error.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class MPToolsType final {

  ///@name Motion Planning Types
  ///@{

  typedef typename MPTraits::MPLibrary MPLibrary;

  ///@}
  ///@name Local Types
  ///@{

  template <template <typename> class Utility>
  using LabelMap = std::unordered_map<std::string, Utility<MPTraits>*>;

  ///@}
  ///@name Internal State
  ///@{

  MPLibrary* const m_library; ///< The owning library.

  LabelMap<SafeIntervalTool>         m_safeIntervalTools;

  ///@}

  public:

    ///@name Construction
    ///@{

    /// Construct a tool set.
    /// @param _library The owning library.
    MPToolsType(MPLibrary* const _library);

    /// Parse an XML node.
    /// @param _node The XML node object.
    void ParseXML(XMLNode& _node);

    /// Initialize the clearance and MA tools prior to use.
    void Initialize();

    /// Uninitialize the clearance and MA tools.
    void Uninitialize();

    ~MPToolsType();

    ///@}
    ///@name Safe Interval Tool
    ///@{

    /// Get a SafeIntervalTool by label.
    /// @param _label The string label of the desired utility as defined in the
    ///               XML file.
    /// @return The labeled utility.
    SafeIntervalTool<MPTraits>* GetSafeIntervalTool(const std::string& _label)
        const;

    /// Set a SafeIntervalTool. This object will take ownership of the utility and
    /// delete it when necessary.
    /// @param _label The string label for the new utility.
    /// @param _utility The TopologicalMap to use.
    void SetSafeIntervalTool(const std::string& _label,
        SafeIntervalTool<MPTraits>* const _utility);

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Get a utility in a label map. Throws if not found.
    /// @param _label The utility label.
    /// @param _map The label map which holds _utility.
    /// @return The named utility.
    template <template <typename> class Utility>
    Utility<MPTraits>* GetUtility(const std::string& _label,
        const LabelMap<Utility>& _map) const;


    /// Set a utility in a label map.
    /// @param _label The utility label.
    /// @param _utility The utility to set.
    /// @param _map The label map which holds _utility.
    template <template <typename> class Utility>
    void SetUtility(const std::string& _label, Utility<MPTraits>* _utility,
        LabelMap<Utility>& _map) const;

    ///@}

};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
MPToolsType<MPTraits>::
MPToolsType(MPLibrary* const _library) : m_library(_library) { }


template <typename MPTraits>
void
MPToolsType<MPTraits>::
ParseXML(XMLNode& _node) {

  // MPTools shouldn't have any data of its own, only child nodes.
  for(auto& child : _node) {
    if(child.Name() == "SafeIntervalTool") {
      auto utility = new SafeIntervalTool<MPTraits>(child);

      // A second node with the same label is an error during XML parsing.
      if(m_safeIntervalTools.count(utility->GetLabel()))
        throw ParseException(child.Where(), "Second SafeIntervalTool "
            "node with the label '" + utility->GetLabel() + "'. Labels must be "
            "unique.");

      SetSafeIntervalTool(utility->GetLabel(), utility);
    }
  }
}


template <typename MPTraits>
void
MPToolsType<MPTraits>::
Initialize() {
  for(auto& pair : m_safeIntervalTools)
    pair.second->Initialize();
}

template <typename MPTraits>
void
MPToolsType<MPTraits>::
Uninitialize() {
}

template <typename MPTraits>
MPToolsType<MPTraits>::
~MPToolsType() {
  for(auto& pair : m_safeIntervalTools)
    delete pair.second;
}

/*---------------------------- Safe Interval Tool ----------------------------*/

template <typename MPTraits>
SafeIntervalTool<MPTraits>*
MPToolsType<MPTraits>::
GetSafeIntervalTool(const std::string& _label) const {
  return GetUtility(_label, m_safeIntervalTools);
}


template <typename MPTraits>
void
MPToolsType<MPTraits>::
SetSafeIntervalTool(const std::string& _label,
    SafeIntervalTool<MPTraits>* const _utility) {
  SetUtility(_label, _utility, m_safeIntervalTools);
}

/*---------------------------------- Helpers ---------------------------------*/

template <typename MPTraits>
template <template <typename> class Utility>
inline
Utility<MPTraits>*
MPToolsType<MPTraits>::
GetUtility(const std::string& _label, const LabelMap<Utility>& _map) const {
  try {
    return _map.at(_label);
  }
  catch(const std::out_of_range&) {
    Utility<MPTraits> dummy;
    throw RunTimeException(WHERE) << "Requested " << dummy.GetName()
                                  << " '" << _label  << "' does not exist.";
  }
  catch(const std::exception& _e) {
    Utility<MPTraits> dummy;
    throw RunTimeException(WHERE) << "Error when fetching " << dummy.GetName()
                                  << " '" << _label << "': " << _e.what();
  }
  catch(...) {
    Utility<MPTraits> dummy;
    throw RunTimeException(WHERE) << "Error when fetching " << dummy.GetName()
                                  << " '" << _label << "': (unknown).";
  }
}


template <typename MPTraits>
template <template <typename> class Utility>
void
MPToolsType<MPTraits>::
SetUtility(const std::string& _label, Utility<MPTraits>* _utility,
    LabelMap<Utility>& _map) const {
  // Set the library pointer.
  _utility->SetMPLibrary(m_library);

  // Check if this label is already in use.
  auto iter = _map.find(_label);
  const bool alreadyExists = iter != _map.end();

  // If the label already exists, we need to release the previous utility first.
  if(alreadyExists) {
    delete iter->second;
    iter->second = _utility;
  }
  else
    _map.insert({_label, _utility});
}

/*----------------------------------------------------------------------------*/

#endif

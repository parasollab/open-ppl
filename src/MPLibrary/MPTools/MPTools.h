#ifndef MP_TOOLS_H_
#define MP_TOOLS_H_

#include <string>
#include <unordered_map>

#include "MedialAxisUtilities.h"
#include "ReebGraphConstruction.h"
#include "TetGenDecomposition.h"

#include "Utilities/XMLNode.h"


////////////////////////////////////////////////////////////////////////////////
/// This class is a general tool box for stateful objects that don't belong
/// elsewhere. It gives us a way to do XML parsing and uniform access for these
/// odd-balls like medial axis tools.
///
/// For the medial axis and clearance utilities, this object creates and
/// maintains a map of string -> instance. One instance will be generated for
/// each MedialAxisUtility node in the XML file (with corresponding label).
/// Instances can also be added manually with the Set functions.
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
  ///@name Internal State
  ///@{

  MPLibrary* const m_library; ///< The owning library.

  std::unordered_map<std::string, ClearanceUtility<MPTraits>*>  m_clearanceUtils;
  std::unordered_map<std::string, MedialAxisUtility<MPTraits>*> m_medialAxisUtils;

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

    ~MPToolsType();

    ///@}
    ///@name Clearance Utility
    ///@{

    /// Get a ClearanceUtility by label.
    /// @param _label The string label of the desired utility as defined in the
    ///               XML file.
    /// @return The labeled utility.
    ClearanceUtility<MPTraits>* GetClearanceUtility(const std::string& _label)
        const;

    /// Set the ClearanceUtility. This object will take ownership of the utility
    /// and delete it when necessary.
    /// @param _label The string label for the new utility.
    /// @param _utility The ClearanceUtility to use.
    void SetClearanceUtility(const std::string& _label,
        ClearanceUtility<MPTraits>* const _utility);

    ///@}
    ///@name Medial Axis Utility
    ///@{

    /// Get a MedialAxisUtility by label.
    /// @param _label The string label of the desired utility as defined in the
    ///               XML file.
    /// @return The labeled utility.
    MedialAxisUtility<MPTraits>* GetMedialAxisUtility(const std::string& _label)
        const;

    /// Set the MedialAxisUtility. This object will take ownership of the utility
    /// and delete it when necessary.
    /// @param _label The string label for the new utility.
    /// @param _utility The MedialAxisUtility to use.
    void SetMedialAxisUtility(const std::string& _label,
        MedialAxisUtility<MPTraits>* const _utility);

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
  // For the tools that use the XML to set defaults, keep track of whether we've
  // seen them before.
  bool parsedTetGen = false,
       parsedReebGraph = false;

  // MPTools shouldn't have any data of its own, only child nodes.
  for(auto& child : _node) {
    if(child.Name() == "ClearanceUtility") {
      auto utility = new ClearanceUtility<MPTraits>(child);

      // A second node with the same label is an error during XML parsing.
      if(m_clearanceUtils.count(utility->GetLabel()))
        throw ParseException(child.Where(), "Second ClearanceUtility node with "
            "the label '" + utility->GetLabel() + "'. Labels must be unique.");

      SetClearanceUtility(utility->GetLabel(), utility);
    }
    else if(child.Name() == "MedialAxisUtility") {
      auto utility = new MedialAxisUtility<MPTraits>(child);

      // A second node with the same label is an error during XML parsing.
      if(m_medialAxisUtils.count(utility->GetLabel()))
        throw ParseException(child.Where(), "Second MedialAxisUtility node with "
            "the label '" + utility->GetLabel() + "'. Labels must be unique.");

      SetMedialAxisUtility(utility->GetLabel(), utility);
    }
    else if(child.Name() == "TetGenDecomposition") {
      if(parsedTetGen)
        throw ParseException(child.Where(),
            "Second TetGenDecomposition node detected. This node sets default "
            "parameters - only one is allowed.");
      parsedTetGen = true;

      TetGenDecomposition::SetDefaultParameters(child);
    }
    else if(child.Name() == "ReebGraphConstruction") {
      if(parsedReebGraph)
        throw ParseException(child.Where(),
            "Second ReebGraphConstruction node detected. This node sets "
            "default parameters - only one is allowed.");
      parsedReebGraph = true;

      ReebGraphConstruction::SetDefaultParameters(child);
    }
  }
}


template <typename MPTraits>
void
MPToolsType<MPTraits>::
Initialize() {
  for(auto& pair : m_clearanceUtils)
    pair.second->Initialize();
  for(auto& pair : m_medialAxisUtils)
    pair.second->Initialize();
}


template <typename MPTraits>
MPToolsType<MPTraits>::
~MPToolsType() {
  for(auto& pair : m_clearanceUtils)
    delete pair.second;
  for(auto& pair : m_medialAxisUtils)
    delete pair.second;
}

/*--------------------------- Clearance Utility ------------------------------*/

template <typename MPTraits>
ClearanceUtility<MPTraits>*
MPToolsType<MPTraits>::
GetClearanceUtility(const std::string& _label) const {
  return m_clearanceUtils.at(_label);
}


template <typename MPTraits>
void
MPToolsType<MPTraits>::
SetClearanceUtility(const std::string& _label,
    ClearanceUtility<MPTraits>* const _utility) {
  _utility->SetMPLibrary(m_library);

  // Check if this label is already in use.
  auto iter = m_clearanceUtils.find(_label);
  const bool alreadyExists = iter != m_clearanceUtils.end();

  // If the label already exists, we need to release the previous utility first.
  if(alreadyExists) {
    delete iter->second;
    iter->second = _utility;
  }
  else
    m_clearanceUtils.insert({_label, _utility});
}


/*-------------------------- Medial Axis Utility -----------------------------*/

template <typename MPTraits>
MedialAxisUtility<MPTraits>*
MPToolsType<MPTraits>::
GetMedialAxisUtility(const std::string& _label) const {
  return m_medialAxisUtils.at(_label);
}


template <typename MPTraits>
void
MPToolsType<MPTraits>::
SetMedialAxisUtility(const std::string& _label,
    MedialAxisUtility<MPTraits>* const _utility) {
  _utility->SetMPLibrary(m_library);

  // Check if this label is already in use.
  auto iter = m_medialAxisUtils.find(_label);
  const bool alreadyExists = iter != m_medialAxisUtils.end();

  // If the label already exists, we need to release the previous utility first.
  if(alreadyExists) {
    delete iter->second;
    iter->second = _utility;
  }
  else
    m_medialAxisUtils.insert({_label, _utility});
}

/*----------------------------------------------------------------------------*/

#endif

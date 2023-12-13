#include "MPTools.h"

#include "MPLibrary/MPLibrary.h"

/*------------------------------ Construction --------------------------------*/

MPToolsType::
MPToolsType(MPLibrary* const _library) : m_library(_library) { }


void
MPToolsType::
ParseXML(XMLNode& _node) {
  // For the tools that use the XML to set defaults, keep track of whether we've
  // seen them before.
  bool parsedReebGraph = false,
//       parsedSVMModel  = false,
       parsedMCS       = false;

  // MPTools shouldn't have any data of its own, only child nodes.
  for(auto& child : _node) {
    if(child.Name() == "ClearanceUtility") {
      auto utility = new ClearanceUtility(child);

      // A second node with the same label is an error during XML parsing.
      if(m_clearanceUtils.count(utility->GetLabel()))
        throw ParseException(child.Where(), "Second ClearanceUtility node with "
            "the label '" + utility->GetLabel() + "'. Labels must be unique.");

      SetClearanceUtility(utility->GetLabel(), utility);
    }
    else if(child.Name() == "MedialAxisUtility") {
      auto utility = new MedialAxisUtility(child);

      // A second node with the same label is an error during XML parsing.
      if(m_medialAxisUtils.count(utility->GetLabel()))
        throw ParseException(child.Where(), "Second MedialAxisUtility node with "
            "the label '" + utility->GetLabel() + "'. Labels must be unique.");

      SetMedialAxisUtility(utility->GetLabel(), utility);
    }
    else if(child.Name() == "SkeletonClearanceUtility") {
      auto utility = new SkeletonClearanceUtility(child);

      // A second node with the same label is an error during XML parsing.
      if(m_skeletonUtils.count(utility->GetLabel()))
        throw ParseException(child.Where(), "Second SkeletonClearanceUtility "
            "node with the label '" + utility->GetLabel() + "'. Labels must be "
            "unique.");

      SetSkeletonClearanceUtility(utility->GetLabel(), utility);
    }
    else if(child.Name() == "TopologicalMap") {
      auto utility = new TopologicalMap(child);

      // A second node with the same label is an error during XML parsing.
      if(m_topologicalMaps.count(utility->GetLabel()))
        throw ParseException(child.Where(), "Second TopologicalMap "
            "node with the label '" + utility->GetLabel() + "'. Labels must be "
            "unique.");

      SetTopologicalMap(utility->GetLabel(), utility);
    }
    else if(child.Name() == "TetGenDecomposition") {
      // Parse the label and check that it is unique.
      const std::string label = child.Read("label", true, "",
          "The label for this decomposition.");

      if(m_decompositions.count(label))
        throw ParseException(child.Where(), "Second decomposition node "
            "with the label " + label + ". Labels must be unique across all "
            "types of decomposition.");

      m_tetgens[label] = TetGenDecomposition(child);
      SetDecomposition(label, nullptr);
    }
    else if(child.Name() == "SafeIntervalTool") {
      auto utility = new SafeIntervalTool(child);

      // A second node with the same label is an error during XML parsing.
      if(m_safeIntervalTools.count(utility->GetLabel()))
        throw ParseException(child.Where(), "Second SafeIntervalTool "
            "node with the label '" + utility->GetLabel() + "'. Labels must be "
            "unique.");

      SetSafeIntervalTool(utility->GetLabel(), utility);
    }
    else if(child.Name() == "WrenchAccessibilityTool") {
      auto utility = new WrenchAccessibilityTool(child);

      // A second node with the same label is an error during XML parsing.
      if(m_wrenchAccessibilityTools.count(utility->GetLabel()))
        throw ParseException(child.Where(), "Second WrenchAccessibilityTool "
            "node with the label '" + utility->GetLabel() + "'. Labels must be "
            "unique.");

      SetWrenchAccessibilityTool(utility->GetLabel(), utility);
    }
    // Below here we are setting defaults rather than creating instances.
    else if(child.Name() == "ReebGraphConstruction") {
      if(parsedReebGraph)
        throw ParseException(child.Where(),
            "Second ReebGraphConstruction node detected. This node sets "
            "default parameters - only one is allowed.");
      parsedReebGraph = true;

      ReebGraphConstruction::SetDefaultParameters(child);
    }
    else if(child.Name() == "MeanCurvatureSkeleton3D") {
      if(parsedMCS)
        throw ParseException(child.Where(),
            "Second meanCurvatureSkeleton3D node detected. This node sets "
            "default parameters - only one is allowed.");
      parsedMCS = true;

      MeanCurvatureSkeleton3D::SetDefaultParameters(child);
    }
/*    else if(child.Name() == "SVMModel") {
      if(parsedSVMModel)
        throw ParseException(child.Where(), "Second SVMModel node detected. "
            "This node sets default parameters - only one is allowed.");
      parsedSVMModel = true;

      SVMModel::SetDefaultParameters(child);
    }
*/
    else if(child.Name() == "ReachabilityUtil") {
      auto util = new ReachabilityUtil(child);
      SetReachabilityUtil(util->GetLabel(), util);
    }
    
    else if(child.Name() == "PointConstruction") {
      auto util = new PointConstruction(child);
      SetPointConstruction(util->GetLabel(), util);
    }
  }
}


void
MPToolsType::
Initialize() {
  //Uninitialize();
  for(auto& pair : m_clearanceUtils)
    pair.second->Initialize();
  for(auto& pair : m_medialAxisUtils)
    pair.second->Initialize();
  for(auto& pair : m_skeletonUtils)
    pair.second->Initialize();
  for(auto& pair : m_topologicalMaps)
    pair.second->Initialize();
  for(auto& pair : m_safeIntervalTools)
    pair.second->Initialize();
  for(auto& pair : m_reachabilityUtils)
    pair.second->Initialize();
  for(auto& pair : m_pointConstruction)
    pair.second->Initialize();
  for(auto& pair : m_wrenchAccessibilityTools)
    pair.second->Initialize();
}


void
MPToolsType::
Uninitialize() {
  for(auto& pair : m_decompositions){
    delete pair.second;
    pair.second = nullptr;
  }
}


MPToolsType::
~MPToolsType() {
  for(auto& pair : m_clearanceUtils)
    delete pair.second;
  for(auto& pair : m_medialAxisUtils)
    delete pair.second;
  for(auto& pair : m_skeletonUtils)
    delete pair.second;
  for(auto& pair : m_topologicalMaps)
    delete pair.second;
  for(auto& pair : m_safeIntervalTools)
    delete pair.second;
  for(auto& pair : m_decompositions)
    delete pair.second;
  for(auto& pair : m_reachabilityUtils)
    delete pair.second;
  for(auto& pair : m_pointConstruction) 
    delete pair.second;
  for(auto& pair : m_wrenchAccessibilityTools)
    delete pair.second;
}

/*--------------------------- Clearance Utility ------------------------------*/

ClearanceUtility*
MPToolsType::
GetClearanceUtility(const std::string& _label) const {
  return GetUtility(_label, m_clearanceUtils);
}


void
MPToolsType::
SetClearanceUtility(const std::string& _label,
    ClearanceUtility* const _utility) {
  SetUtility(_label, _utility, m_clearanceUtils);
}

/*-------------------------- Medial Axis Utility -----------------------------*/

MedialAxisUtility*
MPToolsType::
GetMedialAxisUtility(const std::string& _label) const {
  return GetUtility(_label, m_medialAxisUtils);
}


void
MPToolsType::
SetMedialAxisUtility(const std::string& _label,
    MedialAxisUtility* const _utility) {
  SetUtility(_label, _utility, m_medialAxisUtils);
}

/*----------------------------- Skeleton Tools -------------------------------*/

SkeletonClearanceUtility*
MPToolsType::
GetSkeletonClearanceUtility(const std::string& _label) const {
  return GetUtility(_label, m_skeletonUtils);
}


void
MPToolsType::
SetSkeletonClearanceUtility(const std::string& _label,
    SkeletonClearanceUtility* const _utility) {
  SetUtility(_label, _utility, m_skeletonUtils);
}

/*------------------------------ Topological Map -----------------------------*/

TopologicalMap*
MPToolsType::
GetTopologicalMap(const std::string& _label) const {
  return GetUtility(_label, m_topologicalMaps);
}


void
MPToolsType::
SetTopologicalMap(const std::string& _label,
    TopologicalMap* const _utility) {
  SetUtility(_label, _utility, m_topologicalMaps);
}

/*---------------------------- Safe Interval Tool ----------------------------*/

SafeIntervalTool*
MPToolsType::
GetSafeIntervalTool(const std::string& _label) const {
  return GetUtility(_label, m_safeIntervalTools);
}


void
MPToolsType::
SetSafeIntervalTool(const std::string& _label,
    SafeIntervalTool* const _utility) {
  SetUtility(_label, _utility, m_safeIntervalTools);
}

/*----------------------------- Decompositions -------------------------------*/

const WorkspaceDecomposition*
MPToolsType::
GetDecomposition(const std::string& _label) {
  // Initialize the decomposition if not already.
  typename decltype(m_decompositions)::iterator iter;
  try {
    iter = m_decompositions.find(_label);
  }
  catch(const std::out_of_range&) {
    throw RunTimeException(WHERE) << "Requested decomposition '" << _label
                                  << "' does not exist.";
  }

  if(iter->second == nullptr) {
    MethodTimer mt(m_library->GetStatClass(), "TetGenDecomposition::" + _label);
    iter->second = m_tetgens[_label](m_library->GetMPProblem()->GetEnvironment());
  }
  return iter->second;
}


void
MPToolsType::
SetDecomposition(const std::string& _label,
    const WorkspaceDecomposition* _decomposition) {
  // If a decomposition was already assigned to this label, delete it before
  // storing the new one.
  auto iter = m_decompositions.find(_label);
  const bool alreadyExists = iter != m_decompositions.end();

  if(alreadyExists) {
    delete iter->second;
    iter->second = _decomposition;
  }
  else
    m_decompositions[_label] = _decomposition;
}

/*----------------------------- Reachability Utils -------------------------------*/

ReachabilityUtil*
MPToolsType::
GetReachabilityUtil(const std::string& _label) const {
  return GetUtility(_label, m_reachabilityUtils);
}


void
MPToolsType::
SetReachabilityUtil(const std::string& _label,
    ReachabilityUtil* _util) {
  SetUtility(_label, _util, m_reachabilityUtils);
}

/*----------------------------- Point Construction -------------------------------*/

PointConstruction*
MPToolsType::
GetPointConstruction(const std::string& _label) const {
  return GetUtility(_label, m_pointConstruction);
}


void
MPToolsType::
SetPointConstruction(const std::string& _label,
    PointConstruction* _util) {
  SetUtility(_label, _util, m_pointConstruction);
}

/*---------------------- Wrench Accessibility Tools --------------------------*/

WrenchAccessibilityTool*
MPToolsType::
GetWrenchAccessibilityTool(const std::string& _label) const {
  return GetUtility(_label, m_wrenchAccessibilityTools);
}


void
MPToolsType::
SetWrenchAccessibilityTool(const std::string& _label,
    WrenchAccessibilityTool* const _utility) {
  SetUtility(_label, _utility, m_wrenchAccessibilityTools);
}

#include "MPTools.h"

/*------------------------------ Construction --------------------------------*/

// template <typename MPTraits>
// MPToolsType<MPTraits>::
// MPToolsType(MPLibrary* const _library) : m_library(_library) { }


// template <typename MPTraits>
// void
// MPToolsType<MPTraits>::
// ParseXML(XMLNode& _node) {
//   // For the tools that use the XML to set defaults, keep track of whether we've
//   // seen them before.
//   bool parsedReebGraph = false,
// //       parsedSVMModel  = false,
//        parsedMCS       = false;

//   // MPTools shouldn't have any data of its own, only child nodes.
//   for(auto& child : _node) {
//     if(child.Name() == "ClearanceUtility") {
//       auto utility = new ClearanceUtility<MPTraits>(child);

//       // A second node with the same label is an error during XML parsing.
//       if(m_clearanceUtils.count(utility->GetLabel()))
//         throw ParseException(child.Where(), "Second ClearanceUtility node with "
//             "the label '" + utility->GetLabel() + "'. Labels must be unique.");

//       SetClearanceUtility(utility->GetLabel(), utility);
//     }
//     else if(child.Name() == "MedialAxisUtility") {
//       auto utility = new MedialAxisUtility<MPTraits>(child);

//       // A second node with the same label is an error during XML parsing.
//       if(m_medialAxisUtils.count(utility->GetLabel()))
//         throw ParseException(child.Where(), "Second MedialAxisUtility node with "
//             "the label '" + utility->GetLabel() + "'. Labels must be unique.");

//       SetMedialAxisUtility(utility->GetLabel(), utility);
//     }
//     else if(child.Name() == "SkeletonClearanceUtility") {
//       auto utility = new SkeletonClearanceUtility<MPTraits>(child);

//       // A second node with the same label is an error during XML parsing.
//       if(m_skeletonUtils.count(utility->GetLabel()))
//         throw ParseException(child.Where(), "Second SkeletonClearanceUtility "
//             "node with the label '" + utility->GetLabel() + "'. Labels must be "
//             "unique.");

//       SetSkeletonClearanceUtility(utility->GetLabel(), utility);
//     }
//     else if(child.Name() == "TopologicalMap") {
//       auto utility = new TopologicalMap<MPTraits>(child);

//       // A second node with the same label is an error during XML parsing.
//       if(m_topologicalMaps.count(utility->GetLabel()))
//         throw ParseException(child.Where(), "Second TopologicalMap "
//             "node with the label '" + utility->GetLabel() + "'. Labels must be "
//             "unique.");

//       SetTopologicalMap(utility->GetLabel(), utility);
//     }
//     else if(child.Name() == "TetGenDecomposition") {
//       // Parse the label and check that it is unique.
//       const std::string label = child.Read("label", true, "",
//           "The label for this decomposition.");

//       if(m_decompositions.count(label))
//         throw ParseException(child.Where(), "Second decomposition node "
//             "with the label " + label + ". Labels must be unique across all "
//             "types of decomposition.");

//       m_tetgens[label] = TetGenDecomposition(child);
//       SetDecomposition(label, nullptr);
//     }
//     else if(child.Name() == "SafeIntervalTool") {
//       auto utility = new SafeIntervalTool<MPTraits>(child);

//       // A second node with the same label is an error during XML parsing.
//       if(m_safeIntervalTools.count(utility->GetLabel()))
//         throw ParseException(child.Where(), "Second SafeIntervalTool "
//             "node with the label '" + utility->GetLabel() + "'. Labels must be "
//             "unique.");

//       SetSafeIntervalTool(utility->GetLabel(), utility);
//     }
//     else if(child.Name() == "LKHSearch") {
//       auto utility = new LKHSearch<MPTraits>(child);

//       // A second node with the same label is an error during XML parsing.
//       if(m_lkhSearchTools.count(utility->GetLabel()))
//         throw ParseException(child.Where(), "Second LKHSearch "
//             "node with the label '" + utility->GetLabel() + "'. Labels must be "
//             "unique.");

//       SetLKHSearch(utility->GetLabel(), utility);
//     }
//     else if(child.Name() == "TRPTool") {
//       auto utility = new TRPTool<MPTraits>(child);

//       // A second node with the same label is an error during XML parsing.
//       if(m_trpTools.count(utility->GetLabel()))
//         throw ParseException(child.Where(), "Second TRPTool "
//             "node with the label '" + utility->GetLabel() + "'. Labels must be "
//             "unique.");

//       SetTRPTool(utility->GetLabel(), utility);
//     }
//     // Below here we are setting defaults rather than creating instances.
//     else if(child.Name() == "ReebGraphConstruction") {
//       if(parsedReebGraph)
//         throw ParseException(child.Where(),
//             "Second ReebGraphConstruction node detected. This node sets "
//             "default parameters - only one is allowed.");
//       parsedReebGraph = true;

//       ReebGraphConstruction::SetDefaultParameters(child);
//     }
//     else if(child.Name() == "MeanCurvatureSkeleton3D") {
//       if(parsedMCS)
//         throw ParseException(child.Where(),
//             "Second meanCurvatureSkeleton3D node detected. This node sets "
//             "default parameters - only one is allowed.");
//       parsedMCS = true;

//       MeanCurvatureSkeleton3D::SetDefaultParameters(child);
//     }
// /*    else if(child.Name() == "SVMModel") {
//       if(parsedSVMModel)
//         throw ParseException(child.Where(), "Second SVMModel node detected. "
//             "This node sets default parameters - only one is allowed.");
//       parsedSVMModel = true;

//       SVMModel<MPTraits>::SetDefaultParameters(child);
//     }
// */
//     else if(child.Name() == "ReachabilityUtil") {
//       auto util = new ReachabilityUtil<MPTraits>(child);
//       SetReachabilityUtil(util->GetLabel(), util);
//     }
    
//     else if(child.Name() == "PointConstruction") {
//       auto util = new PointConstruction<MPTraits>(child);
//       SetPointConstruction(util->GetLabel(), util);
//     }
//   }
// }


// template <typename MPTraits>
// void
// MPToolsType<MPTraits>::
// Initialize() {
//   //Uninitialize();
//   for(auto& pair : m_clearanceUtils)
//     pair.second->Initialize();
//   for(auto& pair : m_medialAxisUtils)
//     pair.second->Initialize();
//   for(auto& pair : m_skeletonUtils)
//     pair.second->Initialize();
//   for(auto& pair : m_topologicalMaps)
//     pair.second->Initialize();
//   for(auto& pair : m_safeIntervalTools)
//     pair.second->Initialize();
//   for(auto& pair : m_lkhSearchTools)
//     pair.second->Initialize();
//   /// @todo Homogenize trp tool initialization.
//   //for(auto& pair : m_trpTools)
//   //  pair.second->Initialize();
//   for(auto& pair : m_reachabilityUtils)
//     pair.second->Initialize();
//   for(auto& pair : m_pointConstruction)
//     pair.second->Initialize();
// }

// template <typename MPTraits>
// void
// MPToolsType<MPTraits>::
// Uninitialize() {
//   for(auto& pair : m_decompositions){
//     delete pair.second;
//     pair.second = nullptr;
//   }
// }

// template <typename MPTraits>
// MPToolsType<MPTraits>::
// ~MPToolsType() {
//   for(auto& pair : m_clearanceUtils)
//     delete pair.second;
//   for(auto& pair : m_medialAxisUtils)
//     delete pair.second;
//   for(auto& pair : m_skeletonUtils)
//     delete pair.second;
//   for(auto& pair : m_topologicalMaps)
//     delete pair.second;
//   for(auto& pair : m_safeIntervalTools)
//     delete pair.second;
//   for(auto& pair : m_decompositions)
//     delete pair.second;
//   for(auto& pair : m_lkhSearchTools)
//     delete pair.second;
//   for(auto& pair : m_trpTools)
//     delete pair.second;
//   for(auto& pair : m_reachabilityUtils)
//     delete pair.second;
//   for(auto& pair : m_pointConstruction) 
//     delete pair.second;
// }

// /*--------------------------- Clearance Utility ------------------------------*/

// template <typename MPTraits>
// ClearanceUtility<MPTraits>*
// MPToolsType<MPTraits>::
// GetClearanceUtility(const std::string& _label) const {
//   return GetUtility(_label, m_clearanceUtils);
// }


// template <typename MPTraits>
// void
// MPToolsType<MPTraits>::
// SetClearanceUtility(const std::string& _label,
//     ClearanceUtility<MPTraits>* const _utility) {
//   SetUtility(_label, _utility, m_clearanceUtils);
// }


// /*-------------------------- Medial Axis Utility -----------------------------*/

// template <typename MPTraits>
// MedialAxisUtility<MPTraits>*
// MPToolsType<MPTraits>::
// GetMedialAxisUtility(const std::string& _label) const {
//   return GetUtility(_label, m_medialAxisUtils);
// }


// template <typename MPTraits>
// void
// MPToolsType<MPTraits>::
// SetMedialAxisUtility(const std::string& _label,
//     MedialAxisUtility<MPTraits>* const _utility) {
//   SetUtility(_label, _utility, m_medialAxisUtils);
// }

// /*----------------------------- Skeleton Tools -------------------------------*/

// template <typename MPTraits>
// SkeletonClearanceUtility<MPTraits>*
// MPToolsType<MPTraits>::
// GetSkeletonClearanceUtility(const std::string& _label) const {
//   return GetUtility(_label, m_skeletonUtils);
// }


// template <typename MPTraits>
// void
// MPToolsType<MPTraits>::
// SetSkeletonClearanceUtility(const std::string& _label,
//     SkeletonClearanceUtility<MPTraits>* const _utility) {
//   SetUtility(_label, _utility, m_skeletonUtils);
// }

// /*------------------------------ Topological Map -----------------------------*/

// template <typename MPTraits>
// TopologicalMap<MPTraits>*
// MPToolsType<MPTraits>::
// GetTopologicalMap(const std::string& _label) const {
//   return GetUtility(_label, m_topologicalMaps);
// }


// template <typename MPTraits>
// void
// MPToolsType<MPTraits>::
// SetTopologicalMap(const std::string& _label,
//     TopologicalMap<MPTraits>* const _utility) {
//   SetUtility(_label, _utility, m_topologicalMaps);
// }

// /*---------------------------- Safe Interval Tool ----------------------------*/

// template <typename MPTraits>
// SafeIntervalTool<MPTraits>*
// MPToolsType<MPTraits>::
// GetSafeIntervalTool(const std::string& _label) const {
//   return GetUtility(_label, m_safeIntervalTools);
// }


// template <typename MPTraits>
// void
// MPToolsType<MPTraits>::
// SetSafeIntervalTool(const std::string& _label,
//     SafeIntervalTool<MPTraits>* const _utility) {
//   SetUtility(_label, _utility, m_safeIntervalTools);
// }

// /*------------------------------- LKH Search ---------------------------------*/

// template <typename MPTraits>
// LKHSearch<MPTraits>*
// MPToolsType<MPTraits>::
// GetLKHSearch(const std::string& _label) const {
//   return GetUtility(_label, m_lkhSearchTools);
// }



// template <typename MPTraits>
// void
// MPToolsType<MPTraits>::
// SetLKHSearch(const std::string& _label,
//     LKHSearch<MPTraits>* const _utility) {
//   SetUtility(_label, _utility, m_lkhSearchTools);
// }

// /*-------------------------------- TRP Tool ----------------------------------*/

// template <typename MPTraits>
// TRPTool<MPTraits>*
// MPToolsType<MPTraits>::
// GetTRPTool(const std::string& _label) const {
//   return GetUtility(_label, m_trpTools);
// }



// template <typename MPTraits>
// void
// MPToolsType<MPTraits>::
// SetTRPTool(const std::string& _label,
//     TRPTool<MPTraits>* const _utility) {
//   SetUtility(_label, _utility, m_trpTools);
// }


// /*----------------------------- Decompositions -------------------------------*/

// template <typename MPTraits>
// const WorkspaceDecomposition*
// MPToolsType<MPTraits>::
// GetDecomposition(const std::string& _label) {
//   // Initialize the decomposition if not already.
//   typename decltype(m_decompositions)::iterator iter;
//   try {
//     iter = m_decompositions.find(_label);
//   }
//   catch(const std::out_of_range&) {
//     throw RunTimeException(WHERE) << "Requested decomposition '" << _label
//                                   << "' does not exist.";
//   }

//   if(iter->second == nullptr) {
//     MethodTimer mt(m_library->GetStatClass(), "TetGenDecomposition::" + _label);
//     iter->second = m_tetgens[_label](m_library->GetMPProblem()->GetEnvironment());
//   }
//   return iter->second;
// }


// template <typename MPTraits>
// void
// MPToolsType<MPTraits>::
// SetDecomposition(const std::string& _label,
//     const WorkspaceDecomposition* _decomposition) {
//   // If a decomposition was already assigned to this label, delete it before
//   // storing the new one.
//   auto iter = m_decompositions.find(_label);
//   const bool alreadyExists = iter != m_decompositions.end();

//   if(alreadyExists) {
//     delete iter->second;
//     iter->second = _decomposition;
//   }
//   else
//     m_decompositions[_label] = _decomposition;
// }

// /*----------------------------- Reachability Utils -------------------------------*/

// template <typename MPTraits>
// ReachabilityUtil<MPTraits>*
// MPToolsType<MPTraits>::
// GetReachabilityUtil(const std::string& _label) const {
//   return GetUtility(_label, m_reachabilityUtils);
// }


// template <typename MPTraits>
// void
// MPToolsType<MPTraits>::
// SetReachabilityUtil(const std::string& _label,
//     ReachabilityUtil<MPTraits>* _util) {
//   SetUtility(_label, _util, m_reachabilityUtils);
// }



// /*----------------------------- Point Construction -------------------------------*/

// template <typename MPTraits>
// PointConstruction<MPTraits>*
// MPToolsType<MPTraits>::
// GetPointConstruction(const std::string& _label) const {
//   return GetUtility(_label, m_pointConstruction);
// }


// template <typename MPTraits>
// void
// MPToolsType<MPTraits>::
// SetPointConstruction(const std::string& _label,
//     PointConstruction<MPTraits>* _util) {
//   SetUtility(_label, _util, m_pointConstruction);
// }


// /*---------------------------------- Helpers ---------------------------------*/

// template <typename MPTraits>
// template <template <typename> class Utility>
// inline
// Utility<MPTraits>*
// MPToolsType<MPTraits>::
// GetUtility(const std::string& _label, const LabelMap<Utility>& _map) const {
//   try {
//     return _map.at(_label);
//   }
//   catch(const std::out_of_range&) {
//     Utility<MPTraits> dummy;
//     throw RunTimeException(WHERE) << "Requested " << dummy.GetName()
//                                   << " '" << _label  << "' does not exist.";
//   }
//   catch(const std::exception& _e) {
//     Utility<MPTraits> dummy;
//     throw RunTimeException(WHERE) << "Error when fetching " << dummy.GetName()
//                                   << " '" << _label << "': " << _e.what();
//   }
//   catch(...) {
//     Utility<MPTraits> dummy;
//     throw RunTimeException(WHERE) << "Error when fetching " << dummy.GetName()
//                                   << " '" << _label << "': (unknown).";
//   }
// }


// template <typename MPTraits>
// template <template <typename> class Utility>
// void
// MPToolsType<MPTraits>::
// SetUtility(const std::string& _label, Utility<MPTraits>* _utility,
//     LabelMap<Utility>& _map) const {
//   // Set the library pointer.
//   _utility->SetMPLibrary(m_library);

//   // Check if this label is already in use.
//   auto iter = _map.find(_label);
//   const bool alreadyExists = iter != _map.end();

//   // If the label already exists, we need to release the previous utility first.
//   if(alreadyExists) {
//     delete iter->second;
//     iter->second = _utility;
//   }
//   else
//     _map.insert({_label, _utility});
// }

/*----------------------------------------------------------------------------*/

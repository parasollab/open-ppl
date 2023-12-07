#ifndef MP_TOOLS_H_
#define MP_TOOLS_H_

#include <string>
#include <unordered_map>

#include "Utilities/XMLNode.h"

// #include "MedialAxisUtilities.h"
// #include "MeanCurvatureSkeleton3D.h"
// #include "ReebGraphConstruction.h"
// #include "SafeIntervalTool.h"
// #include "SkeletonClearanceUtility.h"
// #include "TetGenDecomposition.h"
// #include "TopologicalMap.h"
// #include "TRPTool.h"
// #include "ReachabilityUtil.h"
// #include "MPLibrary/MPTools/LKHSearch.h"
// #include "PointConstruction.h"
//#include "MPLibrary/LearningModels/SVMModel.h"


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
class MPToolsType final {

  // ///@name Motion Planning Types
  // ///@{



  // ///@}
  // ///@name Local Types
  // ///@{

  // template <template <typename> class Utility>
  // using LabelMap = std::unordered_map<std::string, Utility<MPTraits>*>;

  // ///@}
  // ///@name Internal State
  // ///@{

  // MPLibrary* const m_library; ///< The owning library.

  // LabelMap<ClearanceUtility>         m_clearanceUtils;
  // LabelMap<MedialAxisUtility>        m_medialAxisUtils;
  // LabelMap<SkeletonClearanceUtility> m_skeletonUtils;
  // LabelMap<TopologicalMap>           m_topologicalMaps;
  // LabelMap<SafeIntervalTool>         m_safeIntervalTools;
  // LabelMap<LKHSearch>                m_lkhSearchTools;
  // LabelMap<TRPTool>                  m_trpTools;
  // LabelMap<ReachabilityUtil>         m_reachabilityUtils;
  // LabelMap<PointConstruction>        m_pointConstruction;

  // std::unordered_map<std::string, TetGenDecomposition> m_tetgens;
  // std::unordered_map<std::string, const WorkspaceDecomposition*> m_decompositions;

  // ///@}

  // public:

  //   ///@name Construction
  //   ///@{

  //   /// Construct a tool set.
  //   /// @param _library The owning library.
  //   MPToolsType(MPLibrary* const _library);

  //   /// Parse an XML node.
  //   /// @param _node The XML node object.
  //   void ParseXML(XMLNode& _node);

  //   /// Initialize the clearance and MA tools prior to use.
  //   void Initialize();

  //   /// Uninitialize the clearance and MA tools.
  //   void Uninitialize();

  //   ~MPToolsType();

  //   ///@}
  //   ///@name Clearance Utility
  //   ///@{

  //   /// Get a ClearanceUtility by label.
  //   /// @param _label The string label of the desired utility as defined in the
  //   ///               XML file.
  //   /// @return The labeled utility.
  //   ClearanceUtility<MPTraits>* GetClearanceUtility(const std::string& _label)
  //       const;

  //   /// Set a ClearanceUtility. This object will take ownership of the utility
  //   /// and delete it when necessary.
  //   /// @param _label The string label for the new utility.
  //   /// @param _utility The ClearanceUtility to use.
  //   void SetClearanceUtility(const std::string& _label,
  //       ClearanceUtility<MPTraits>* const _utility);

  //   ///@}
  //   ///@name Medial Axis Utility
  //   ///@{

  //   /// Get a MedialAxisUtility by label.
  //   /// @param _label The string label of the desired utility as defined in the
  //   ///               XML file.
  //   /// @return The labeled utility.
  //   MedialAxisUtility<MPTraits>* GetMedialAxisUtility(const std::string& _label)
  //       const;

  //   /// Set a MedialAxisUtility. This object will take ownership of the utility
  //   /// and delete it when necessary.
  //   /// @param _label The string label for the new utility.
  //   /// @param _utility The MedialAxisUtility to use.
  //   void SetMedialAxisUtility(const std::string& _label,
  //       MedialAxisUtility<MPTraits>* const _utility);

  //   ///@}
  //   ///@name Skeleton Clearance Utility
  //   ///@{

  //   /// Get a SkeletonClearanceUtility by label.
  //   /// @param _label The string label of the desired utility as defined in the
  //   ///               XML file.
  //   /// @return The labeled utility.
  //   SkeletonClearanceUtility<MPTraits>* GetSkeletonClearanceUtility(
  //       const std::string& _label) const;

  //   /// Set a SkeletonClearanceUtility. This object will take ownership of the
  //   /// utility and delete it when necessary.
  //   /// @param _label The string label for the new utility.
  //   /// @param _utility The SkeletonClearanceUtility to use.
  //   void SetSkeletonClearanceUtility(const std::string& _label,
  //       SkeletonClearanceUtility<MPTraits>* const _utility);

  //   ///@}
  //   ///@name Topological Map
  //   ///@{

  //   /// Get a TopologicalMap by label.
  //   /// @param _label The string label of the desired utility as defined in the
  //   ///               XML file.
  //   /// @return The labeled utility.
  //   TopologicalMap<MPTraits>* GetTopologicalMap(const std::string& _label) const;

  //   /// Set a TopologicalMap. This object will take ownership of the utility and
  //   /// delete it when necessary.
  //   /// @param _label The string label for the new utility.
  //   /// @param _utility The TopologicalMap to use.
  //   void SetTopologicalMap(const std::string& _label,
  //       TopologicalMap<MPTraits>* const _utility);

  //   ///@}
  //   ///@name Safe Interval Tool
  //   ///@{

  //   /// Get a SafeIntervalTool by label.
  //   /// @param _label The string label of the desired utility as defined in the
  //   ///               XML file.
  //   /// @return The labeled utility.
  //   SafeIntervalTool<MPTraits>* GetSafeIntervalTool(const std::string& _label)
  //       const;

  //   /// Set a SafeIntervalTool. This object will take ownership of the utility and
  //   /// delete it when necessary.
  //   /// @param _label The string label for the new utility.
  //   /// @param _utility The TopologicalMap to use.
  //   void SetSafeIntervalTool(const std::string& _label,
  //       SafeIntervalTool<MPTraits>* const _utility);

  //   ///@}
  //   ///@name LKH Search
  //   ///@{

  //   /// Get an LKH Search by label.
  //   /// @param _label The string label of the desired utility as defined in the
  //   ///               XML file.
  //   /// @return The labeled utility.
  //   LKHSearch<MPTraits>* GetLKHSearch(const std::string& _label) const;

  //   /// Set an LKH Search
  //   /// @param _label The string label for the new utility
  //   /// @param _utility The LKHSearch to use
  //   void SetLKHSearch(const std::string& _label,
  //       LKHSearch<MPTraits>* const _utility);

  //   ///@}
  //   ////@name TRP Tool
  //   ///@{

  //   /// Get a TRP Tool by label.
  //   /// @param _label The string label of the desired utility as defined in the
  //   ///               XML file.
  //   /// @return The labeled utility.
  //   TRPTool<MPTraits>* GetTRPTool(const std::string& _label) const;

  //   /// Set a TRP Tool
  //   /// @param _label The string label for the new utility
  //   /// @param _utility The LKHSearch to use
  //   void SetTRPTool(const std::string& _label,
  //       TRPTool<MPTraits>* const _utility);

  //   ///@}
  //   ///@name Decompositions
  //   ///@{

  //   /// Get a decomposition.
  //   /// @param _label The label of the decomposition to use.
  //   const WorkspaceDecomposition* GetDecomposition(const std::string& _label);

  //   /// Set a workspace decomposition by label. This object will take ownership
  //   /// the decomposition and delete it when necessary.
  //   /// @param _label The label for this decomposition.
  //   /// @param _decomposition The decomposition object to set.
  //   void SetDecomposition(const std::string& _label,
  //       const WorkspaceDecomposition* _decomposition);

  //   ///@}
  //   ///@name Reachability
  //   ///@{

  //   /// Get a Reachability Utility
  //   /// @param _label The label of the decomposition to use.
  //   ReachabilityUtil<MPTraits>* GetReachabilityUtil(const std::string& _label) const;

  //   /// Set a reachability utility  by label.
  //   /// @param _label The label for this utility
  //   /// @param _decomposition the reachability utility
  //   void SetReachabilityUtil(const std::string& _label,
  //       ReachabilityUtil<MPTraits>* _util);

  //   ///}

  //   ///@{

  //   /// Get a Point Construction
  //   /// @param _label The label of the decomposition to use.
  //   PointConstruction<MPTraits>* GetPointConstruction(const std::string& _label) const;

  //   /// Set a Point Construction  by label.
  //   /// @param _label The label for this utility
  //   /// @param _decomposition the point construction
  //   void SetPointConstruction(const std::string& _label,
  //       PointConstruction<MPTraits>* _util);

  //   ///}

  // private:

  //   ///@name Helpers
  //   ///@{

  //   /// Get a utility in a label map. Throws if not found.
  //   /// @param _label The utility label.
  //   /// @param _map The label map which holds _utility.
  //   /// @return The named utility.
  //   template <template <typename> class Utility>
  //   Utility<MPTraits>* GetUtility(const std::string& _label,
  //       const LabelMap<Utility>& _map) const;


  //   /// Set a utility in a label map.
  //   /// @param _label The utility label.
  //   /// @param _utility The utility to set.
  //   /// @param _map The label map which holds _utility.
  //   template <template <typename> class Utility>
  //   void SetUtility(const std::string& _label, Utility<MPTraits>* _utility,
  //       LabelMap<Utility>& _map) const;

  //   ///@}

};

#endif

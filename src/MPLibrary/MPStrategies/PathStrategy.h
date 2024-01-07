#ifndef PATH_STRATEGY_H_
#define PATH_STRATEGY_H_

#include "MPStrategyMethod.h"

#include "Utilities/XMLNode.h"

////////////////////////////////////////////////////////////////////////////////
/// \brief   PathStrategy generates configurations along user-generated paths.
/// \details PathStrategy collects a set of workspace points along the
///          user-generated paths and generates configurations at those points.
///          Valid points are added to the map and connected in path order.
///          Invalid points are taken as the center of a sampling radius for a
///          valid configuration. If a connection between two path
///          configurations \c c1 and \c c2 fails, an intermediate configuration
///          cI is generated at their midpoint if it is valid.  The path
///          connection then recurses on the set (c1, cI, c2).
////////////////////////////////////////////////////////////////////////////////
class PathStrategy : public MPStrategyMethod {
 public:
  ///\name Motion Planning Types
  ///@{

  typedef typename MPBaseObject::RoadmapType RoadmapType;
  typedef typename RoadmapType::VID VID;
  typedef typename RoadmapType::VertexSet VertexSet;

  ///@}
  ///\name Construction
  ///@{

  PathStrategy();
  PathStrategy(XMLNode& _node);

  ///@}
  ///\name MPBaseObject Overrides
  ///@{

  virtual void Print(ostream& _os) const override;

  ///@}
  ///\name MPStrategyMethod Overrides
  ///@{

  virtual void Initialize() override;
  virtual void Run() override;
  virtual void Finalize() override;

  ///@}

 protected:
  ///\name Helpers
  ///@{

  ////////////////////////////////////////////////////////////////////////////
  /// \brief Read the users paths from file.
  void getUserPaths();

  ////////////////////////////////////////////////////////////////////////////
  /// \brief Check the configuration's validity.
  /// \param[in/out] _cfg The configuration to validate.
  /// \return A bool indicating whether the configuration was successfully
  ///         validated.
  bool ValidateCfg(Cfg& _cfg);

  ////////////////////////////////////////////////////////////////////////////
  /// \brief Samples a node near the invalid node trying to connect it to the
  ///        previous and, if valid, the next node.
  /// \param[in] _invalidNode Invalid node.
  /// \param[in] _nextNode Subsequent node.
  /// \param[in] _lastValidNode Previous valid node of the path.
  /// \return A configuration sampled near the invalid one.
  Cfg RandomNearNode(Cfg& _invalidNode, Cfg& _nextNode, Cfg& _lastValidNode);

  ////////////////////////////////////////////////////////////////////////////
  /// \brief Validate and add configurations to the roadmap. Store their VID's
  ///        for connection.
  /// \param[in] _samples Configurations to add.
  /// \param[out] _vids   The corresponding VID's in the roadmap.
  void AddToRoadmap(vector<Point3d>& _points, vector<VID>& _vids);

  ////////////////////////////////////////////////////////////////////////////
  /// \brief Try to connect new nodes to the roadmap in the path sequence. If
  ///        a connection attempt fails, attempt to create intermediates.
  /// \param[in] _vids The VID's of the path nodes, in path order.
  void ConnectPath(const vector<VID>& _vids);

  ////////////////////////////////////////////////////////////////////////////
  /// \brief Tries to connect the two connected components represented by the
  ///        given nodes.
  /// \param[in] _vid1 Representative of the first connected component.
  /// \param[in] _vid2 Representative of the first connected component.
  /// \return A bool indicating whether the two CCs were successfully connected.
  bool ConnectCC(VID _vid1, VID _vid2);

  ////////////////////////////////////////////////////////////////////////////
  /// \brief Attempts to connect all the connected components.
  void ConnectCCs();

  ///@}

 private:
  ///\name PMPL Object Labels
  ///@{

  string m_vcLabel{"pqp_solid"};  ///< The ValidityChecker label.
  string m_dmLabel{"euclidean"};  ///< The DistanceMetric label.
  string m_lpLabel{"sl"};         ///< The LocalPlanner label.
  string m_ncLabel{"Closest"};    ///< The Connector label.
  string m_nfLabel{"BFNF"};       ///< The NeighbothoodFinder label.

  ///@}
  ///\name Internal State
  ///@{

  vector<vector<Point3d>> m_userPaths;  ///< User paths.
  vector<VID> m_endPoints;              ///< The set of all path heads/tails.
  string m_UserPathsFilename;           ///< User paths file.
  double m_radius;                      ///< Radius os sampling.

  ///@}
};

#endif

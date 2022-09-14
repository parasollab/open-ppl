#ifndef PPL_PATH_GUIDED_PATH_H_
#define PPL_PATH_GUIDED_PATH_H_

#include "BasicRRTStrategy.h"

template <typename MPTraits>
class PathGuidedRRT : public BasicRRTStrategy<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;

    ///@}
    ///@name Path Guiding Types
    ///@{

    /// Guiding path struct to hold path waypoints, bias probability towards waypoints,
    /// and the distance to a waypoint to be considered reached.
    struct GuidingPath {
      std::vector<CfgType> waypoints;
      size_t currentIndex{0};
    };

    ///@}
    ///@name Construction
    ///@{

    PathGuidedRRT();

    PathGuidedRRT(XMLNode& _node);

    ~PathGuidedRRT();

    ///@}

  protected:

    ///@name MPStrategy Overrides
    ///@{

    virtual void Initialize();

    ///@}
    ///@name RRT Overrides
    ///@{

    /// Override SelectTarget to bias towards next waypoint with some probability.
    virtual CfgType SelectTarget() override;

    /// Override ExpandTree to check if the next waypoint has been reached.
    virtual VID ExpandTree(const CfgType& _target) override;

    ///@}
    ///@name Path Guidance Functions
    ///@{

    /// Check if a vertex is within the distance threshold of the current path waypoint.
    bool ReachedWaypoint(VID _vid);

    ///@}
    ///@name Internal State

    double m_bias;

    double m_threshold;

    std::vector<std::string> m_filenames; ///< Set input path filenames.

    std::string m_bbxString;

    std::vector<GuidingPath> m_guidingPaths; ///< Set of paths to guide sampling.

    std::unique_ptr<CSpaceBoundingBox> m_boundary; ///< Relative boundary to sample from.

    size_t m_guidingPathIndex{0}; ///< Index of current guiding path.

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
PathGuidedRRT<MPTraits>::
PathGuidedRRT() {
  this->SetName("PathGuidedRRT");
}

template <typename MPTraits>
PathGuidedRRT<MPTraits>::
PathGuidedRRT(XMLNode& _node) : BasicRRTStrategy<MPTraits>(_node) {
  this->SetName("PathGuidedRRT");

  m_bias  = _node.Read("bias",false,.5,0.,1.,
            "Probability of biasing towards waypoints.");
  m_threshold = _node.Read("distanceThreshold",false,1.,0.,MAX_DBL,
                "Distance threshold to 'reach' a waypoint.");
  std::string filenames = _node.Read("filenames",true,"","Path file names.");

  m_bbxString = _node.Read("boundary",true,"",
                          "Relative CSpace boundary to sample around waypoints.");

  stringstream ss(filenames);
  do {
    std::string filename;
    ss >> filename;
    if(filename == "")
      break;
    m_filenames.push_back(filename);
  } while(ss);
}

template <typename MPTraits>
PathGuidedRRT<MPTraits>::
~PathGuidedRRT() { }

/*---------------------------- MPStrategy Overrides --------------------------*/

template <typename MPTraits>
void
PathGuidedRRT<MPTraits>::
Initialize() {
 
  BasicRRTStrategy<MPTraits>::Initialize();
 
  // Initialize bounding box  
  auto robot = this->GetTask()->GetRobot();
  m_boundary = std::unique_ptr<CSpaceBoundingBox>(new CSpaceBoundingBox(
    robot->GetMultiBody()->DOF() * (robot->IsNonholonomic() ? 2 : 1))
  );

  auto bbx = m_boundary.get();
  std::istringstream bbxStream(m_bbxString);
  bbxStream >> *bbx;

  // Parse all of the guiding path filenames
  for(auto filename : m_filenames) {

    GuidingPath path;

    std::ifstream ifs(filename);
    if(!ifs)
      throw RunTimeException(WHERE) << "Cannot open file '" << filename << "'.";

    std::string tag;

    std::getline(ifs,tag); 
    std::getline(ifs,tag); 
    std::getline(ifs,tag); 


    std::string waypoint;

    while(std::getline(ifs,waypoint)) {
      std::stringstream ss(waypoint);
      Cfg cfg(robot);
      cfg.Read(ss);

      path.waypoints.push_back(cfg);
    }

    m_guidingPaths.push_back(path);
  }
}

/*------------------------------- RRT Overrides ------------------------------*/

template <typename MPTraits>
typename PathGuidedRRT<MPTraits>::CfgType
PathGuidedRRT<MPTraits>::
SelectTarget() {

  // With probability 1 - m_bias, take random sample
  if(DRand() > m_bias) {
    return BasicRRTStrategy<MPTraits>::SelectTarget();
  }

  // Otherwise, sample from boundary around next waypoint
  auto path = m_guidingPaths[m_guidingPathIndex];
  auto waypoint = path.waypoints[path.currentIndex];

  auto robot = this->GetTask()->GetRobot();
  CfgType relativeCfg(robot);
  relativeCfg.GetRandomCfg(m_boundary.get());

  return waypoint + relativeCfg;
}

template <typename MPTraits>
typename PathGuidedRRT<MPTraits>::VID
PathGuidedRRT<MPTraits>::
ExpandTree(const CfgType& _target) {

  auto vid = BasicRRTStrategy<MPTraits>::ExpandTree(_target);

  if(vid == MAX_INT or vid == INVALID_VID)
    return vid;

  // Check if new vertex reaches waypoint
  auto dm = this->GetDistanceMetric(this->m_goalDmLabel);

  auto& path = m_guidingPaths[m_guidingPathIndex];
  auto waypoint = path.waypoints[path.currentIndex]; 
  auto cfg = this->GetRoadmap()->GetVertex(vid);

  // If so, iterate towards next waypoint
  if(dm->Distance(waypoint,cfg) < m_threshold) {
    path.currentIndex++;
  }
 
  return vid;
}

/*-------------------------- Path Guidance Functions -------------------------*/

template <typename MPTraits>
bool
PathGuidedRRT<MPTraits>::
ReachedWaypoint(VID _vid) {
  return false;
}

/*----------------------------------------------------------------------------*/


#endif

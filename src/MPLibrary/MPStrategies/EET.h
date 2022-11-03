#ifndef PMPL_EET_H_
#define PMPL_EET_H_

// #include "MPStrategyMethod.h"
// #include "MPProblem/Constraints/Constraint.h"
// #include "Utilities/XMLNode.h"
// #include "ConfigurationSpace/GenericStateGraph.h"

// #include <iomanip>
// #include <iterator>
#include <string>
// #include <unordered_set>
// #include <vector>
#include <queue>

template <typename MPTraits>
class EET : public BasicRRTStrategy<MPTraits> {
  public:
    typedef GenericStateGraph<mathtool::Point3d, 
                                std::vector<mathtool::Point3d>> BaseType;

    typedef typename mathtool::Point3d      Point;
    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::WeightType   WeightType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;
    typedef typename RoadmapType::VertexSet VertexSet;


    // Constructors
    EET();
    EET(XMLNode& _node);
    // virtual ~EET() = default;

  protected:
    // Standard lifecycle functions
    virtual void Initialize() override;
    virtual void Iterate() override;


    // WAVEFRONT expansion functions
    void Wavefront();
    double DistanceToObstacles(Point _p);
    double Distance(Point _p1, Point _p2);

    
    // double explore_exploit_bias{0};  
    int m_nSphereSamples{100};

    // Start and goal points (not cfgs)
    Point pStart;
    std::vector<Point> pGoals;

    std::string m_samplerLabel{BasicRRTStrategy<MPTraits>::m_samplerLabel};

    struct Sphere{
      Point center;
      double radius;
      double priority;
      VID wavefrontVID;
      VID parentVID;

      Sphere(const Point _center, double _radius, double _priority, VID _parent)
      : center(_center), radius(_radius), priority(_priority), parentVID(_parent) {}

      Sphere() {}


      bool operator==(const Sphere& _other) const {
        return center == _other.center && radius == _other.radius;
      }

      bool operator!=(const Sphere& _other) const {
        return !(this == _other);
      }

      friend std::ostream& operator<<(std::ostream& _os, const Sphere& _vertex) {
        return _os;
      }

      bool operator < (const Sphere& rhs) const {
        return priority < rhs.priority;
      }
    };

    // Tree for actual WAVEFRONT expansion. 
    GenericStateGraph<Sphere, std::vector<Sphere>> wavefrontExpansion;
    // Keep track of the spheres and their VIDs. 
    std::unordered_map<VID, Sphere> wavefrontExpansionSpheres;
    // Root of the Wavefront. 
    Sphere wavefrontRoot;

  private:

    // For WAVEFRONT expansion.
    void InsertSphereIntoPQ(Point pCenter, std::priority_queue<Sphere>& q);

    
};

/*---------------------------------------------------------------------------*/
template <typename MPTraits>
EET<MPTraits>::
EET() {
  this->SetName("EET");
}

template <typename MPTraits>
EET<MPTraits>::
EET(XMLNode& _node) : BasicRRTStrategy<MPTraits>(_node) {
  this->SetName("EET");

  m_nSphereSamples = _node.Read("nSphereSamples", false, m_nSphereSamples,
      "Number of samples on the sphere, for Wavefront Expansion.");
}


/*---------------------------------------------------------------------------*/
template <typename MPTraits>
void
EET<MPTraits>::
Initialize() {
  // Override some basics
  this->m_growGoals = false;

  // Set start, goals, etc
  BasicRRTStrategy<MPTraits>::Initialize();

  //compute sphere tree
  Wavefront();

  wavefrontExpansion.Write("wavefront", this->GetMPProblem()->GetEnvironment());
  // take first sphere
  // Sphere s = 

}


template <typename MPTraits>
void
EET<MPTraits>::
Iterate() {
  // Sample new workspace frame

  // select nearest vertex in tree
}

/*----------------------------------------------------------------------------*/
/*---WAVEFRONT---*/


template <typename MPTraits>
void
EET<MPTraits>::
Wavefront() {
  auto rdmp = this->GetRoadmap();

  // Get start point. 
  const VID startVID = this->GenerateStart(m_samplerLabel);
  CfgType& startCfg = rdmp->GetVertex(startVID);
  pStart = startCfg.GetPoint();

  // Get goal points. 
  std::vector<VID> goals = this->GenerateGoals(m_samplerLabel);
  for (VID goal : goals){
    Point pGoal = rdmp->GetVertex(goal).GetPoint();
    pGoals.push_back(pGoal);
  }

  // Root the WE. 
  std::priority_queue<Sphere> sphereQueue;
  InsertSphereIntoPQ(pStart, sphereQueue);
  wavefrontRoot = sphereQueue.top();

  // Grow WE! 
  while (!sphereQueue.empty()) {
    Sphere s = sphereQueue.top();
    sphereQueue.pop();

    // add sphere to WE
    VID sVID = wavefrontExpansion.AddVertex(s);
    s.wavefrontVID = sVID;
    wavefrontExpansionSpheres[sVID] = s;

    // add edge to WE
    VID parentVID = s.parentVID;
    wavefrontExpansion.AddEdge(parentVID, sVID);

    // If we are within a goal region, we do not need to pursue this sphere.
    bool withinGoalRegion = false;
    for (Point pGoal : pGoals) {
      if (Distance(pGoal, s.center) < s.radius) 
        withinGoalRegion = true;
    }
    if (withinGoalRegion)
      continue;

    // Sample n points on sphere surface
    auto sampler = this->GetSampler("UniformRandomFree");
    std::vector<CfgType> samples;
    sampler->Sample(1, m_nSphereSamples, this->GetEnvironment()->GetBoundary(),
        std::back_inserter(samples));

    // Calculate sphere diameter for all points
    for (auto sample : samples) {
      Point pi = sample.GetPoint();

      // spheres in tree. 
      for (auto sphereVID : wavefrontExpansion.GetAllVIDs()) {
        Sphere sphere = wavefrontExpansionSpheres[sphereVID];

        // outside existing spheres
        if (Distance(pi, sphere.center) < sphere.radius) {
          InsertSphereIntoPQ(pi, sphereQueue);
        }
      }
    } // sphere diameter for all pts

  } // tree growth while loop

  return;
}

template<class MPTraits>
double 
EET<MPTraits>::
Distance(Point _p1, Point _p2) {
  auto pointRobot = this->GetMPProblem()->GetRobot("point");
  auto dmLabel = BasicRRTStrategy<MPTraits>::m_goalDmLabel;
  auto dm = this->GetDistanceMetric(dmLabel);

  CfgType cfg1(_p1, pointRobot);
  CfgType cfg2(_p2, pointRobot);

  return dm->Distance(cfg1, cfg2);
}

template<class MPTraits>
double
EET<MPTraits>::
DistanceToObstacles(Point _p){ 
  // This whole function shamelessly copied from Workspace/ClearanceMap.h
  
  // auto boundary = this->GetEnvironment()->GetBoundary();
  auto vc = this->GetValidityChecker("pqp_solid");
  auto pointRobot = this->GetMPProblem()->GetRobot("point");

  CfgType cfg(_p, pointRobot);
  CDInfo cdInfo;
  cdInfo.ResetVars();
  cdInfo.m_retAllInfo = true;
  vc->IsValid(cfg, cdInfo, "Obtain clearance map");

  // Check against boundary
  // const double boundaryClearance = boundary->GetClearance(_p);
  // if(boundaryClearance < cdInfo.m_minDist){
  //   cdInfo.m_objectPoint = boundary->GetClearancePoint(_p);
  //   cdInfo.m_minDist = boundaryClearance;
  // }
  return cdInfo.m_minDist;
}

template <class MPTraits>
void 
EET<MPTraits>::
InsertSphereIntoPQ(Point pCenter, std::priority_queue<Sphere>& q) {

  double radius = DistanceToObstacles(pCenter); // radius. 

  // Calculate the priority for this point
  // If multiple goals, use the min distance i guess
  // We're probably never going to run this with multiple goals anyway

  double min_priority = 1000000; //big number
  for (Point pGoal : pGoals) {
    // calculate the difference between p_goal and p_other_center
    double distance = Distance(pGoal, pCenter);

    double priority_value = distance - radius;

    if (priority_value < min_priority) {
      min_priority = priority_value;
    }
  }

  // use "-1" as default parent VID. 
  Sphere s(pCenter, radius, min_priority, -1);

  q.push(s);
}


/*----------------------------------------------------------------------------*/

#endif

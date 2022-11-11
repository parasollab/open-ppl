#ifndef PMPL_EET_H_
#define PMPL_EET_H_

// #include "MPStrategyMethod.h"
// #include "MPProblem/Constraints/Constraint.h"
// #include "Utilities/XMLNode.h"
// #include "ConfigurationSpace/GenericStateGraph.h"

// #include <iomanip>
// #include <iterator>
#include <string>
#include <set>
// #include <vector>
#include <queue>


/* HYPERPARAMETER NOTES THAT ANANYA NEEDS TO MOVE BUT IS
TOO HAPPY TO AT THE MOMENT

1. THE MINIMUM RADIUS IS HIGHLY DEPENDENT ON THE CLEARANCE OF THE AREA. LOWER CLEARANCE, YOU NEED TO STEP DOWN THE MIN RADIUS BY AN ORDER OF MAGNITUDE OR SO. 
2. THE MINIMUM RADIUS IS HIGHLY FINNICKY. IN ONE TEST, MIN RADIUS 0.1 WAS TOO BIG, 0.01 TOOK FOREVER (TOO SMALL) AND 0.05 WAS JUST RIGHT
3. WHEN CHANGING MIN RADIUS YOU ALSO NEED TO CHANGE NUM SAMPLES TAKEN ON SPHERE. RADIUS INCREASES, NUM POINTS SHOULD INCREASE (I THINK)

*/

template <typename MPTraits>
class EET : public BasicRRTStrategy<MPTraits> {
  public:
    typedef typename mathtool::Point3d      Point;
    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::WeightType   WeightType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;
    typedef typename RoadmapType::EID       EID;
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
    double m_minSphereRadius{0.001};

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

      Sphere(const Point _center, double _radius, double _priority, VID _parentVID)
      : center(_center), radius(_radius), priority(_priority), parentVID(_parentVID) {
        // Priority=distance to goal. 
        // So we want the lower numbers to be processed first. 
        priority *= -1; 
      }

      Sphere() {
        radius = 0;
        priority = 0;
        wavefrontVID = INVALID_VID;
        parentVID = INVALID_VID;
        center = Point();
      }

      bool isEmpty() {
        return radius == 0 
            && priority == 0
            && wavefrontVID == INVALID_VID
            && parentVID == INVALID_VID;
      }

      bool operator==(const Sphere& _other) const {
        return center == _other.center 
            && radius == _other.radius 
            && priority == _other.priority;
      }

      bool operator!=(const Sphere& _other) const {
        return !(this == _other);
      }

      friend std::ostream& operator<<(std::ostream& _os, const Sphere& _vertex) {
        _os << _vertex.center[0] << " " 
            << _vertex.center[1] << " " 
            << _vertex.center[2];
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
    Sphere InsertSphereIntoPQ(Point pCenter, std::priority_queue<Sphere>& q, VID parentVID);
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
                                0, std::numeric_limits<int>::max(),
      "Number of samples on the sphere, for Wavefront Expansion.");
  m_minSphereRadius = _node.Read("minSphereRadius", false, m_minSphereRadius, 
                                0., std::numeric_limits<double>::max(),
      "Minimum sphere radius for sphere growth.");
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
  InsertSphereIntoPQ(pStart, sphereQueue, INVALID_VID);
  wavefrontRoot = sphereQueue.top();

  // in-loop variables
  std::set<int> goalsReached;

  // Grow WE! 
  while (!sphereQueue.empty()) {
    // if we've reached all the goals, we are done. 
    if (goalsReached.size() == pGoals.size()) {
      break;
    }


    Sphere s = sphereQueue.top();
    sphereQueue.pop();

    // add sphere to WE
    VID sVID = wavefrontExpansion.AddVertex(s);
    s.wavefrontVID = sVID;
    wavefrontExpansionSpheres[sVID] = s;
    if (this->m_debug) {
      std::cout << "Investigating " << 
                    "Sphere " << s.center[0] << " " 
                              << s.center[1] << " " 
                              << s.center[2] << " " 
                        << "with radius " << s.radius
                        << " priority: " << s.priority 
                << "Added to WE." << std::endl 
                << "   (There are " << sphereQueue.size() << " elments in the queue)."
                << std::endl;
    }

    // add edge to WE
    VID parentVID = s.parentVID;
    wavefrontExpansion.AddEdge(parentVID, sVID);
    if (this->m_debug) {
      std::cout << "   Adding edge from VID " << parentVID 
                << " to VID " << sVID << std::endl;
      std::cout << "   VID " << parentVID << " has children VIDs: ";
      if (parentVID != INVALID_VID) {
        for (auto v : wavefrontExpansion.GetChildren(parentVID)){
          std::cout << " " << v;
        }
        std::cout << " " << std::endl;
      } else {std::cout << "parentVID invalid. This is likely the root node. " << std::endl;}
    }

    // If we are within the goal region, we are done.
    // TODO ANANYA: CHECK ALL GOAL REGIONS.
    bool inGoalRegion = false;
    for (size_t i=0; i < pGoals.size(); i++) { 
      auto pGoal = pGoals[i];
      if (Distance(pGoal, s.center) < s.radius) {
        inGoalRegion = true;
        goalsReached.insert(i);
        if (this->m_debug) {
          std::cout << "   This sphere contains the goal region." << std::endl;
        }
        break;
      }
    }
    if (inGoalRegion) {
      continue;
    }

    // Sample n points on sphere surface. 
    PointConstruction<MPTraits> _p;
    std::vector<Point> samples;
    samples = _p.SampleSphereSurface(s.center, s.radius, m_nSphereSamples);
    if (this->m_debug){
      std::cout << "   sampled " << samples.size() 
                << " samples on the surface of sphere with VID " << sVID 
                << std::endl;
    }

    // Calculate sphere diameter for all points
    for (Point p_i : samples) {
      // check which sphere in tree to attach this one to. 

      // not in pseudocode, but don't want to insert 
      // the same sphere into queue so many times. Priority will be the same. 
      // TODO: is sphereAddedToQueue bad? making path longer?
      bool sphereAddedToQueue = false; 
      for (auto sphereVID : wavefrontExpansion.GetAllVIDs()) {
        Sphere sphere = wavefrontExpansionSpheres[sphereVID];

        // outside existing spheres
        if (Distance(p_i, sphere.center) < sphere.radius) {
          InsertSphereIntoPQ(p_i, sphereQueue, sphereVID);
          sphereAddedToQueue = true;
        }

        if (sphereAddedToQueue) { break; }
      }
    } // sphere diameter for all pts

  } // tree growth while loop

  if (this->m_debug) {
    std::cout << "Wavefront expansion has "
              << wavefrontExpansion.GetAllVIDs().size() 
              << " nodes." << std::endl;
  }

  return;
}


//TODO ANANYA: FIGURE OUT WHY THIS SOMETIMES RETURNS A NEGATIVE NUMBER. 
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
  // This whole function is shamelessly copied from Workspace/ClearceMap.h
  // (And slightly modified, but not much.)
  
  auto boundary = this->GetEnvironment()->GetBoundary();
  auto vc = this->GetValidityChecker("pqp_solid");
  auto pointRobot = this->GetMPProblem()->GetRobot("point");

  CfgType cfg(_p, pointRobot);
  CDInfo cdInfo;
  cdInfo.ResetVars();
  cdInfo.m_retAllInfo = true;
  vc->IsValid(cfg, cdInfo, "Obtain clearance map");

  // Check against boundary, 
  // because we don't want any sphere growing action outside the boundary. 
  const double boundaryClearance = boundary->GetClearance(_p);
  if(boundaryClearance < cdInfo.m_minDist){
    cdInfo.m_objectPoint = boundary->GetClearancePoint(_p);
    cdInfo.m_minDist = boundaryClearance;
  }
  return cdInfo.m_minDist;
}

template <class MPTraits>
typename EET<MPTraits>::Sphere
EET<MPTraits>::
InsertSphereIntoPQ(Point pCenter, std::priority_queue<Sphere>& q, VID parentVID) {

  double radius = DistanceToObstacles(pCenter); // radius. 
  
  // Throw out spheres that are too small.. 
  if (radius < m_minSphereRadius) {
    return Sphere();
  }

  // Calculate the priority for this point
  // If multiple goals, use the min distance i guess
  // We're probably never going to run this with multiple goals anyway

  double min_priority = std::numeric_limits<int>::max();
  for (Point pGoal : pGoals) {
    // calculate the difference between p_goal and p_other_center
    double distance = Distance(pGoal, pCenter);

    double priority_value = distance - radius;

    if (priority_value < min_priority) {
      min_priority = priority_value;
    }
  }

  Sphere s(pCenter, radius, min_priority, parentVID);
  if (this->m_debug) {
    std::cout << "Added to queue " << 
              "Sphere " << s.center[0] << " " 
                        << s.center[1] << " " 
                        << s.center[2] << " " 
                  << "with radius " << s.radius
              << " and priority " << s.priority
               << std::endl;
  }

  q.push(s);

  return s;
}

/*----------------------------------------------------------------------------*/
#endif

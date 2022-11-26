#ifndef PMPL_EET_H_
#define PMPL_EET_H_

#include "Geometry/Boundaries/CSpaceBoundingSphere.h"
#include "MPLibrary/Samplers/GaussianSampler.h"

#include <string>
#include <set>
#include <queue>


/* HYPERPARAMETER NOTES THAT ANANYA NEEDS TO MOVE 

1. THE MINIMUM RADIUS IS HIGHLY DEPENDENT ON THE CLEARANCE OF
 THE AREA. LOWER CLEARANCE, YOU NEED TO STEP DOWN THE MIN RADIUS
  BY AN ORDER OF MAGNITUDE OR SO. 
2. THE MINIMUM RADIUS IS HIGHLY FINNICKY. IN ONE TEST, MIN RADIUS
 0.1 WAS TOO BIG, 0.01 TOOK FOREVER (TOO SMALL) AND 0.05 WAS JUST RIGHT
3. WHEN CHANGING MIN RADIUS YOU ALSO NEED TO CHANGE NUM SAMPLES 
TAKEN ON SPHERE. RADIUS INCREASES, NUM POINTS SHOULD INCREASE (I THINK)

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

    // overrides of BasicRRT functions
    virtual CfgType SelectTarget() override;    
    virtual VID ExpandTree(const VID _nearestVID, const CfgType& _target) override;

    
    // Start and goal points (not cfgs)
    Point pGoal;
    VID startVID;
  
    // Algorithm parameters (taken from XML or hard-defaulted here). 
    int m_nSphereSamples{20};
    double m_minSphereRadius{0.001};
    double m_defaultExploreExploit{1./3.}; // "gamma". Default value provided in paper.
    double m_wavefrontExplorationBias{0.1};
    double m_controlManipulation{0.01}; // "alpha". Default value provided in paper.
    double m_pGoalState{0.5}; // "rho". Default value provided in paper. 

    // Saved labels from XML.
    std::string m_samplerLabel{BasicRRTStrategy<MPTraits>::m_samplerLabel};
    std::string m_gaussianSamplerLabel;

    // Data structure Sphere, for wavefront expansion and guidance. 
    struct Sphere{
      Point center;
      double radius;
      double priority;
      VID wavefrontVID;
      VID parentVID;

      Sphere(const Point _center, double _radius, double _priority, VID _parentVID)
      : center(_center), radius(_radius), priority(_priority), parentVID(_parentVID) {
        // Priority=distance to goal(ish). Always > 0.  
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

      // Is this sphere some sort of dumb object?
      bool isEmpty() {
        return radius == 0 
            && priority == 0
            && wavefrontVID == INVALID_VID
            && parentVID == INVALID_VID
            && center == Point();
      }

      /* The below three are required for us to use GenericStateGraph. */
      bool operator==(const Sphere& _other) const {
        return center == _other.center 
            && radius == _other.radius 
            && priority == _other.priority;
      }

      bool operator!=(const Sphere& _other) const {
        return !(*this == _other);
      }

      friend std::ostream& operator<<(std::ostream& _os, const Sphere& _vertex) {
        _os << _vertex.center[0] << " " 
            << _vertex.center[1] << " " 
            << _vertex.center[2];
        return _os;
      }

      /* For pqueue. */
      bool operator < (const Sphere& rhs) const {
        return priority < rhs.priority;
      }
    };

    /* WAVEFRONT expansion structures. */
    // Tree for actual WAVEFRONT expansion. 
    GenericStateGraph<Sphere, std::vector<Sphere>> wavefrontExpansion;
    // Keep track of the spheres and their VIDs in the above graph. 
    std::unordered_map<VID, Sphere> wavefrontExpansionSpheres;
    // Root of the Wavefront. 
    Sphere wavefrontRoot;

  private:

    /// For WAVEFRONT expansion. Insert a sphere into the priority queue q. 
    /// @param pCenter Center point of the new sphere.
    /// @param parentVID Sphere's parent VID.
    /// @param q Queue to insert. 
    /// @returns The sphere inserted. 
    Sphere InsertSphereIntoPQ(Point pCenter, 
                              VID parentVID,
                              std::priority_queue<Sphere>& q);

    Sphere goalSphere; // The sphere that contains the goal. 
    Sphere currentSphere; // The sphere we are currently investigating. (In Iterate())

    // More (mutable) algorithm parameters. 
    double exploreExploitBias; // "sigma" 

    // Is environment 3D?
    bool threeD{true};
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

  m_gaussianSamplerLabel = _node.Read("gaussSamplerLabel", true, "", 
                                      "Gaussian Sampler Label");
  m_nSphereSamples = _node.Read("nSphereSamples", false, m_nSphereSamples, 
                                0, std::numeric_limits<int>::max(),
      "Number of samples on the sphere, for Wavefront Expansion.");
  m_minSphereRadius = _node.Read("minSphereRadius", false, m_minSphereRadius, 
                                0., std::numeric_limits<double>::max(),
      "Minimum sphere radius for sphere growth.");
  m_defaultExploreExploit = _node.Read("defaultExploreExploit", false, 
                                m_defaultExploreExploit, 
                                0., 1., "Default explore/exploit bias. ");
  m_wavefrontExplorationBias = _node.Read("wavefrontExplore", false, 
                                m_wavefrontExplorationBias, 
                                0., 1., "Wavefront exploration bias");
  m_controlManipulation = _node.Read("controlManipulation", false, m_controlManipulation, 
                           0., 1., 
                          "How much exploration grows/shrinks based on sampling success"); 
                                // TODO THIS IS A GROSS NAME
}


/*---------------------------------------------------------------------------*/
template <typename MPTraits>
void
EET<MPTraits>::
Initialize() {
  // set the dimension
  threeD = this->GetTask()->GetRobot()->GetMultiBody()->GetBaseType()
                   == Body::Type::Volumetric;

  // Override some basics. 
  this->m_growGoals = false;

  // Set up variables. 
  exploreExploitBias = m_defaultExploreExploit;

  // Set start, goals, etc
  BasicRRTStrategy<MPTraits>::Initialize();

  //compute sphere tree
  Wavefront();
  currentSphere = wavefrontRoot;

  wavefrontExpansion.Write("wavefront", this->GetMPProblem()->GetEnvironment());
}


template <typename MPTraits>
void
EET<MPTraits>::
Iterate() {
  auto rdmp = this->GetRoadmap();

  // sample a sphere. 
  const CfgType target = this->SelectTarget();

  // find nearest neighbor and expand tree towards target. 
  // if expansion was successful, add to rdmp. 
  // nearest neighbor should not be goal vertex. 
  const VID nearestVID = this->FindNearestNeighbor(target, &rdmp->GetAllVIDs());
  const VID newVID = this->ExpandTree(nearestVID, target);

  // adjust hyperparameters
  if (newVID != INVALID_VID) { // expansion successful. 
    if (this->m_debug){
      std::cout << "This expansion was successful." << std::endl;
    }

    Point newPoint = rdmp->GetVertex(newVID).GetPoint();

    exploreExploitBias *= (1 - m_controlManipulation);

    // Iterate through the spheres from goal to root, 
    // figure out the farthest sphere that this sample is in. 
    // Then, advance. 
    std::vector<Sphere> spheresSeen;
    VID iterationSphereVID = goalSphere.wavefrontVID;
    while (iterationSphereVID != INVALID_VID) { // from goal to root.
      auto iterationSphere = wavefrontExpansionSpheres[iterationSphereVID];
      Point iterationSphereCenter = iterationSphere.center;

      if (Distance(iterationSphereCenter, newPoint) < iterationSphere.radius) {
        if (this->m_debug) {
          std::cout << "This sample is in sphere " 
                    << iterationSphere.wavefrontVID << std::endl;
        }

        // Update current sphere (SS.size() = 0 means goal sphere) and explore params
        currentSphere = spheresSeen.size() == 0 ? iterationSphere : spheresSeen.back();
        exploreExploitBias = m_defaultExploreExploit;

        if (this->m_debug) {
          std::cout << "New \"current\" Sphere has VID: " 
                    << currentSphere.wavefrontVID << std::endl;
        }
        break;
      } 

      // sphere advancement operations.
      spheresSeen.push_back(iterationSphere);
      iterationSphereVID = iterationSphere.parentVID;
    }
  
    // This is not in the pseudocode but is necessary for our implementation.
    this->TryGoalExtension(newVID);

  } else { // expansion unsuccessful. 
    exploreExploitBias *= (1 + m_controlManipulation);
  }


  if (exploreExploitBias > 1) { // Backtrack to previous sphere.
    currentSphere = wavefrontExpansionSpheres[currentSphere.parentVID];
    exploreExploitBias = m_defaultExploreExploit;
  }
}

/*----------------------------------------------------------------------------*/
/*---WAVEFRONT---*/


template <typename MPTraits>
void
EET<MPTraits>::
Wavefront() {
  auto rdmp = this->GetRoadmap();

  // Apparently, GenerateStart and GenerateGoals will not re-generate new S&G.

  // Get start point. 
  const VID startVID = this->GenerateStart(m_samplerLabel);
  CfgType& startCfg = rdmp->GetVertex(startVID);
  Point pStart = startCfg.GetPoint();
  this->startVID = startVID;

  // Get goal point. I don't know any other way to do this.
  std::vector<VID> goals = this->GenerateGoals(m_samplerLabel);
  pGoal = rdmp->GetVertex(goals[0]).GetPoint();
  for (VID goal : goals) { // We don't want to grow the goals. Hence remove them from rdmp.
    rdmp->DeleteVertex(goal);
  }

  // Root the WE. 
  std::priority_queue<Sphere> sphereQueue;
  InsertSphereIntoPQ(pStart, INVALID_VID, sphereQueue);

  // in-loop variables
  std::set<int> goalsReached;

  // Grow WE! 
  while (!sphereQueue.empty()) {
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
                << " Added to WE." << std::endl 
                << "   (There are " << sphereQueue.size() << " elments in the queue)."
                << std::endl;
    }
    if (wavefrontRoot.isEmpty()){
      wavefrontRoot = s;
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
      } else {
        std::cout << "parentVID invalid. This is likely the root node." << std::endl;
      }
    }

    // If we are within a goal region, we are done.
    if (Distance(pGoal, s.center) < s.radius) {
      goalSphere = s;
      if (this->m_debug) {
        std::cout << "   Sphere " << s.wavefrontVID 
                  << ". This sphere contains the goal region." << std::endl;
      }
      break;
    }

    // Sample n points on sphere surface. 
    PointConstruction<MPTraits> _p;
    std::vector<Point> samples;
    if (!threeD) {
      mathtool::Vector3d radius({s.radius, s.radius, 0});
      samples = _p.SampleSphereSurface(s.center, radius, m_nSphereSamples);
    } else {
      samples = _p.SampleSphereSurface(s.center, s.radius, m_nSphereSamples);
    } 
    if (this->m_debug){
      std::cout << "   sampled " << samples.size() 
                << " samples on the surface of sphere with VID " << sVID 
                << std::endl;
    }

    // Calculate sphere diameter for all points
    for (Point p_i : samples) {
      // check which sphere in tree to attach this one to. 

      // not in pseudocode, but we don't want to insert 
      // the same sphere into queue so many times. Priority will be the same. 
      bool sphereAddedToQueue = false; 
      for (auto sphereVID : wavefrontExpansion.GetAllVIDs()) {
        Sphere sphere = wavefrontExpansionSpheres[sphereVID];

        // outside existing spheres
        if (Distance(p_i, sphere.center) < sphere.radius) {
          InsertSphereIntoPQ(p_i, sphereVID, sphereQueue);
          sphereAddedToQueue = true;
        }

        if (sphereAddedToQueue) { break; }
      }
    } // calculate sphere diameter for all pts for loop

  } // tree growth while loop

  if (this->m_debug) {
    std::cout << "Wavefront expansion has "
              << wavefrontExpansion.GetAllVIDs().size() 
              << " nodes." << std::endl;
  }

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
  // This whole function is shamelessly copied from Workspace/ClearanceMap.h
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
InsertSphereIntoPQ(Point pCenter, VID parentVID, std::priority_queue<Sphere>& q) {
  double radius = DistanceToObstacles(pCenter); // radius. 
  
  // Throw out spheres that are too small.. 
  if (radius < m_minSphereRadius) {
    return Sphere();
  }

  // Calculate the priority for this point
  double distance = fabs(Distance(pGoal, pCenter) - radius);

  /* The below few lines is not implemented in the paper's pseudocode. 
  Using priority = [distance to goal] is not a perfect measure because we
  are moving solely in the direction of the goal. This is not necessarily 
  the best option - especially in cluttered environments. 
  Wavefront Expansion should have some extra exploration factor. 
  Hence the following adjustment. 
  */
  // double min_priority;
  // if (m_wavefrontExplorationBias == 0 || parentVID == INVALID_VID) {
  //   min_priority = distance; // I'm p sure this isn't necessary but I don't have the patience to try it out without
  // } else {
  //   double std = m_wavefrontExplorationBias * distance / 4.;
  //   min_priority = GaussianDistribution(distance, std);
  // }
  
  double min_priority = ((1-m_wavefrontExplorationBias) * distance) +
                 (m_wavefrontExplorationBias * distance * ((DRand()*2)-1));


  Sphere s(pCenter, radius, min_priority, parentVID);
  if (this->m_debug) {
    std::cout << "   Added to queue " << 
              "Sphere " << s.center[0] << " " 
                        << s.center[1] << " " 
                        << s.center[2] << " " 
                  << "with radius " << s.radius
              << " and priority " << s.priority
              << " (actual distance=)" << distance 
               << std::endl;
  }

  q.push(s);

  return s;
}

/*-ACTUAL ALGORITHM-----------------------------------------------------------*/
template <class MPTraits>
typename MPTraits::CfgType
EET<MPTraits>::
SelectTarget() {
  auto robot = this->GetTask()->GetRobot();
  CfgType target(robot);

  if (currentSphere == goalSphere && DRand() < m_pGoalState) {
    if (this->m_debug) {
      std::cout << "Current sphere is the goal sphere, and random number lets us choose goal config." << std::endl;
    }
    // return a sample within goal boundary. (not necessarily exact goal configuration, like in the pseudocode.)
    auto& goalConstraints = this->GetTask()->GetGoalConstraints();
    auto goalBoundary = goalConstraints[0]->GetBoundary();
    const Boundary* b = goalBoundary;

    // sample inside boundary. 
    std::vector<CfgType> samples;
    auto s = this->GetSampler(m_samplerLabel);
    s->Sample(1, 100, b, std::back_inserter(samples));
    target = samples.front();

    if(this->m_debug) {
      std::cout << "\t" << target.PrettyPrint() << std::endl;
    }

    return target;
  } 

  // Point _center = currentSphere.center;
  // std::vector<double> boundary_center{_center[0], _center[1], _center[2]};
  // double boundary_radius = currentSphere.radius * exploreExploitBias;
  // CSpaceBoundingSphere samplingBoundary(boundary_center, boundary_radius);
  // b = &samplingBoundary;

  CfgType centerCfg(currentSphere.center, robot);

  // TEMPORARY JANK SOLUTION TILL I CAN FIGURE OUT HOW TO CONTROL GAUSSIANSAMPLER
  GaussianSampler<MPTraits>* gs = (GaussianSampler<MPTraits>*) this->GetSampler(m_gaussianSamplerLabel);
  gs->ReturnNewSamplesOnly(true);
  gs->ChangeGaussianParams(0, currentSphere.radius * exploreExploitBias);
  std::vector<CfgType> samples, collisions;
  while (!samples.size()) {
    gs->Sampler(centerCfg, this->GetEnvironment()->GetBoundary(), samples, collisions);
    // gs->Sample(1, 100, this->GetEnvironment()->GetBoundary(), std::back_inserter(samples));
  }
  target = samples.front();

  if(this->m_debug) {
    std::cout << "\t" << target.PrettyPrint() << std::endl;
  }
  return target;
  
}

template<class MPTraits>
typename EET<MPTraits>::VID
EET<MPTraits>::
ExpandTree(const VID _nearestVID, const CfgType& _target) {
  // Literally just copied from BasicRRTStrategy's ExpandTree
  if(this->m_debug)
    std::cout << "Trying expansion from node " << _nearestVID << " "
         << this->GetRoadmap()->GetVertex(_nearestVID).PrettyPrint()
         << std::endl;

  // Try to extend from the _nearestVID to _target
  return this->Extend(_nearestVID, _target);
}

/*----------------------------------------------------------------------------*/
#endif

#ifndef PMPL_EET_H_
#define PMPL_EET_H_

#include "BasicRRTStrategy.h"
#include "Geometry/Boundaries/CSpaceBoundingSphere.h"

/* HYPERPARAMETER NOTES THAT ANANYA NEEDS TO MOVE 

1. THE MINIMUM RADIUS IS HIGHLY DEPENDENT ON THE CLEARANCE OF
 THE AREA. LOWER CLEARANCE, YOU NEED TO STEP DOWN THE MIN RADIUS
  BY AN ORDER OF MAGNITUDE OR SO. 
2. THE MINIMUM RADIUS IS HIGHLY FINNICKY. IN ONE TEST, MIN RADIUS
 0.1 WAS TOO BIG, 0.01 TOOK FOREVER (TOO SMALL) AND 0.05 WAS JUST RIGHT
3. WHEN CHANGING MIN RADIUS YOU ALSO NEED TO CHANGE NUM SAMPLES 
TAKEN ON SPHERE. RADIUS INCREASES, NUM POINTS SHOULD INCREASE (I THINK)

*/


class EET : public BasicRRTStrategy {
  public:
    typedef typename mathtool::Point3d      Point;
    typedef typename MPBaseObject::WeightType   WeightType;
    typedef typename MPBaseObject::RoadmapType  RoadmapType;
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
    virtual Cfg SelectTarget() override;    
    virtual VID ExpandTree(const VID _nearestVID, const Cfg& _target) override;

    
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
    std::string m_samplerLabel{BasicRRTStrategy::m_samplerLabel};
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

#endif

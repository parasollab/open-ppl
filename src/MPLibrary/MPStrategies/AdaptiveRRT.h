#ifndef ADAPTIVE_RRT_H_
#define ADAPTIVE_RRT_H_

#include "BasicRRTStrategy.h"

// assembly code to measure cpu cycles
static inline uint64_t GetCycles(){
  uint64_t n;
  __asm__ __volatile__ ("rdtsc" : "=A"(n));
  return n;
}

////////////////////////////////////////////////////////////////////////////////
/// Adaptively selects growth methods in RRT.
///
/// References:
///   Denny, Jory, et al. "Adapting RRT growth for heterogeneous environments." 2013
///   IEEE/RSJ International Conference on Intelligent Robots and Systems. IEEE,
///   2013.
///
/// AdaptiveRRT employs structural filtering to the RRT paradigm by providing a
/// two-level cost-adaptive strategy to select the RRT growth method. First,
/// it uses the "visibility" of a node the method selects a set of RRT methods
/// to choose from based on some probability distribution. This probability
/// distribution is updated based upon the success/fail of the growth and its
/// cost.
///
/// @ingroup MotionPlanningStrategies
////////////////////////////////////////////////////////////////////////////////
class AdaptiveRRT : public BasicRRTStrategy {
  public:
    ///@name Motion Planning Types
    ///@{

    // GrowthSet: map<Growth Method, pair<Weight Tuple, Cost>>
    typedef std::map<std::string, std::pair<std::pair<double, long>, double>> GrowthSet;
    // GrowthSets: map<Visibility Threshold, GrowthSet>
    typedef std::map<double, GrowthSet> GrowthSets;

    typedef typename MPBaseObject::WeightType   WeightType;
    typedef typename MPBaseObject::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID           VID;

    ///@}
    ///@name Local Types
    ///@{

    //cost calculation method for AdaptiveRRT
    enum CostMethod {FIXED, REWARD, CYCLES};

    ///@}
    ///@ Construction
    ///@{

    AdaptiveRRT(double _wallPenalty = 0.5, double _gamma = 0.5,
        const GrowthSets& _growthSets = GrowthSets(), CostMethod _c = FIXED);

    AdaptiveRRT(XMLNode& _node);

    //AdaptiveRRT();

    virtual ~AdaptiveRRT() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const;

    ///@}


  protected:
    ///@name MPStrategy Overrides
    ///@{

    virtual void Initialize() override;

    ///@}
    ///@name Tree Helpers
    ///@{

    /// Expand tree in the direction of a given configuration
    /// @param _dir Direction of random configuration
    /// @return VID of newly created Cfg if successful, otherwise INVALID_VID
    virtual VID ExpandTree(Cfg& _dir);

    /// Select growth method from a set of growth methods
    /// @param _gs The growth set containing the growth method we want to access
    /// @return The string indicator for the chosen growth method
    std::string SelectGrowthMethod(GrowthSet& _gs);

    /// Update average cost of growth method
    /// @param _cost Average cost used to update cost of growth method
    /// @param _s String indicator for growth method
    /// @param _gs The growth set containing the growth method we want to access
    void UpdateCost(double _cost, std::string _s, GrowthSet& _gs);

    /// Update weight of growth method given a reward value
    /// @param _r Reward value
    /// @param _s String indicator for growth method
    /// @param _gs The growth set containing the growth method we want to access
    void RewardGrowthMethod(double _r, std::string _s, GrowthSet& _gs);


    /// Updates tree based on expansion type
    /// @param _nearest Vertex ID of nearest configuration
    /// @param _new New configuration to be added to tree
    /// @param _dir Direction of random configuration
    /// @param _delta Expansion distance between _nearest and _new
    /// @return The VID of the newly added configuration
    VID UpdateTree(VID _nearest, Cfg& _new, Cfg& _dir, double _delta);

    /// Adds node to tree and updates visibility
    /// @param _newCfg New configuration to add to tree
    /// @param _nearVID Vertex ID of nearest tree configuration
    /// @param _againstWall Bool flag indicating if _newCfg is against a wall.
    /// @param _ratio Reward ratio
    /// @return The VID of the configuration that was added to the tree
    VID AddNode(Cfg& _newCfg, VID _nearVID, bool _againstWall,
        double _ratio);

    ///@}


  private:

    ///@name Helpers
    ///@{
    /// Updates configuration's running average of visibility with a given value
    /// @param _cfg Configuration who's visibility we want to update
    /// @param _val The value we want to incorporate into the running average of
    ///             visibility
    void AvgVisibility(Cfg& _cfg, double _val);

    /// Calculates cost insensitive probability for a growth method
    /// @param _s String indicator for growth method
    /// @param _gs The growth set containing the growth method we want to access
    /// @return Cost insensitive probability calculated for growth method
    double CostInsensitiveProb(std::string _s, GrowthSet& _gs);

    ///@}
    ///@name Accessors
    ///@{

    /// Gets the visibility of a configuration
    /// @param _cfg Configuration that we want to look up visibility for
    /// @return Visibility value of configuration
    double GetVisibility(Cfg& _cfg);

    /// Gets the cost of a growth method
    /// @param _gs The growth set containing the growth method we want to access
    /// @param _s String indicator for growth method
    /// @return Cost of growth method
    double GetCost(std::string _s, GrowthSet& _gs);

    ///@}
    ///@name Adaptive RRT Properties
    ///@{

    double m_wallPenalty;      ///< Penalty for hitting C-obst, initial visibility for nodes which extend into C-obst

    double m_gamma;            ///< Weighting factor on probability

    GrowthSets m_growthSets;   ///< Growth Strategy set pairs

    CostMethod m_costMethod;   ///< Cost calculation method

    ///@}
};

#endif

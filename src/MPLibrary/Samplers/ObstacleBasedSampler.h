#ifndef OBSTACLE_BASED_SAMPLER_H_
#define OBSTACLE_BASED_SAMPLER_H_

#include "SamplerMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// OBPRM samples by pushing random configurations along a random ray until they
/// change validity, keeping the best free configuration.
///
/// @todo Implement GetCenterOfMass function in the ChooseCenterOfMass class
///
/// @ingroup Samplers
/// @brief Obstacle-based sampling
////////////////////////////////////////////////////////////////////////////////
class ObstacleBasedSampler : virtual public SamplerMethod {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPBaseObject::GroupCfgType GroupCfgType;
    typedef std::map<Robot*, const Boundary*> BoundaryMap;

    ///@}
    ///@name Construction
    ///@{

    ObstacleBasedSampler(string _vcLabel = "", string _dmLabel = "",
        int _free = 1, int _coll = 0, double _step = 0,
        bool _useBBX = true, string _pointSelection = "cspace");

    ObstacleBasedSampler(XMLNode& _node);

    virtual ~ObstacleBasedSampler() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const override;

    ///@}

  protected:

    // ///@name XML Parser
    // ///@{

    // void ParseXML(XMLNode& _node);

    // ///@}
    ///@name Helpers
    ///@{
    
    /// Main Sampler
    /// Sampler for a single robot in a single boundary
    /// @param _cfg Configuration Type
    /// @param _boundary Boundary of the environment
    /// @param _result A vector that stores Free Configuration Space
    /// @param _collision A vector that stores Collision Configuration Space
    /// @return Success or Failure of sampling
    virtual bool Sampler(Cfg& _cfg, const Boundary* const _boundary,
        std::vector<Cfg>& _result, std::vector<Cfg>& _collision);
    
    /// Sampler for multiple robots in a single boundary
    /// @param _cfg Configuration Type
    /// @param _boundary Boundary of the environment
    /// @param _result A vector that stores Free Group Configuration Space
    /// @param _collision A vector that stores Collision Group Configuration Space
    /// @return Success or Failure of sampling
    virtual bool Sampler(GroupCfgType& _cfg, const Boundary* const _boundary,
        std::vector<GroupCfgType>& _result, std::vector<GroupCfgType>& _invalid);

    /// Sampler for multiple robots in each individual boundary
    /// @param _cfg Configuration Type
    /// @param _boundaryMap BoundaryMap of the environment
    /// @param _result A vector that stores Free Group Configuration Space
    /// @param _collision A vector that stores Collision Group Configuration Space
    /// @return Success or Failure of sampling
    virtual bool Sampler(GroupCfgType& _cfg, const BoundaryMap& _boundaryMap,
        std::vector<GroupCfgType>& _result, std::vector<GroupCfgType>& _collision);

    /// Generates and adds shells to their containers 
    /// for a single robot in a single boundary case
    /// @param _boundary Boundary of the environment
    /// @param _cFree The configuration in the Free Configuration Space
    /// @param _cColl The configuration in the Collision Configuration Space
    /// @param _incr The amount of increments in the specified direction.
    /// @param _result A vector that stores Free Configuration Space
    /// @param _collision A vector that stores Collision Configuration Space
    void GenerateShells(const Boundary* const _boundary,
        Cfg& _cFree, Cfg& _cColl, Cfg& _incr,
        std::vector<Cfg>& _result, std::vector<Cfg>& _collision);

    /// Generates and adds shells to their containers
    /// for multiple robots in a single boundary case
    /// @param _boundary Boundary of the environment
    /// @param _cFree The configuration in the Free Configuration Space
    /// @param _cColl The configuration in the Collision Configuration Space
    /// @param _incr The amount of increments in the specified direction.
    /// @param _result A vector that stores Free Group Configuration Space
    /// @param _collision A vector that stores Collision Group Configuration Space
    void GenerateShells(const Boundary* const _boundary,
        GroupCfgType& _cFree, GroupCfgType& _cColl, GroupCfgType& _incr,
        std::vector<GroupCfgType>& _result, std::vector<GroupCfgType>& _collision);
    
    /// Generates and adds shells to their containers
    /// for multiple robots in each individual boundary case
    /// @param _boundary Boundary of the environment
    /// @param _cFree The configuration in the Free Configuration Space
    /// @param _cColl The configuration in the Collision Configuration Space
    /// @param _incr The amount of increments in the specified direction.
    /// @param _result A vector that stores Free Group Configuration Space
    /// @param _collision A vector that stores Collision Group Configuration Space
    // void GenerateShells(const Environment* const _environment,
    void GenerateShells(const BoundaryMap& _boundaryMap,
        GroupCfgType& _cFree, GroupCfgType& _cColl, GroupCfgType& _incr,
        std::vector<GroupCfgType>& _result, std::vector<GroupCfgType>& _collision);

    /// Returns a Cfg at the center of mass of the MultiBody
    /// @param _mBody Multibody of the obstacle
    /// @return Configuration corresponding to the center of mass of the Multibody
    Cfg ChooseCenterOfMass(MultiBody* _mBody);

    // Returns a Cfg at a random vertex of the MultiBody
    /// @param _mBody Multibody of the obstacle
    /// @return Configuration chosen from the Random Vertex method
    Cfg ChooseRandomVertex(MultiBody* _mBody);

    /// Returns a point inside the triangle determined by the vectors
    /// @param _p The first 3d point of a triangle
    /// @param _q The second 3d point of a triangle
    /// @param _r The Third 3d point of a triangle
    /// @return Configuration chosen from the Point On Triangle method
    Vector3d ChoosePointOnTriangle(Vector3d _p, Vector3d _q, Vector3d _r);

    /// Chooses a random point on a random triangle (weighted by area) in the MultiBody
    /// @param _mBody Multibody of the obstacle
    /// @return Configuration chosen from the Random Weighted Triangle method
    Cfg ChooseRandomWeightedTriangle(MultiBody* _mBody);

    /// Chooses a random point in a random triangle in the MultiBody
    /// @param _mBody Multibody of the obstacle
    /// @return Configuration chosen from the Random Triangle method
    Cfg ChooseRandomTriangle(MultiBody* _mBody);

    /// Chooses a random extreme vertex of the MultiBody
    /// @param _mBody Multibody of the obstacle
    /// @return Configuration chosen from the Extreme Vertex method
    Cfg ChooseExtremeVertex(MultiBody* _mBody);

    /// Checks m_pointSelection and returns an appropriate Cfg
    /// @param _cfg Configuration of the sample
    /// @return Configuration of the sample
    virtual Cfg ChooseASample(Cfg& _cfg);

    /// Checks m_pointSelection and returns an appropriate GroupCfgType
    /// @param _cfg Group configuration of the sample
    /// @return Group configuration chosen from the Extreme Vertex method
    virtual GroupCfgType ChooseASample(GroupCfgType& _cfg);

    /// Call the various sample method other than cspace method
    /// @return Configuration obtained according to the m_pointSelection
    Cfg ChooseASampleOtherMethods();

    /// Returns a Cfg with the coordinates specified in the vector and no rotation
    /// @param _v Multibody of the obstacle
    /// @return Configuration Type of the sample
    Cfg GetCfgWithParams(const Vector3d& _v);

    /// Checks if the groupCfg is in the boundary map
    /// @param _groupCfg The group configuration needs to be checked
    /// @param _boundaryMap The map that pairs each robot and corresponding boundary
    /// @return true or false
    bool GroupInBounds(GroupCfgType& _groupCfg, const BoundaryMap& _boundaryMap);

    /// Checks if the groupCfg is in the boundary map
    /// @param _groupCfg The group configuration needs to be checked
    /// @param _boundary The single boundary of every robots
    /// @return true or false
    bool GroupInBounds(GroupCfgType& _groupCfg, const Boundary* const _boundary);
    ///@}

  private:

    ///@name Internal States
    ///@{
    
    string m_vcLabel, m_dmLabel; ///< Validity checker method, distance metric method
    int m_nShellsFree, m_nShellsColl; ///< Number of free and collision shells
    double m_stepSize; ///< Step size along the random ray
    bool m_useBBX; ///< Is the bounding box an obstacle?
    string m_pointSelection; ///< Needed for the WOBPRM

    ///@}
};

#endif

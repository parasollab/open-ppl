#ifndef UNIFORM_OBSTACLE_BASED_SAMPLER_H_
#define UNIFORM_OBSTACLE_BASED_SAMPLER_H_

#include "SamplerMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// This sampler generates obstacle-based configurations that uniformly cover
/// the contact surface. It first generates a set of uniformly distributed fixed length
/// segments, and then tests intermediate points on the segments in order to find 
/// valid configurations adjacent to invalid configurations. Those are retained as
/// roadmap nodes.
///
/// Reference: https://bit.ly/3P1f8dq
///   Hsin-Yi Yeh and Shawna Thomas and David Eppstein and Nancy M. Amato. 
///   "UOBPRM: A uniformly distributed obstacle-based PRM". TRO 2012.
///
/// @ingroup Samplers
////////////////////////////////////////////////////////////////////////////////
class UniformObstacleBasedSampler : virtual public SamplerMethod {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPBaseObject::GroupCfgType GroupCfgType;

    ///@}
    ///@name Construction
    ///@{

    UniformObstacleBasedSampler(XMLNode& _node);

    virtual ~UniformObstacleBasedSampler() = default;

    UniformObstacleBasedSampler(string _vcLabel = "", string _dmLabel = "",
    double _margin = 0, bool _useBoundary = false);

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    void Print(ostream& _os) const override;

    ///@}
    ///@name Sampler Interface
    ///@{

    /// Takes the input configuration and applies the sampler rule to
    /// generate output configurations on the contact surface.
    /// @param _cfg The input configuration.
    /// @param _boundary The sampling boundary.
    /// @param _result The resulting output configurations. 
    /// @param _invalid The (optional) return for failed attempts. 
    /// @return true if a valid configuration was generated, false otherwise.
    virtual bool Sampler(Cfg& _cfg, const Boundary* const _boundary,
        std::vector<Cfg>& _result, std::vector<Cfg>& _invalid) override;

    virtual bool Sampler(GroupCfgType& _cfg, const Boundary* const _boundary,
    std::vector<GroupCfgType>& _result, std::vector<GroupCfgType>& _invalid) override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    double m_margin; //The length of line segments 
    bool m_useBoundary; //Use bounding box as obstacle 
    string m_vcLabel, m_dmLabel; //Validity checker label, distance metric label

    ///@}
};

#endif

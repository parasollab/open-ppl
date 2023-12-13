#ifndef PPL_BRIDGE_TEST_SAMPLER_H_
#define PPL_BRIDGE_TEST_SAMPLER_H_

#include "SamplerMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Samplers
/// @brief This sampler validity checks the input sample and accepts it iff it 
/// passes the bridge test - i.e., a random, Gaussian ray places the sample
/// in the center of a ray and it only passes if the sample is valid but the 
/// ray's two endpoints are invalid. 
////////////////////////////////////////////////////////////////////////////////
class BridgeTestSampler : virtual public SamplerMethod {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPBaseObject::GroupCfgType GroupCfgType;
    typedef std::map<Robot*, const Boundary*>   BoundaryMap;
  
    ///@}
    ///@name Construction
    ///@{

    BridgeTestSampler(string _vcLabel = "", string _dmLabel = "",
        double _d = 0.5, bool _useBoundary = false);

    BridgeTestSampler(XMLNode& _node);

    virtual ~BridgeTestSampler() = default;

    ///@}
    ///@name MPBaseObjectOverrides
    ///@{

    virtual void Print(ostream& _os) const override;

    ///@}
    ///@name Parser
    ///@{

    void ParseXML(XMLNode& _node);

    ///@}
    ///@name Sampler Rule
    ///@{

    virtual bool Sampler(GroupCfgType& _cfg, const Boundary* const _boundary,
        vector<GroupCfgType>& _valid, vector<GroupCfgType>& _invalid);

    virtual bool Sampler(Cfg& _cfg, const Boundary* const _boundary,
        vector<Cfg>& _valid, vector<Cfg>& _invalid);

    virtual bool Sampler(GroupCfgType& _cfg, const BoundaryMap& _boundaryMap,
        std::vector<GroupCfgType>& _valid, std::vector<GroupCfgType>& _invalid);

    ///@}

  protected:

    ///@name Internal State
    ///@{ 

    double m_d;  ///< Gaussian d-value obtained from distribution
    bool m_useBoundary;  ///< use Bbox as obstacle?
    std::string m_vcLabel, m_dmLabel;  ///< The distance metric to use

    ///@}
    ///@name Helpers
    ///@{ 

    /// Check if all configurations within a group are inside their respective 
    /// boundaries
    bool GroupInBounds(GroupCfgType& _gcfg, const BoundaryMap& _boundaryMap);

    ///@}
};

#endif

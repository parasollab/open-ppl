#ifndef PMPL_PATH_SAMPLER_STRATEGY_H_
#define PMPL_PATH_SAMPLER_STRATEGY_H_

#include "MPStrategyMethod.h"

#include "Geometry/Boundaries/CSpaceBoundingSphere.h"

////////////////////////////////////////////////////////////////////////////////
/// Work in progress - A strategy that fits many bounding spheres along a path
/// and randomly samples from them.
///
/// @ingroup MotionPlanningStrategies
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class PathSamplerStrategy : public MPStrategyMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;

    ///@}
    ///@name Construction
    ///@{

    PathSamplerStrategy();

    PathSamplerStrategy(XMLNode& _node);

    virtual ~PathSamplerStrategy() = default;

    ///@}

  private:

    ///@name MPStrategyMethod Overrides
    ///@{

    virtual void Initialize() override;
    virtual void Iterate() override;

    ///@}
    ///@name Internal State
    ///@{

    std::string m_samplerLabel;
    std::string m_strategyLabel;
    double m_radius; ///< The region radius.
    double m_resolution; ///< The distance between spheres.

    std::vector<CSpaceBoundingSphere> m_regions;
    RoadmapType m_roadmap;

    ///@}

};

template <typename MPTraits>
PathSamplerStrategy<MPTraits>::
PathSamplerStrategy() {
  this->SetName("PathSamplerStrategy");
}

template <typename MPTraits>
PathSamplerStrategy<MPTraits>::
PathSamplerStrategy(XMLNode& _node) : MPStrategyMethod<MPTraits>(_node) {
  this->SetName("PathSamplerStrategy");

  m_samplerLabel = _node.Read("samplerLabel", true, "",
      "region sampler label to use");

  m_strategyLabel = _node.Read("strategyLabel", true, "",
      "strategy label to generate path");

  m_radius = _node.Read("radius", true, m_radius, 0.01, 
      1000.0, "radius of the regions");
  
  m_resolution = _node.Read("resolution", true, m_resolution, 0.0001, 
      1000.0, "distance between regions");
}

template <typename MPTraits>
void
PathSamplerStrategy<MPTraits>::
Initialize() {
  // Generate the path using the given strategy
  auto s = this->GetMPStrategy(m_strategyLabel);
  (*s)();

  RoadmapType* r = this->GetRoadmap();
  auto vids = this->GetMPSolution()->GetPath()->VIDs();

  if(this->m_debug)
    std::cout << "Input roadmap has " << r->get_num_vertices()
              << " nodes and " << r->get_num_edges() << " edges.\n"
              << "Shortest path has " << vids.size() << "VIDs."
              << " Making regions around path..." << std::endl;
  
  // Make regions around the path edges (assume straight line edges)
  auto svid = vids.at(0);
  auto tvid = vids.at(0);
  for(size_t i = 1; i < vids.size(); ++i) {
    svid = tvid;
    tvid = vids.at(i);

    auto scfg = this->GetRoadmap()->GetVertex(svid);
    auto tcfg = this->GetRoadmap()->GetVertex(tvid);

    auto spoint = scfg.GetPoint();
    auto tpoint = tcfg.GetPoint();

    auto disp = tpoint - spoint;
    int npoints = (int) ceil(disp.norm() / m_resolution);

    for(int j = 0; j < npoints; ++j) {
      Point3d d = {j * disp[0]/npoints,
                   j * disp[1]/npoints, 
                   j * disp[2]/npoints};

      auto center = spoint + d;

      const bool threeD = this->GetTask()->GetRobot()->GetMultiBody()->GetBaseType()
                   == Body::Type::Volumetric;

      if (threeD)
        m_regions.push_back(CSpaceBoundingSphere({center[0], center[1], center[2]}, m_radius));
      else
        m_regions.push_back(CSpaceBoundingSphere({center[0], center[1]}, m_radius));
    }
  }

  // Print original roadmap
  const std::string base = this->GetBaseFilename();
  this->GetRoadmap()->Write(base + "." + m_strategyLabel + ".map",this->GetEnvironment());
  
  // Reset the roadmap now
  m_roadmap = RoadmapType(this->GetTask()->GetRobot());
  this->GetMPSolution()->SetRoadmap(this->GetTask()->GetRobot(), &m_roadmap);
}

template <typename MPTraits>
void
PathSamplerStrategy<MPTraits>::
Iterate() {
  // Sample from a randomly chosen region
  int regionIdx = (int) floor(DRand() * m_regions.size());
  if(this->m_debug)
    std::cout << "Sampling from region " << regionIdx << std::endl;

  auto boundary = m_regions.at(regionIdx);

  std::vector<CfgType> samples;
  auto sampler = this->GetSampler(m_samplerLabel);
  sampler->Sample(1, 100, &boundary, std::back_inserter(samples));

  for(auto& sample : samples)
    this->GetRoadmap()->AddVertex(sample);
}

#endif

#ifndef PMPL_OPTIMAL_NF_H_
#define PMPL_OPTIMAL_NF_H_

#include "NeighborhoodFinderMethod.h"
#include "BruteForceNF.h"
#include "RadiusNF.h"

#include <cmath>
#include <functional>


////////////////////////////////////////////////////////////////////////////////
/// Compute the "optimal" nearest-neighbor set defined for asymptotically-optimal
/// planners. This isn't really optimal since it is driven by random geometric
/// graph theory which doesn't account for obstacles, but in the asymptotic
/// limit it converges to the optimal behavior. It should only be used for
/// optimal planners like RRT* or PRM*.
///
/// Reference:
///   Sertac Karaman and Emilio Frazzoli. "Sampling-based algorithms for optimal
///   motion planning". IJRR 2011.
///
/// @ingroup NeighborhoodFinders
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class OptimalNF : public NeighborhoodFinderMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType                CfgType;
    typedef typename MPTraits::RoadmapType            RoadmapType;
    typedef typename RoadmapType::VID                 VID;
    typedef typename RoadmapType::VertexSet           VertexSet;
    typedef typename MPTraits::GroupRoadmapType       GroupRoadmapType;
    typedef typename MPTraits::GroupCfgType           GroupCfgType;

    typedef typename MPTraits::MPLibrary::NeighborhoodFinderPointer
                                                      NeighborhoodFinderPointer;

    ///@}
    ///@name Local Types
    ///@{

    using typename NeighborhoodFinderMethod<MPTraits>::Type;
    using typename NeighborhoodFinderMethod<MPTraits>::OutputIterator;

    ///@}
    ///@name Construction
    ///@{

    OptimalNF();

    OptimalNF(XMLNode& _node);

    virtual ~OptimalNF();

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Initialize() override;

    ///@}
    ///@name NeighborhoodFinderMethod Overrides
    ///@{

    virtual void FindNeighbors(RoadmapType* const _r, const CfgType& _cfg,
        const VertexSet& _candidates, OutputIterator _out) override;

    virtual void FindNeighbors(GroupRoadmapType* const _r,
        const GroupCfgType& _cfg, const VertexSet& _candidates,
        OutputIterator _out) override;

    ///@}

  protected:

    ///@name Internal State
    ///@{

    NeighborhoodFinderPointer m_nf; ///< The internal neighborhood finder.

    /// This function adjusts the NF parameters based on the current roadmap
    /// size before each run.
    std::function<void(const RoadmapType* const)> m_setParameters;

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
OptimalNF<MPTraits>::
OptimalNF() : NeighborhoodFinderMethod<MPTraits>() {
  this->SetName("OptimalNF");
}


template <typename MPTraits>
OptimalNF<MPTraits>::
OptimalNF(XMLNode& _node) : NeighborhoodFinderMethod<MPTraits>(_node) {
  this->SetName("OptimalNF");

  // We must not have the 'unconnected' option set for this to work.
  if(this->m_unconnected)
    throw ParseException(_node.Where()) << "OptimalNF requires unconnected = "
                                        << "false.";

  std::string exLabel = _node.Read("exLabel", false, "",
      "The RRT extender label if used with an RRT method. If supplied, the "
      "optimal radius will not exceed the extender's max distance.");

  // Parse the type (radius or k).
  const std::string choices = "Choices are 'radius' or 'k'.";
  std::string nfType = _node.Read("nfType", true, "",
      "Type of neighbors to find. " + choices);
  std::transform(nfType.begin(), nfType.end(), nfType.begin(), ::tolower);

  constexpr double e = std::exp(1.);

  // Set the NF type, internal nf, and parameter function according to the
  // chosen type.
  if(nfType == "radius") {
    this->m_nfType = Type::RADIUS;

    // Create a radius NF.
    m_nf = NeighborhoodFinderPointer(new RadiusNF<MPTraits>());

    // Define the parameter function to set K based on the roadmap size and
    // dimension.
    m_setParameters = [this, e, exLabel](const RoadmapType* const _r) {
      auto robot  = _r->GetRobot();
      auto cspace = robot->GetCSpace();
      auto vspace = robot->GetVSpace();
      const size_t dimension = cspace->GetDimension()
                             + (vspace ? vspace->GetDimension() : 0);
      const double rd = 1. / dimension;

      // There is no efficient way to compute or even estimate the hypervolume
      // of cfree, which is called for by this method. Fortunately we can use an
      // over-estimate, so we will use the volume of the full cspace instead.
      /// @todo If we discover a good means of estimating the hypervolume of
      ///       cfree efficiently, use that here instead for a tighter radius.
      const double cfreeHyperVolume = cspace->GetVolume()
                                    * (vspace ? vspace->GetVolume() : 1);

      // The actual hypervolume of a unit ball in R^dimension space.
      // Ref: https://en.wikipedia.org/wiki/Volume_of_an_n-ball
      const double halfD = dimension / 2.;
      const double unitBallHyperVolume = std::pow(PI, halfD)
                                       / std::tgamma(halfD + 1);

      // The volume ratio is basically the number of unit balls in cfree
      // (although we've instead used the number of balls in cspace since we
      // can't compute the measure of cfree).
      const double volumeRatio = cfreeHyperVolume / unitBallHyperVolume;

      // Find the optimal radius.
      const double gamma = 2. * std::pow(1. + rd, rd) * std::pow(volumeRatio, rd);
      const double optimalR = gamma
                            * std::pow(std::log(_r->Size()) / _r->Size(), rd);
      if(exLabel.empty())
        this->m_nf->GetRadius() = optimalR;
      else {
        auto ex = this->GetExtender(exLabel);
        this->m_nf->GetRadius() = std::min(optimalR, ex->GetMaxDistance());
      }

      if(this->m_debug)
        std::cout << "Finding closest neighbors with radius = "
                  << this->m_nf->GetRadius() << " (extender: "
                  << (exLabel.empty() ? "none" : exLabel) << ")."
                  << std::endl;
    };
  }
  else if(nfType == "k") {
    this->m_nfType = Type::K;
    this->GetK() = 1;

    // Create a k-nearest NF (currently we use brute force but can probably
    // switch this to kd-tree).
    m_nf = NeighborhoodFinderPointer(new BruteForceNF<MPTraits>());

    // Define the parameter function to set K based on the roadmap size and
    // dimension.
    m_setParameters = [this, e](const RoadmapType* const _r) {
      auto robot  = _r->GetRobot();
      auto vspace = robot->GetVSpace();
      const size_t dimension = robot->GetCSpace()->GetDimension()
                             + (vspace ? vspace->GetDimension() : 0);
      const double k = e * (1. + 1. / dimension);
      const size_t optimalK = std::ceil(k * std::log(_r->Size()));
      this->m_nf->GetK() = optimalK;
      if(this->m_debug)
        std::cout << "Finding closest neighbors with k = " << this->m_nf->GetK()
                  << "."
                  << std::endl;
    };
  }
  else
    throw ParseException(_node.Where()) << "Unrecognized nfType. " << choices;

  m_nf->SetDMLabel(this->GetDMLabel());
}


template <typename MPTraits>
OptimalNF<MPTraits>::
~OptimalNF() = default;

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
OptimalNF<MPTraits>::
Initialize() {
  // Make sure the internal NF points to the correct MPLibrary.
  m_nf->SetMPLibrary(this->GetMPLibrary());
}

/*-------------------- NeighborhoodFinderMethod Interface --------------------*/

template <typename MPTraits>
void
OptimalNF<MPTraits>::
FindNeighbors(RoadmapType* const _r, const CfgType& _cfg,
    const VertexSet& _candidates, OutputIterator _out) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::FindNeighbors");

  // Update parameters and compute neighbors.
  m_setParameters(_r);
  m_nf->FindNeighbors(_r, _cfg, _candidates, _out);
}


template <typename MPTraits>
void
OptimalNF<MPTraits>::
FindNeighbors(GroupRoadmapType* const _r, const GroupCfgType& _cfg,
    const VertexSet& _candidates, OutputIterator _out) {
  throw NotImplementedException(WHERE);
}

/*----------------------------------------------------------------------------*/

#endif

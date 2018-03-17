#ifndef GRID_SAMPLER_H_
#define GRID_SAMPLER_H_

#include "SamplerMethod.h"

#include <algorithm>


////////////////////////////////////////////////////////////////////////////////
/// Generate configurations along a grid through a submanifold in @cspace. The
/// DOFs not specified by the grid will be (independently) randomly sampled.
///
/// @ingroup Samplers
////////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
class GridSampler : public SamplerMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Local Types
    ///@{

    using typename SamplerMethod<MPTraits>::InputIterator;
    using typename SamplerMethod<MPTraits>::OutputIterator;

    ///@}
    ///@name Construction
    ///@{

    GridSampler(std::string _vcm = "", std::vector<size_t> _numPoints = {});

    GridSampler(XMLNode& _node);

    virtual ~GridSampler() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;
    virtual void Initialize() override;

    ///@}
    ///@name SamplerMethod Overrides
    ///@{

    /// For a grid sampler, the number of nodes is the desired number of nodes
    /// PER GRID POINT, so usually this will be 1. The max attempts is also per
    /// grid point.
    virtual void Sample(size_t _numNodes, size_t _maxAttempts,
        const Boundary* const _boundary, OutputIterator _result,
        OutputIterator _collision) override;

    /// Not implemented for this method.
    virtual void Sample(InputIterator _first, InputIterator _last,
        size_t _maxAttempts, const Boundary* const _boundary,
        OutputIterator _result, OutputIterator _collision) override;

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Parse the ticks string, which defines the number of ticks to use in each
    /// grid dimension.
    void ParseTicksString(XMLNode& _node, const std::string& _s);

    /// Reset the ticker to its starting position.
    void ResetTicker();

    /// Advance the ticker to the next grid position.
    void AdvanceTicker();

    /// Position a configuration at the current ticker position.
    void PositionAtTicker(CfgType& _cfg, const Boundary* const _b) const;

    ///@}
    ///@name Internal State
    ///@{

    std::string m_vcLabel;        ///< Validity checker label.
    std::vector<size_t> m_ticks;  ///< The number of ticks in each dimension.
    std::vector<size_t> m_ticker; ///< Current tick in sampling process.
    size_t m_numPoints{0};        ///< The number of points in the grid.

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
GridSampler<MPTraits>::
GridSampler(string _vcm, std::vector<size_t> _ticks)
    : m_vcLabel(_vcm), m_ticks(_ticks) {
  this->SetName("GridSampler");
}


template <typename MPTraits>
GridSampler<MPTraits>::
GridSampler(XMLNode& _node) : SamplerMethod<MPTraits>(_node) {
  this->SetName("GridSampler");

  m_vcLabel = _node.Read("vcLabel", true, "", "Validity test method");

  const std::string ticksString = _node.Read("ticks", true, "", "Either a "
      "string of numbers where each represents the number of ticks in one "
      "dimension (starting from dof 0), or a single number which represents the "
      "number of ticks in all dimensions.");

  ParseTicksString(_node, ticksString);
}

/*------------------------- MPBaseObject Overrides ---------------------------*/

template <typename MPTraits>
void
GridSampler<MPTraits>::
Print(std::ostream& _os) const {
  SamplerMethod<MPTraits>::Print(_os);

  _os << "\tvcLabel: " << m_vcLabel
      << "\n\tticks: " << m_ticks
      << "\n\ttotal points: " << m_numPoints
      << std::endl;
}


template <typename MPTraits>
void
GridSampler<MPTraits>::
Initialize() {
  // If we have only one point token, assume we want to use that value as the
  // number of points in each DOF.
  if(m_ticks.size() == 1)
    m_ticks.resize(this->GetTask()->GetRobot()->GetMultiBody()->DOF(),
                   m_ticks[0]);

  // Compute the total number of positions in the grid.
  m_numPoints = 1;
  for(auto numTicks : m_ticks)
    m_numPoints *= numTicks;
}

/*------------------------- SamplerMethod Overrides --------------------------*/

template <typename MPTraits>
void
GridSampler<MPTraits>::
Sample(size_t _numNodes, size_t _maxAttempts,
    const Boundary* const _boundary,
    OutputIterator _result, OutputIterator _collision) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetName() + "::Sample");

  // Ensure that the boundary has enough dimensions.
  if(m_ticks.size() > _boundary->GetDimension())
    throw RunTimeException(WHERE, "Boundary has too few dimensions " +
        std::to_string(_boundary->GetDimension()) + ", expected at least " +
        std::to_string(m_ticks.size()) + ".");

  const std::string callee = this->GetName() + "::Sampler";
  auto vc = this->GetValidityChecker(m_vcLabel);

  CfgType cfg(this->GetTask()->GetRobot());
  ResetTicker();

  // Create samples at each grid point.
  for(size_t n = 0; n < m_numPoints; ++n) {
    if(this->m_debug)
      std::cout << "Sampling point " << n << " / " << m_numPoints
                << "\n\tTicker position: " << m_ticker << " / " << m_ticks
                << std::endl;

    // Try to sample _numNodes points at this position.
    for(size_t i = 0; i < _numNodes; ++i) {
      // Try to sample a point at this position up to _maxAttempts times.
      for(size_t attempts = 0; attempts < _maxAttempts; ++attempts) {
        // Sample a new configuration in the boundary.
        stats->IncNodesAttempted(this->GetNameAndLabel());
        cfg.GetRandomCfg(_boundary);

        // Overwrite the DOFs which are specified by the grid.
        PositionAtTicker(cfg, _boundary);

        // Evaluate validity and store the result.
        if(cfg.InBounds(_boundary) and vc->IsValid(cfg, callee)) {
          if(this->m_debug)
            std::cout << "\tGenerated valid cfg at " << cfg.PrettyPrint()
                      << std::endl;

          stats->IncNodesGenerated(this->GetNameAndLabel());
          _result++ = cfg;
          break;
        }
        else
          _collision++ = cfg;
      }
    }
    // We are done attempting for this position; advance the ticker to the next
    // one.
    AdvanceTicker();
  }
}


template <typename MPTraits>
void
GridSampler<MPTraits>::
Sample(InputIterator _first, InputIterator _last,
    size_t _maxAttempts, const Boundary* const _boundary,
    OutputIterator _result, OutputIterator _collision) {
  throw RunTimeException(WHERE, "GridSampler does not support the filtering "
      "version of Sample.");
}

/*-------------------------------- Helpers -----------------------------------*/

template <typename MPTraits>
void
GridSampler<MPTraits>::
ParseTicksString(XMLNode& _node, const std::string& _s) {
  m_ticks.clear();

  // Accept space, comma, or semicolon as a delimiter.
  std::vector<std::string> tokens = nonstd::tokenize(_s, ",; ");

  // Convert each token into an integer. The conversion may throw an
  // out-of-range if it doesn't work. Upgrade those to our exception so that we
  // can see where it came from.
  /// @TODO It would be a good idea to add a regex check for non-numeric
  ///       characters, but the lab's gcc version (4.8.5) doesn't currently
  ///       support <regex>.
  try {
    for(const auto& token : tokens) {
      const size_t numPoints = std::stoul(token);
      m_ticks.push_back(numPoints);
    }
  }
  catch(const std::exception&) {
    throw ParseException(_node.Where(), "Could not convert the point string '"
        + _s + "' into a set of unsigned integers.");
  }
}


template <typename MPTraits>
void
GridSampler<MPTraits>::
ResetTicker() {
  m_ticker.resize(m_ticks.size(), 0);
}


template <typename MPTraits>
void
GridSampler<MPTraits>::
AdvanceTicker() {
  const size_t dimensions = m_ticks.size();

  // Advance along the first dimension.
  ++m_ticker[0];

  // If it now exceeds the max number of ticks, advance the next one. Cascade as
  // we over-extend the limits.

  for(size_t d = 0; d < dimensions; ++d)
  {
    // If we are not at the end, quit now
    if(m_ticker[d] < m_ticks[d])
      break;

    // Otherwise we are at the end. Reset this dimension to the 0th tick and
    // increment the next one.
    m_ticker[d] = 0;
    if(d + 1 < dimensions)
      ++m_ticker[d + 1];
  }
}


template <typename MPTraits>
void
GridSampler<MPTraits>::
PositionAtTicker(CfgType& _cfg, const Boundary* const _b) const {
  for(size_t d = 0; d < m_ticker.size(); ++d) {
    // Skip dimensions with no ticks.
    if(m_ticks[d] == 0)
      continue;

    // If this dimension has only one tick, use the center point.
    if(m_ticks[d] == 1)
      _cfg[d] = _b->GetCenter()[d];
    // Otherwise extrapolate the appropriate fraction of the range in dimension
    // d.
    else {
      auto r = _b->GetRange(d);
      _cfg[d] = r.min + r.Length() * m_ticker[d] / (m_ticks[d] - 1);
    }
  }
}

/*----------------------------------------------------------------------------*/

#endif

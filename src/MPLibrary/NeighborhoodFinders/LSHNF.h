#ifndef PMPL_LSHNF_H_
#define PMPL_LSHNF_H_

#include "NeighborhoodFinderMethod.h"

#include <map>
#include <utility>
#include <vector>


////////////////////////////////////////////////////////////////////////////////
/// This method implements a locality sensitive hashing-based approximate
/// neigborhood finder.
///
/// Reference:
///   Mayur Datar and Nicloe Immorlica and Piotr Indyk and Vahab S. Mirronki.
///   "Locality-sensitive Hashing Scheme Based on P-stable Distributions".
///   Proceedings of the Twentieth Annual Symposium on Computational Geometry,
///   2004.
///
/// @ingroup NeighborhoodFinders
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class LSHNF : public NeighborhoodFinderMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType          CfgType;
    typedef typename MPTraits::RoadmapType      RoadmapType;
    typedef typename RoadmapType::VI            VI;
    typedef typename RoadmapType::VID           VID;
    typedef typename MPTraits::GroupRoadmapType GroupRoadmapType;
    typedef typename MPTraits::GroupCfgType     GroupCfgType;

    ///@}
    ///@name Local Types
    ///@{

    using typename NeighborhoodFinderMethod<MPTraits>::Type;
    using typename NeighborhoodFinderMethod<MPTraits>::OutputIterator;

    /// The a vector and b scalar that define our hash functions.
    typedef std::pair<std::vector<double>,double> hash_parameters;

    /// A hash family is a set of hash functions.
    typedef std::vector<hash_parameters> HashFamily;

    /// The hash key (or bucket) for a hash family.
    typedef std::vector<int> HashKey;

    /// Maps a hash key to a set of VIDs.
    typedef std::multimap<HashKey, VID> BucketMap;

    ///@}
    ///@name Construction
    ///@{

    LSHNF();

    LSHNF(XMLNode& _node);

    virtual ~LSHNF();

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Initialize() override;

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name NeighborhoodFinder Methods
    ///@{

    template <typename InputIterator>
    void FindNeighbors(RoadmapType* _r,
        InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
        const CfgType& _cfg, OutputIterator _out);

    template <typename InputIterator>
    void FindNeighbors(GroupRoadmapType* _r,
        InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
        const GroupCfgType& _cfg, OutputIterator _out);

    ///@}

  private:

    ///@name Helper Functions
    ///@{

    /// Generate parameters for a hash family,
    /// @return A set of parameters that compose a hash family.
    std::vector<hash_parameters> MakeHashFamily() const;

    /// Compute a single value of a hash key for some configuration.
    /// @param _cfg The configuration.
    /// @param _h The hash function.
    /// @return The hash value of _cfg using _h.
    int HashFunction(const CfgType& _cfg, const hash_parameters& _h) const;

    /// Compute a complete hash key for a configuration.
    /// @param _cfg The configuration.
    /// @param _h The hash family to use.
    /// @return A complete hash key for _cfg using _h.
    HashKey HashFamilyFunction(const CfgType& _cfg, const HashFamily& _h) const;

    /// Add a new node to the hash maps.
    /// @param _vi An iterator to the new node in the roadmap graph.
    void InsertIntoMap(const VI _vi);

    ///@}
    ///@name Internal State
    ///@{

    double m_hashRadius;      ///< Sort-of-bucket radius in c-space.
    size_t m_hashFamilyCount; ///< The number of hash families.
    size_t m_hashDimension;   ///< The number of hash functions in a family.

    std::vector<HashFamily> m_hashFamilies; ///< The set of hash families.
    std::vector<BucketMap> m_hashMaps;      ///< The set of hash maps.

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
LSHNF<MPTraits>::
LSHNF() : NeighborhoodFinderMethod<MPTraits>() {
  this->SetName("LSHNF");
  this->m_nfType = Type::K;
}


template <typename MPTraits>
LSHNF<MPTraits>::
LSHNF(XMLNode& _node) : NeighborhoodFinderMethod<MPTraits>(_node) {
  this->SetName("LSHNF");
  this->m_nfType = Type::K;
  this->m_k = _node.Read("k", true, 5, 0, MAX_INT, "Number of neighbors to find");

  m_hashDimension = _node.Read("hashDimension", true, 1, 1, MAX_INT,
      "Dimension of hash space.");

  m_hashFamilyCount = _node.Read("hashFamilies", true,
      size_t(1), size_t(1), std::numeric_limits<size_t>::max(),
      "Number of hash families.");

  m_hashRadius = _node.Read("hashRadius", true, 0.0,
      std::numeric_limits<double>::min(), std::numeric_limits<double>::max(),
      "Radius of hash buckets in configuration space.");
}


template <typename MPTraits>
LSHNF<MPTraits>::
~LSHNF() = default;

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
LSHNF<MPTraits>::
Initialize() {
  /*
  auto boundary = this->GetEnvironment()->GetBoundary();
  auto mb = this->GetTask()->GetRobot()->GetMultiBody();

  // Getting robot's base radius
  double radius = mb->GetBase()->GetBoundingSphereRadius();
  double hyper = 1;//to compute hypervolume of the environment
  size_t dim = boundary->GetDimension();

  //Computing hypervolume Venv
  for (size_t i = 0 ; i  < dim ; ++i ) {
    hyper = hyper * (boundary->GetRange(i).max - boundary->GetRange(i).min);
  }
  size_t targetSize = ceil(hyper / ( pow(radius,dim)) );//Ratio Venv/Vrobot

  // Next values for l,k and r were obtained in practice, for now they are commented
  // they could be useful in the future
  m_hashFamilyCount = ceil(2 * log2(targetSize));
  m_hashRadius = ceil( pow((log10(targetSize) - 1.5),2)+1);
  m_hashDimesion = 2 * mb->PosDOF();

  //If  the base is fixed we multiply the joint angles 10x
  if(mb->PosDOF() == 0) {
    m_hashDimension = 5;
  }
  */

  // Create the hash families and initialize empty maps.
  m_hashMaps.resize(m_hashFamilyCount);
  m_hashFamilies.resize(m_hashFamilyCount);
  for(auto& family : m_hashFamilies)
    family = MakeHashFamily();

  // Each time we add a new vertex, compute its mapping in each hash map.
  auto g = this->GetRoadmap();
  g->InstallHook(RoadmapType::HookType::AddVertex, this->GetNameAndLabel(),
      [this](const VI _vi){this->InsertIntoMap(_vi);});

  /// @todo Create a hook for removing the mapping.
}


template <typename MPTraits>
void
LSHNF<MPTraits>::
Print(std::ostream& _os) const {
  NeighborhoodFinderMethod<MPTraits>::Print(_os);
  _os << "\thash space dimension: " << m_hashDimension
      << "\tnumber of hash families: " << m_hashFamilyCount
      << "\tbucket radius: " << m_hashRadius
      << std::endl;
}

/*-------------------- NeighborhoodFinderMethod Interface--------------------*/

template <typename MPTraits>
template <typename InputIterator>
void
LSHNF<MPTraits>::
FindNeighbors(RoadmapType* _r,
    InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
    const CfgType& _query, OutputIterator _out) {
  auto g = _r;
  auto dm = this->GetDistanceMetric(this->m_dmLabel);

  // Find the first m_k approximate nearest-neighbors from the hash map.
  std::set<Neighbor> neighbors;

  try {
    for(size_t i = 0 ; i < m_hashMaps.size() ; ++i) {
      // Compute the hash key for _query using this hash function.
      const HashKey key = HashFamilyFunction(_query, m_hashFamilies.at(i));

      // Look for configurations in the corresponding hash map which shares this
      // key.
      auto range = m_hashMaps.at(i).equal_range(key);
      for(auto it = range.first; it != range.second; ++it) {
        const VID vid = it->second;

        // Skip connections to self.
        const CfgType& cfg = g->GetVertex(vid);
        if(_query == cfg)
          continue;

        // Check distance. If it is infinite, these are not connectable.
        const double distance = dm->Distance(_query, cfg);
        if(std::isinf(distance))
          continue;

        // Add this configuration to the neighbor list.
        neighbors.emplace(vid, distance);
        if(neighbors.size() >= this->m_k)
          break;
      }
    }
  }
  catch(const std::runtime_error& _e) {
    // If we had an out-of-range access, re-propogate the error with WHERE info.
    throw RunTimeException(WHERE) << _e.what();
  }

  if(this->m_debug)
    std::cout << "Found " << neighbors.size() << " neighbors."
              << std::endl;

  std::copy(neighbors.begin(), neighbors.end(), _out);
}


template <typename MPTraits>
template <typename InputIterator>
void
LSHNF<MPTraits>::
FindNeighbors(GroupRoadmapType* _r,
    InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
    const GroupCfgType& _cfg, OutputIterator _out) {
  throw NotImplementedException(WHERE);
}

/*------------------------------ Helper Functions ----------------------------*/

template <typename MPTraits>
std::vector<typename std::pair<std::vector<double>,double>>
LSHNF<MPTraits>::
MakeHashFamily() const {
  double b;
  std::vector<double> a;
  std::vector<hash_parameters> family;

  const size_t dof = this->GetTask()->GetRobot()->GetMultiBody()->DOF();

  for(size_t i = 0 ; i < m_hashDimension ; ++i) {
    // Compute m_hashDimension gaussian-random values for the a vector.
    a.clear();
    for(size_t j = 0 ; j < dof ; ++j)
      a.push_back(GRand());

    // The b scalar is a random fraction of m_hashRadius.
    b = DRand() * m_hashRadius;

    family.emplace_back(a, b);
  }

  return family;
}


template <typename MPTraits>
int
LSHNF<MPTraits>::
HashFunction(const CfgType& _cfg, const hash_parameters& _h) const {
  const auto& _a = _h.first;
  const auto& _b = _h.second;
  const double dot = std::inner_product(_a.begin(), _a.end(),
                                        _cfg.GetData().begin(), 0);

  return std::floor((dot + _b) / m_hashRadius);
}


template <typename MPTraits>
typename std::vector<int>
LSHNF<MPTraits>::
HashFamilyFunction(const CfgType& _cfg, const HashFamily& _h) const {
  HashKey output;
  for(const auto& parameters : _h)
    output.push_back(HashFunction(_cfg, parameters));
  return output;
}


template <typename MPTraits>
void
LSHNF<MPTraits>::
InsertIntoMap(const VI _vi) {
  MethodTimer(this->GetStatClass(), "LSHNF::InsertIntoMap");

  const VID vid = _vi->descriptor();
  const CfgType& _cfg = this->GetRoadmap()->GetVertex(vid);

  // Add this cfg to each hash map.
  for(unsigned int i = 0; i < m_hashMaps.size(); ++i) {
    const HashKey hash = HashFamilyFunction(_cfg, m_hashFamilies[i]);
    m_hashMaps[i].emplace(hash, vid);
  }
}


/// Define an ordering operation on hash keys so that we can use them as a key in
/// a multimap.
inline
bool
operator<(const std::vector<int>& _h1, const std::vector<int>& _h2) {
  const size_t numElements = std::min(_h1.size(), _h2.size());
  for(size_t i = 0; i < numElements; ++i) {
    if(_h1[i] < _h2[i])
      return true;
    else if(_h2[i] < _h1[i])
      return false;
  }
  return false;
}

#endif

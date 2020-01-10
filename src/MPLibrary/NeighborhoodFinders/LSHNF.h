#ifndef PMPL_LSHNF_H_
#define PMPL_LSHNF_H_

#include "BruteForceNF.h"

#include <map>
#include <utility>
#include <vector>


////////////////////////////////////////////////////////////////////////////////
/// This method implements a locality sensitive hashing-based approximate
/// neigborhood finder. It is adapted from the paper for k-nearest neighbors
/// rather than
///
/// Reference:
///   Mayur Datar and Nicloe Immorlica and Piotr Indyk and Vahab S. Mirronki.
///   "Locality-sensitive Hashing Scheme Based on P-stable Distributions".
///   Proceedings of the Twentieth Annual Symposium on Computational Geometry,
///   2004.
///
/// @todo Fix hashmaps and hash families so that we keep one per roadmap.
///       Currently this will give incorrect results if we use more than one
///       roadmap.
///
/// @note A major weakness with this method is the non-obvious tuning of three
///       parameters. As of yet there is no intelligible advice available; the
///       paper claims some process is available and then resorts to numeric
///       estimation and trial-and-error. @todo Come up with a way to set the
///       parameters automatically. For now they are XML parameters that would
///       have to be tuned for each problem with a 3-dimensional grid search.
///
/// @ingroup NeighborhoodFinders
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class LSHNF : public BruteForceNF<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType          CfgType;
    typedef typename MPTraits::RoadmapType      RoadmapType;
    typedef typename RoadmapType::VI            VI;
    typedef typename RoadmapType::VID           VID;
    typedef typename RoadmapType::VertexSet     VertexSet;
    typedef typename MPTraits::GroupRoadmapType GroupRoadmapType;
    typedef typename MPTraits::GroupCfgType     GroupCfgType;

    ///@}
    ///@name Local Types
    ///@{

    using typename NeighborhoodFinderMethod<MPTraits>::Type;
    using typename NeighborhoodFinderMethod<MPTraits>::OutputIterator;

    ///@}

  private:

    ///@name Internal Types
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// A locality-sensitive hash functor generated with gaussian-distributed
    /// 'a' vector (see paper).
    ////////////////////////////////////////////////////////////////////////////
    struct HashFunction {

      /// Constructor.
      HashFunction(const size_t _cSpaceDimension, const double _radius);

      /// Hash function.
      int operator()(const CfgType& _query) const noexcept;

      private:

        ///@name Internal State
        ///@{

        std::vector<double> m_a;  ///< The 'a' vector.
        double m_b;               ///< The 'b' constant.
        double m_radius;          ///< The projection radius 'r'.

        ///@}

    };

    ////////////////////////////////////////////////////////////////////////////
    /// A set of hash functions which together map configurations from c-space
    /// to a hash space with possibly different dimension.
    ////////////////////////////////////////////////////////////////////////////
    struct HashFamily {

      typedef std::vector<int> HashKey;

      /// Constructor.
      HashFamily(const size_t _hashSpaceDimension,
          const size_t _cSpaceDimension, const double _radius);

      /// Hash function.
      HashKey operator()(const CfgType& _query) const noexcept;

      private:

        ///@name Internal State
        ///@{

        std::vector<HashFunction> m_hashFunctions; ///< The set of hashers.

        ///@}

    };

    /// The hash key (or bucket) for a hash family.
    typedef typename HashFamily::HashKey HashKey;

    ////////////////////////////////////////////////////////////////////////////
    /// A mapping kit for a particular roadmap.
    ////////////////////////////////////////////////////////////////////////////
    struct LSHMap {

      /// Constructor.
      LSHMap(LSHNF<MPTraits>* const _lshNF, RoadmapType* const _r);

      /// Locate the set of candidate neighbors for a given query Cfg.
      VertexSet operator()(const CfgType& _query) const noexcept;

      private:

        /// Add a new node to the hash maps.
        /// @param _vi An iterator to the new node in the roadmap graph.
        void MapCfg(const VI _vi);

        /// Remove an existing node from the hash maps.
        /// @param _vi An iterator to the existing node in the roadmap graph.
        void UnmapCfg(const VI _vi);

        /// Maps a hash key to a set of VIDs.
        typedef std::map<HashKey, VertexSet> BucketMap;

        ///@name Internal State
        ///@{

        StatClass* const m_stats;               ///< Stat class for timing.
        const std::string m_id;                 ///< Timing label.
        std::vector<HashFamily> m_hashFamilies; ///< The set of hash families.
        std::vector<BucketMap> m_hashMaps;      ///< The set of hash maps.
        const size_t m_candidateLimit;          ///< Max returned candidates.

        ///@}

    };

    ///@}

  public:

    ///@name Construction
    ///@{

    LSHNF();

    LSHNF(XMLNode& _node);

    virtual ~LSHNF() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Initialize() override;

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name NeighborhoodFinder Overrides
    ///@{

    virtual void FindNeighbors(RoadmapType* const _r, const CfgType& _cfg,
        const VertexSet& _candidates, OutputIterator _out) override;

    virtual void FindNeighbors(GroupRoadmapType* const _r,
        const GroupCfgType& _cfg, const VertexSet& _candidates,
        OutputIterator _out) override;

    ///@}

  private:

    ///@name Helper Functions
    ///@{

    const LSHMap& GetLSHMap(RoadmapType* const _r);

    ///@}
    ///@name Internal State
    ///@{

    double m_hashRadius{4};       ///< Sort-of-bucket radius in hash space.
    size_t m_hashFamilyCount{30}; ///< The number of hash families.
    size_t m_hashDimension{10};   ///< The number of hash functions in a family.
    /// Skip to distance checks after finding k*this candidates (0 to disable).
    double m_kMultiple{0};

    /// LSH maps for each roadmap.
    std::unordered_map<RoadmapType*, LSHMap> m_lshMaps;

    ///@}

    friend LSHMap; ///< The maps need a lot of this class's data.

};

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ HashFunction ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

template <typename MPTraits>
LSHNF<MPTraits>::HashFunction::
HashFunction(const size_t _cSpaceDimension, const double _radius) :
    m_radius(_radius) {
  // Compute _cSpaceDimension gaussian-random values for the a vector.
  m_a.reserve(_cSpaceDimension);
  for(size_t i = 0 ; i < _cSpaceDimension ; ++i)
    m_a.push_back(GRand());

  // The b scalar is a random fraction of _radius.
  m_b = DRand() * m_radius;
}


template <typename MPTraits>
int
LSHNF<MPTraits>::HashFunction::
operator()(const typename MPTraits::CfgType& _query) const noexcept {
  const double dot = std::inner_product(m_a.begin(), m_a.end(),
                                        _query.GetData().begin(), 0);

  return std::floor((dot + m_b) / m_radius);
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ HashFamily ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

template <typename MPTraits>
LSHNF<MPTraits>::HashFamily::
HashFamily(const size_t _hashSpaceDimension, const size_t _cSpaceDimension,
    const double _radius) {
  for(size_t i = 0; i < _hashSpaceDimension; ++i)
    m_hashFunctions.emplace_back(_cSpaceDimension, _radius);
}


template <typename MPTraits>
typename LSHNF<MPTraits>::HashFamily::HashKey
LSHNF<MPTraits>::HashFamily::
operator()(const typename MPTraits::CfgType& _query) const noexcept {
  HashKey out;
  out.reserve(m_hashFunctions.size());
  for(const auto& h : m_hashFunctions)
    out.push_back(h(_query));
  return out;
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ LSHMap ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

template <typename MPTraits>
LSHNF<MPTraits>::LSHMap::
LSHMap(LSHNF<MPTraits>* const _lshNF, typename MPTraits::RoadmapType* const _r)
    : m_stats(_lshNF->GetStatClass())
    , m_id(_lshNF->GetNameAndLabel() + "::Hashing")
    , m_candidateLimit(_lshNF->GetK() * _lshNF->m_kMultiple) {
  const size_t dof         = _r->GetRobot()->GetMultiBody()->DOF(),
               familyCount = _lshNF->m_hashFamilyCount,
               dimension   = _lshNF->m_hashDimension;
  const double radius      = _lshNF->m_hashRadius;

  // Create the hash families and initialize empty maps.
  m_hashMaps.resize(familyCount);
  m_hashFamilies.reserve(familyCount);
  for(size_t i = 0; i < familyCount; ++i)
    m_hashFamilies.emplace_back(dimension, dof, radius);

  // Add each configuration in the roadmap to the maps.
  for(auto vi = _r->begin(); vi != _r->end(); ++vi)
    MapCfg(vi);

  // Each time we add a new vertex, compute its mapping in each hash map.
  _r->InstallHook(RoadmapType::HookType::AddVertex, m_id,
      [this](const VI _vi){this->MapCfg(_vi);});
  // Each time we delete a vertex, remove it from the mappings.
  _r->InstallHook(RoadmapType::HookType::DeleteVertex, m_id,
      [this](const VI _vi){this->UnmapCfg(_vi);});
}


template <typename MPTraits>
typename MPTraits::RoadmapType::VertexSet
LSHNF<MPTraits>::LSHMap::
operator()(const typename MPTraits::CfgType& _query) const noexcept {
  // Collect the set of candidates from each hash map.
  VertexSet candidates;
  for(size_t i = 0; i < m_hashMaps.size(); ++i) {
    // Compute the hash key for the _query using this hash function.
    const HashKey key = m_hashFamilies[i](_query);

    // Look for configurations in the corresponding hash map which shares this
    // key.
    const BucketMap& map = m_hashMaps[i];
    const VertexSet& mapped = map.at(key);

    // Add the mapped configurations to the candidate set.
    VertexSetUnionInPlace(candidates, mapped);

    if(m_candidateLimit and candidates.size() > m_candidateLimit)
      break;
  }

  return candidates;
}


template <typename MPTraits>
void
LSHNF<MPTraits>::LSHMap::
MapCfg(const typename MPTraits::RoadmapType::VI _vi) {
  MethodTimer(m_stats, m_id);

  const VID vid = _vi->descriptor();
  const CfgType& cfg = _vi->property();

  // Add this cfg to each hash map.
  for(size_t i = 0; i < m_hashMaps.size(); ++i) {
    const HashKey hash = m_hashFamilies[i](cfg);
    m_hashMaps[i][hash].insert(vid);
  }
}


template <typename MPTraits>
void
LSHNF<MPTraits>::LSHMap::
UnmapCfg(const typename MPTraits::RoadmapType::VI _vi) {
  MethodTimer(m_stats, m_id);

  const VID vid = _vi->descriptor();
  const CfgType& cfg = _vi->property();

  // Add this cfg to each hash map.
  for(size_t i = 0; i < m_hashMaps.size(); ++i) {
    const HashKey hash = m_hashFamilies[i](cfg);
    m_hashMaps[i][hash].erase(vid);
  }
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ LSHNF ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
LSHNF<MPTraits>::
LSHNF() : BruteForceNF<MPTraits>() {
  this->SetName("LSHNF");
}


template <typename MPTraits>
LSHNF<MPTraits>::
LSHNF(XMLNode& _node) : BruteForceNF<MPTraits>(_node) {
  this->SetName("LSHNF");

  m_hashDimension = _node.Read("hashDimension", false, m_hashDimension,
      size_t(1), std::numeric_limits<size_t>::max(),
      "Dimension of hash space.");

  m_hashFamilyCount = _node.Read("hashFamilies", false,
      m_hashFamilyCount, size_t(1), std::numeric_limits<size_t>::max(),
      "Number of hash families.");

  m_hashRadius = _node.Read("hashRadius", false, m_hashRadius,
      std::numeric_limits<double>::min(), std::numeric_limits<double>::max(),
      "Projection width in hash space.");

  m_kMultiple = _node.Read("kMultiple", false, m_kMultiple,
      1., std::numeric_limits<double>::max(),
      "Stop searching for hash-mapped candidates after finding k*this many.");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
LSHNF<MPTraits>::
Initialize() {
  m_lshMaps.clear();
}


template <typename MPTraits>
void
LSHNF<MPTraits>::
Print(std::ostream& _os) const {
  BruteForceNF<MPTraits>::Print(_os);
  _os << "\thash space dimension: " << m_hashDimension
      << "\tnumber of hash families: " << m_hashFamilyCount
      << "\tbucket radius: " << m_hashRadius
      << std::endl;
}

/*-------------------- NeighborhoodFinderMethod Overrides --------------------*/

template <typename MPTraits>
void
LSHNF<MPTraits>::
FindNeighbors(RoadmapType* const _r, const CfgType& _cfg,
    const VertexSet& _candidates, OutputIterator _out) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::FindNeighbors");

  auto lshMap = GetLSHMap(_r);

  // Intersect with the candidate set.
  const VertexSet mapped     = lshMap(_cfg),
                  candidates = VertexSetIntersection(_candidates, mapped);
  if(this->m_debug)
    std::cout << "LSH map:"
              << "\n\tinput candidates:  " << _candidates.size()
              << "\n\tmapped:            " << mapped.size()
              << "\n\tcommon candidates: " << candidates.size()
              << std::endl;

  // Find the closest m_k (approximate) nearest-neighbors from the candidates.
  this->FindNeighborsImpl(_r, _cfg, candidates, _out);
}


template <typename MPTraits>
void
LSHNF<MPTraits>::
FindNeighbors(GroupRoadmapType* const _r, const GroupCfgType& _cfg,
    const VertexSet& _candidates, OutputIterator _out) {
  throw NotImplementedException(WHERE);
}

/*------------------------------ Helper Functions ----------------------------*/

template <typename MPTraits>
const typename LSHNF<MPTraits>::LSHMap&
LSHNF<MPTraits>::
GetLSHMap(RoadmapType* const _r) {
  auto iter = m_lshMaps.find(_r);
  if(iter != m_lshMaps.end())
    return iter->second;

  auto iterBool = m_lshMaps.emplace(std::piecewise_construct,
      std::forward_as_tuple(_r), std::forward_as_tuple(this, _r));
  iter = iterBool.first;
  return iter->second;
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

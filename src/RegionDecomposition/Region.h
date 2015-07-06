#ifndef REGION_H_
#define REGION_H_

#include "Environment/BoundingBox.h"
#include "MPProblem/Roadmap.h"


////////////////////////////////////////////////////////////////////////////////
/// @ingroup RegionDecomposition
/// @brief Region information.
/// @tparam MPTraits Motion planning universe
///
/// Stores boundary and roadmap information for a Region.
////////////////////////////////////////////////////////////////////////////////
template<class BOUNDARY, class MPTraits>
class Region {
  public:

    typedef typename MPTraits::MPProblemType::VID VID;
    typedef BOUNDARY BoundaryType;
    /////////////////////////
    //Constructors
    /////////////////////////
    Region();
    Region(const Region& _r);
    //second input: vector of pair (cc size, vid of cc representative)
    Region(shared_ptr<BOUNDARY> _bbox, std::vector<pair<size_t, VID> >& _pairIds);
    Region(shared_ptr<BOUNDARY> _bbox, std::vector<VID>& _ids);
    ~Region();

    //////////////////////////
    //Accessors
    //////////////////////////
    shared_ptr<BOUNDARY> GetBoundary() const;
    void SetBoundary(shared_ptr<BOUNDARY> _bb);

    pair<shared_ptr<BOUNDARY>, vector<pair<size_t, VID> > > GetRegionInfo() const;

    //returns all the vids contained in the region
    std::vector<VID> GetAllVIDs() const;
    //returns the vid representatives from all the CCs within the region
    std::vector<VID> GetCCVIDs() const;
    //returns the vid of cc representative and size of cc for all CCs in the
    //region
    std::vector<pair<VID, size_t> > GetCCs() const;
    void SetCCs(std::vector<pair<VID, size_t> > _ccs);

    std::vector<VID> RegionVIDs() const;
    void SetVIDs(std::vector<VID> _ids);

    //number of valid nodes in a region, needed for region migration
    size_t RegionWeight() const;

    void Print(ostream& _os) const ;

    bool operator==(const Region& _a) const;
    // friend ostream& operator<<(ostream&, const Region<BOUNDARY>&);

  protected:

    shared_ptr<BOUNDARY> m_bb;
    vector<pair<VID, size_t> > m_ccs;//vector of pair (vid of cc representative,size)
    vector<VID> m_vids;

  public:
#ifdef _PARALLEL
    void define_type(stapl::typer& _t){
      _t.member(m_bb);
      _t.member(m_vids);
      _t.member(m_ccs);
    }
#endif


};

#ifdef _PARALLEL
namespace stapl {

  template <typename BoundaryType, class MPTraits, typename Accessor>
    class proxy<Region<BoundaryType, MPTraits>, Accessor>
    : public Accessor {
      private:
        friend class proxy_core_access;
        typedef Region<BoundaryType, MPTraits> m_targetT;
        typedef typename MPTraits::MPProblemType::VID VID;

      public:
        explicit proxy(Accessor const& _acc) : Accessor(_acc) { }
        operator m_targetT() const { return Accessor::read(); }
        proxy const& operator=(proxy const& _rhs) { Accessor::write(_rhs); return *this; }
        proxy const& operator=(m_targetT const& _rhs) { Accessor::write(_rhs); return *this;}
//        void Print(ostream& _os) const { return Accessor::const_invoke(&m_targetT::Print, _os);}
        std::vector<VID> RegionVIDs() const{ return Accessor::const_invoke(&m_targetT::RegionVIDs);}
        size_t RegionWeight() const{ return Accessor::const_invoke(&m_targetT::RegionWeight);}
        shared_ptr<BoundaryType> GetBoundary() const { return Accessor::const_invoke(&m_targetT::GetBoundary);}
//        typename Region<BoundaryType>::ostream& operator<< (ostream& _os, const Region<BoundaryType>& _r) { return Accessor::invoke(&m_targetT::
        void SetVIDs(std::vector<VID> _ids) { Accessor::invoke(&m_targetT::SetVIDs, _ids); }
        void SetCCs(std::vector<pair<VID, size_t> > _ccs) { Accessor::invoke(&m_targetT::SetCCs, _ccs); }
        vector<pair<VID, size_t> > GetCCs() const{ return Accessor::const_invoke(&m_targetT::GetCCs);}
    }; //struct proxy
}
#endif


/////////////////////////
//Constructors
////////////////////////
template<class BOUNDARY, class MPTraits>
Region<BOUNDARY, MPTraits>::Region(){ }

template<class BOUNDARY, class MPTraits>
Region<BOUNDARY, MPTraits>::~Region(){ }

template<class BOUNDARY, class MPTraits>
Region<BOUNDARY, MPTraits>::Region(const Region& _r){
  m_bb=_r.m_bb;
  m_ccs=_r.m_ccs;
  m_vids = _r.m_vids;
}

template<class BOUNDARY, class MPTraits>
Region<BOUNDARY, MPTraits>::Region(shared_ptr<BOUNDARY> _bbox, std::vector<pair<size_t, VID> >& _pairIds ){
  m_bb=_bbox;
  m_ccs.clear();
  m_ccs=_pairIds;
}

template<class BOUNDARY, class MPTraits>
Region<BOUNDARY, MPTraits>::Region(shared_ptr<BOUNDARY> _bbox, std::vector<VID>& _ids ){
  m_bb=_bbox;
  m_vids.clear();
  typedef typename  std::vector<VID>::iterator itr;
  for(itr vit = _ids.begin(); vit!=_ids.end(); vit++){
    m_vids.push_back(*vit);
  }
}

template<class BOUNDARY, class MPTraits>
shared_ptr<BOUNDARY>
Region<BOUNDARY, MPTraits>::GetBoundary() const{
  return m_bb;
}

template<class BOUNDARY, class MPTraits>
std::vector<pair<typename MPTraits::MPProblemType::VID, size_t> >
Region<BOUNDARY, MPTraits>::GetCCs() const{
  return m_ccs;
}

template<class BOUNDARY, class MPTraits>
pair<shared_ptr<BOUNDARY>, vector<pair<size_t, typename MPTraits::MPProblemType::VID> > >
Region<BOUNDARY, MPTraits>::GetRegionInfo() const{
  pair<shared_ptr<BOUNDARY>, vector<pair<size_t, VID> > > infoPair = make_pair(m_bb, m_ccs);
  return infoPair;
}

template<class BOUNDARY, class MPTraits>
std::vector<typename MPTraits::MPProblemType::VID>
Region<BOUNDARY, MPTraits>::RegionVIDs() const {
  return m_vids;
}

template<class BOUNDARY, class MPTraits>
void
Region<BOUNDARY, MPTraits>::SetVIDs(std::vector<VID> _ids) {
  typename std::vector<VID>::iterator itr;
  for (itr = _ids.begin(); itr != _ids.end(); ++itr) {
    m_vids.push_back(*itr) ;
  }
}

template<class BOUNDARY, class MPTraits>
size_t
Region<BOUNDARY, MPTraits>::RegionWeight() const {
  return m_vids.size();
}

template<class BOUNDARY, class MPTraits>
void
Region<BOUNDARY, MPTraits>::SetBoundary(shared_ptr<BOUNDARY> _bb){
  m_bb=_bb;
}

///This doesn't make much sense, Shuvra needs to look into it.
template<class BOUNDARY, class MPTraits>
std::vector<typename MPTraits::MPProblemType::VID>
Region<BOUNDARY, MPTraits>::GetAllVIDs() const{
  vector<VID> allVIDs;
  return allVIDs;
}

template<class BOUNDARY, class MPTraits>
bool
Region<BOUNDARY, MPTraits>::operator==(const Region& _a) const{
  return (m_bb == _a.m_bb) &&
    (m_ccs == _a.m_ccs) &&
    (m_vids == _a.m_vids);
}

/*
template<class BOUNDARY>
ostream&
operator<< (ostream& _os, const Region<BOUNDARY>& _r) {
  _r.Print(_os);
  return _os;
}
*/
template<class BOUNDARY, class MPTraits>
void
Region<BOUNDARY,MPTraits>::SetCCs(std::vector<pair<typename MPTraits::MPProblemType::VID, size_t> > _ccs){
  m_ccs=_ccs;
}


template<class BOUNDARY, class MPTraits>
void
Region<BOUNDARY, MPTraits>::Print(ostream& _os) const  {
  typedef typename std::vector<VID>::const_iterator itr;
  _os << "  " << "Region Weight :" << m_vids.size() << " ";
  _os << "Region VIDs: ";
  for(itr vit = m_vids.begin(); vit!=m_vids.end(); vit++){
    _os << "  " << *vit<< "";
  }
}

#endif

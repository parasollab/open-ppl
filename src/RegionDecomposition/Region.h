#ifndef _REGION_H
#define _REGION_H
#include "Roadmap.h"
#include "CfgTypes.h"
#include "Boundary.h"

//region class will save the associated bounding volume and the CC information in the region
typedef RoadmapGraph<CfgType, WeightType>::VID RVID;

class Region {
 public:
 
 
  Region();
  
  Region(const Region& _r);
  Region(shared_ptr<BoundingBox> _bbox, std::vector<pair<size_t, RVID> >& _pairIds ); //second input: vector of pair (cc size, vid of cc representative)
  Region(shared_ptr<BoundingBox> _bbox, std::vector<RVID> &_ids ); 
    
  ~Region();
  shared_ptr<BoundingBox> GetBoundingBox() const;
  
  void SetBoundingBox(shared_ptr<BoundingBox> _bb);
  
  pair <shared_ptr<BoundingBox>, vector<pair<size_t, RVID> > > GetRegionInfo() const;
 
  bool operator==(const Region &_a) const;
  
  std::vector<RVID> GetAllVIDs() const; //returns all the vids from all the CCs of the region
  
  friend ostream& operator<< (ostream&, const Region&);
  
  std::vector<RVID> GetCCVIDs() const; //returns all the vids from all the CCs of the region

  std::vector<pair<RVID, size_t> > GetCCs() const; //returns the vector of pair (vid of cc representative, size of cc)
  
  void SetCCs(std::vector<pair<RVID, size_t> > _ccs);
  
  std::vector<RVID> RegionVIDs() const;
  
  void SetVIDs(std::vector<RVID> &_ids);
  
  size_t RegionWeight() const; // number of valid nodes in a region, needed for region migration
  
  void Print(ostream &_os) const ;
  
  protected:
     
  shared_ptr<BoundingBox> m_bb;
  vector<pair<RVID, size_t> > m_ccs;//vector of pair (cc size, vid of cc representative)
  vector<RVID> m_vids;
  
 public:
   #ifdef _PARALLEL
      void define_type(stapl::typer &t){
        t.member(m_bb);
        t.member(m_vids);
        t.member(m_ccs);
      }
   #endif
   

};
#ifdef _PARALLEL
namespace stapl {
template <typename Accessor>
class proxy<Region, Accessor> 
: public Accessor {
private:
  friend class proxy_core_access;
  typedef Region target_t;
  
public:
  explicit proxy(Accessor const& acc) : Accessor(acc) { }
  operator target_t() const { return Accessor::read(); }
  proxy const& operator=(proxy const& rhs) { Accessor::write(rhs); return *this; }
  proxy const& operator=(target_t const& rhs) { Accessor::write(rhs); return *this;}
  void Print(ostream& _os) const { return Accessor::const_invoke(&target_t::Print, _os);}
  std::vector<RVID> RegionVIDs() const{ return Accessor::const_invoke(&target_t::RegionVIDs);}
  size_t RegionWeight() const{ return Accessor::const_invoke(&target_t::RegionWeight);}
  shared_ptr<BoundingBox> GetBoundingBox() const { return Accessor::const_invoke(&target_t::GetBoundingBox);}
  void SetVIDs(std::vector<RVID> &_ids) { Accessor::invoke(&target_t::SetVIDs, _ids); }
  void SetCCs(std::vector<pair<RVID, size_t> > _ccs) { Accessor::invoke(&target_t::SetCCs, _ccs); }
   vector<pair<RVID, size_t> > GetCCs() const{ return Accessor::const_invoke(&target_t::GetCCs);}
}; //struct proxy
}
#endif


#endif 

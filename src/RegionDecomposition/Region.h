//////////////////////////////////////////////////////////////////////////////////////////
//  class Region
//
//  region class will save the associated bounding volume and the CC information in the region
//////////////////////////////////////////////////////////////////////////////////////////

#ifndef REGION_H_
#define REGION_H_

#include "Roadmap.h"
#include "CfgTypes.h"
#include "Boundary.h"

typedef RoadmapGraph<CfgType, WeightType>::VID VID;

class Region {
  public:

    /////////////////////////
    //Constructors
    /////////////////////////
    Region();
    Region(const Region& _r);
    //second input: vector of pair (cc size, vid of cc representative)
    Region(shared_ptr<BoundingBox> _bbox, std::vector<pair<size_t, VID> >& _pairIds); 
    Region(shared_ptr<BoundingBox> _bbox, std::vector<VID>& _ids); 
    ~Region();
    
    //////////////////////////
    //Accessors
    //////////////////////////
    shared_ptr<BoundingBox> GetBoundingBox() const;
    void SetBoundingBox(shared_ptr<BoundingBox> _bb);

    pair<shared_ptr<BoundingBox>, vector<pair<size_t, VID> > > GetRegionInfo() const;
    
    //returns all the vids contained in the region
    std::vector<VID> GetAllVIDs() const;
    //returns the vid representatives from all the CCs within the region
    std::vector<VID> GetCCVIDs() const; 
    //returns the vid of cc representative and size of cc for all CCs in the
    //region
    std::vector<pair<VID, size_t> >& GetCCs() const; 
    void SetCCs(std::vector<pair<VID, size_t> >& _ccs);

    std::vector<VID>& RegionVIDs() const;
    void SetVIDs(std::vector<VID>& _ids);
    
    //number of valid nodes in a region, needed for region migration
    size_t RegionWeight() const; 

    void Print(ostream& _os) const ;

    bool operator==(const Region& _a) const;
    friend ostream& operator<<(ostream&, const Region&);

  protected:

    shared_ptr<BoundingBox> m_bb;
    vector<pair<VID, size_t> > m_ccs;//vector of pair (vid of cc representative,size)
    vector<VID> m_vids;

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
        typedef Region m_targetT;

      public:
        explicit proxy(Accessor const& _acc) : Accessor(_acc) { }
        operator m_targetT() const { return Accessor::read(); }
        proxy const& operator=(proxy const& _rhs) { Accessor::write(_rhs); return *this; }
        proxy const& operator=(m_targetT const& _rhs) { Accessor::write(_rhs); return *this;}
        void Print(ostream& _os) const { return Accessor::const_invoke(&m_targetT::Print, _os);}
        std::vector<VID> RegionVIDs() const{ return Accessor::const_invoke(&m_targetT::RegionVIDs);}
        size_t RegionWeight() const{ return Accessor::const_invoke(&m_targetT::RegionWeight);}
        shared_ptr<BoundingBox> GetBoundingBox() const { return Accessor::const_invoke(&m_targetT::GetBoundingBox);}
        void SetVIDs(std::vector<VID>& _ids) { Accessor::invoke(&m_targetT::SetVIDs, _ids); }
        void SetCCs(std::vector<pair<VID, size_t> > _ccs) { Accessor::invoke(&m_targetT::SetCCs, _ccs); }
        vector<pair<VID, size_t> > GetCCs() const{ return Accessor::const_invoke(&m_targetT::GetCCs);}
    }; //struct proxy
}
#endif


#endif 

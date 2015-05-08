#ifndef SRTINFO_H_
#define SRTINFO_H_

#include "MPUtils.h"
#include "MetricUtils.h"
#include <utility>
#include "RegionGraph.h"
#include "Boundary.h"
#include "CfgTypes.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
class SRTInfo {
 public:
  typedef typename RoadmapGraph<CfgType,WeightType>::vertex_descriptor VID;
  SRTInfo(BoundingBox &m_bbox, std::pair<CfgType,std::vector<VID> > &m_ids );
  SRTInfo(const SRTInfo &input_srtinfo);
  SRTInfo();
  virtual ~SRTInfo();

  bool operator==(const SRTInfo& bb) const;
  friend ostream& operator<< (ostream&, const SRTInfo&);
  BoundingBox GetBoundingBox() const;
  std::vector<VID> GetVIDs() const;
  CfgType GetCandidate() const;
  std::pair<CfgType,std::vector<VID> > GetTree() const;
  void SetTree(std::pair<CfgType,std::vector<VID> > &ids);
  void SetBoundingBox(BoundingBox &bbox);

  /*void connectRegion(shared_ptr<ComponentConnectionMethod<CfgType,WeightType> > pCCon,
    Roadmap<CfgType,WeighType>* rm, BoundingBox& bb, Stat_Class& Stats,
    LocalPlanners<CfgType,WeightType>* lp,
    bool addPartialEdge, bool addAllEdges,
    vector<VID>& cc1);*/

  const pair<BoundingBox, pair<CfgType,vector<VID> > > GetData() const;
  void SetData(BoundingBox &m_bbox, std::pair<CfgType,std::vector<VID> > &m_ids);
  void Print(ostream &os) const ;

 protected:
  std::pair<BoundingBox, std::pair<CfgType,std::vector<VID> > > srt_info;
  BoundingBox bb;
  pair<CfgType,vector<VID> > tree;

 public:
#ifdef _PARALLEL
  void define_type(stapl::typer &t) {
    t.member(srt_info);
    t.member(tree);
    t.member(bb);
  }
#endif
};

namespace stapl {
  template <typename Accessor>
    class proxy<SRTInfo, Accessor>
    : public Accessor {
  private:
    friend class proxy_core_access;
    typedef SRTInfo target_t;

  public:
   typedef typename RoadmapGraph<CfgType,WeightType>::vertex_descriptor VID;
   explicit proxy(Accessor const& acc) : Accessor(acc) { }
      operator target_t() const { return Accessor::read(); }
      proxy const& operator=(proxy const& rhs) { Accessor::write(rhs); return *this; }
      proxy const& operator=(target_t const& rhs) { Accessor::write(rhs); return *this;}
      std::vector<VID> GetVIDs() const{ Accessor::const_invoke(&target_t::GetVIDs);}
      CfgType GetCandidate() const{ Accessor::const_invoke(&target_t::GetCandidate);}
  }; //struct proxy
}

#endif

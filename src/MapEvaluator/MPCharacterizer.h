#ifndef MPCharacterizer_h
#define MPCharacterizer_h

#include "util.h"
class MPProblem;

template<typename CFG,typename WEIGHT>
class NodeCharacterizerMethod : public MPBaseObject 
{
  public:
  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
    NodeCharacterizerMethod(XMLNodeReader& in_Node, MPProblem* in_pProblem) : 
      MPBaseObject(in_Node,in_pProblem) { };
    virtual ~NodeCharacterizerMethod() {}
    virtual void ParseXML(XMLNodeReader& in_Node)=0;
    virtual void Characterize(MPRegion<CFG,WEIGHT>*)=0;
    virtual void Characterize(MPRegion<CFG,WEIGHT>*, VID) {};
    virtual void PrintOptions(ostream& out_os)=0;
  private:
  
};


template<typename CFG,typename WEIGHT>
class CCExpandCharacterizer : public NodeCharacterizerMethod<CFG,WEIGHT>
{
  public: 
    typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID; 

    CCExpandCharacterizer(XMLNodeReader& in_Node, MPProblem* in_pProblem) : 
      NodeCharacterizerMethod<CFG,WEIGHT>(in_Node,in_pProblem) {
      LOG_DEBUG_MSG("CCExpandCharacterizer::LocalNodeInfoCharacterizer()");
      ParseXML(in_Node);    
      string dm_label =in_Node.stringXMLParameter(string("dm_method"), false,
                                    string("default"), string("Distance Metric Method"));

      dm = in_pProblem->GetDistanceMetric()->GetDMMethod(dm_label);
      LOG_DEBUG_MSG("~CCExpandCharacterizer::LocalNodeInfoCharacterizer()");
    };
    
    virtual void ParseXML(XMLNodeReader& in_Node) {
      LOG_DEBUG_MSG("CCExpandCharacterizer::ParseXML()");
      LOG_DEBUG_MSG("~CCExpandCharacterizer::ParseXML()");
    };
      
    virtual void Characterize(MPRegion<CFG,WEIGHT>* inout_pRegion) {
      LOG_DEBUG_MSG("CCExpandCharacterizer::Characterize()");
      cout << "CCExpandCharacterizer::Characterize(MPRegion*) Not implemented" << endl;
      exit(-1);
      LOG_DEBUG_MSG("~CCExpandCharacterizer::Characterize()");
    };
    
    virtual void Characterize(MPRegion<CFG,WEIGHT>* inout_pRegion, VID in_vid) {
      LOG_DEBUG_MSG("CCExpandCharacterizer::Characterize()");
      Roadmap<CFG,WEIGHT>* pRoadmap = inout_pRegion->GetRoadmap();
      RoadmapGraph<CFG,WEIGHT>* pGraph = pRoadmap->m_pRoadmap;
      LocalPlanners < CFG, WEIGHT > * lp = this->GetMPProblem()->GetMPStrategy()->GetLocalPlanners();
      LPOutput< CFG, WEIGHT > lp_output; 
      Environment * env = this->GetMPProblem()->GetEnvironment();
//      CollisionDetection * cd = this->GetMPProblem()->GetCollisionDetection();
      double pos_res = this->GetMPProblem()->GetEnvironment()->GetPositionRes();
      double ori_res = this->GetMPProblem()->GetEnvironment()->GetOrientationRes();
      Stat_Class Stats;

      vector<VID> neighbors;
      if(pGraph->get_successors(in_vid, neighbors) > 1)
      {
         cout << "Pls sort me first since your not using CHECKIF SAME CC" << endl; exit(-1);
      }
      //Next find neighbor's neighbors
      vector<VID> neighbor_neighbor;
      pGraph->get_successors(neighbors[0],neighbor_neighbor);
      bool is_expansion = true;
      for(typename vector<VID>::iterator i_n = neighbor_neighbor.begin(); i_n !=neighbor_neighbor.end(); ++i_n)
      {  //test connection to each;
        if(!(lp->IsConnected(env, Stats, dm, 
                             (*(pGraph->find_vertex(in_vid))).property(),
                             (*(pGraph->find_vertex(*i_n))).property(),
                             &lp_output, pos_res, ori_res, true))) {
          is_expansion = false; // cannot connect in_vid to neighbor_neighbor
        }
      }
      if(is_expansion) 
         ((*(pGraph->find_vertex(in_vid))).property()).SetLabel("CCEXPAND",true);
      else
         (*(pGraph->find_vertex(in_vid))).property().SetLabel("CCOVERSAMPLE",true);

      LOG_DEBUG_MSG("~CCExpandCharacterizer::Characterize()");
    };
    virtual void PrintOptions(ostream& out_os) {};
  private:
    shared_ptr<DistanceMetricMethod> dm;
};



template<typename CFG,typename WEIGHT>
class LocalNodeInfoCharacterizer : public NodeCharacterizerMethod<CFG,WEIGHT>
{
  public: 
    typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
    LocalNodeInfoCharacterizer(XMLNodeReader& in_Node, MPProblem* in_pProblem) : 
      NodeCharacterizerMethod<CFG,WEIGHT>(in_Node,in_pProblem) {
      
      LOG_DEBUG_MSG("LocalNodeInfoCharacterizer::LocalNodeInfoCharacterizer()");
      m_dRadius = 0;
      ParseXML(in_Node);    
      string dm_label =in_Node.stringXMLParameter(string("dm_method"), false,
                                    string("default"), string("Distance Metric Method"));
      dm = in_pProblem->GetDistanceMetric()->GetDMMethod(dm_label);
      LOG_DEBUG_MSG("~LocalNodeInfoCharacterizer::LocalNodeInfoCharacterizer()");
    };
    
    virtual void ParseXML(XMLNodeReader& in_Node) {
      LOG_DEBUG_MSG("LocalNodeInfoCharacterizer::ParseXML()");
      

      m_dRadius = in_Node.numberXMLParameter(string("radius"),true,double(0.5),
                                          double(0.0),double(1000.0),
                                          string("Radius Value")); 
      
      LOG_DEBUG_MSG("~LocalNodeInfoCharacterizer::ParseXML()");
    };
      
    virtual void Characterize(MPRegion<CFG,WEIGHT>* inout_pRegion) {
      LOG_DEBUG_MSG("LocalNodeInfoCharacterizer::Characterize()");
      cout << "*********LocalNodeInfoCharacterizer:: m_dRadius = " << m_dRadius << endl;
      Roadmap<CFG,WEIGHT>* pRoadmap = inout_pRegion->GetRoadmap();
      Roadmap<CFG,WEIGHT>* pColRoadmap = inout_pRegion->GetColRoadmap();
      RoadmapGraph<CFG,WEIGHT>* pGraph = pRoadmap->m_pRoadmap;
      RoadmapGraph<CFG,WEIGHT>* pColGraph = pColRoadmap->m_pRoadmap;
      vector<VID> vids;
      pGraph->GetVerticesVID(vids);
      cout << "********Graph has "<< vids.size() << " nodes" << endl;
      vector<VID> col_vids;
      pColGraph->GetVerticesVID(col_vids);
      cout << "********ColGraph has "<< col_vids.size() << " nodes" << endl;
      cout << "Starting Range Query" << endl;
      typename vector<VID>::iterator itr;
      for(itr = vids.begin(); itr != vids.end(); ++itr)
      {
        vector<VID> vids = dm->RangeQuery(pRoadmap,*itr,m_dRadius);
        vector<VID> col_vids = dm->RangeQuery(pColRoadmap,(*(pGraph->find_vertex(*itr))).property(),m_dRadius);
        //cout << "VID = " << *itr << " has " << vids.size() 
        //<< "nodes in radius in Free Map and " 
        //<< col_vids.size() << " in Col Map " << endl;
        if(col_vids.size() >= 1) {
       //   cout << "Gauss-like node found" << endl;
          (*(pGraph->find_vertex(*itr))).property().SetLabel("GaussLike",true);
        }
        if(col_vids.size() >= 2) {
          vector<CFG> tmpCFG;
          for(size_t z=0;z<col_vids.size(); ++z)
            tmpCFG.push_back((*(pColGraph->find_vertex(col_vids[z]))).property());
          if(IsBridgeLike((*(pGraph->find_vertex(*itr))).property(),tmpCFG)) {
          cout << "Bridge-like node found" << endl;
          (*(pGraph->find_vertex(*itr))).property().SetLabel("BridgeLike",true);
          }
        }
        if(col_vids.size() > 2) {
         // cout << "Better than bridge node found" << endl;
          (*(pGraph->find_vertex(*itr))).property().SetLabel("BetterThanBridge",true);
        }
      }
      LOG_DEBUG_MSG("~LocalNodeInfoCharacterizer::Characterize()");
    };
    
    virtual void PrintOptions(ostream& out_os) {};
  private:
    shared_ptr< DistanceMetricMethod> dm;
    
    bool IsBridgeLike(CFG free_cfg,vector<CFG> vec_col) {
      typename vector<CFG>::iterator I,J;
      for(I=vec_col.begin(); I!=vec_col.end(); ++I) {
        for(J=I; J!=vec_col.end(); ++J) {
          if(*I == *J)
            continue;
          double df1,df2,dc;
          df1 = dm->Distance(this->GetMPProblem()->GetEnvironment(),free_cfg,*I);
          df2 = dm->Distance(this->GetMPProblem()->GetEnvironment(),free_cfg,*J);
          dc = dm->Distance(this->GetMPProblem()->GetEnvironment(),*I,*J);
          if(dc >= 0.491 * (df1 + df2 + dc)) {
             return true;
          }
        }
      }
      return false;
    };
    
    double m_dRadius;
};







template<typename CFG, typename WEIGHT>
class MPCharacterizer : public MPBaseObject {
public:
  MPCharacterizer(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
      MPBaseObject(in_Node, in_pProblem) {
    LOG_DEBUG_MSG( "MPCharacterizer::MPCharacterizer()");
    ParseXML(in_Node);
    LOG_DEBUG_MSG( "~MPCharacterizer::MPCharacterizer()");
  }
  
  void ParseXML(XMLNodeReader& in_Node) {
    LOG_DEBUG_MSG("MPCharacterizer::ParseXML()");
    
    XMLNodeReader::childiterator citr;
    for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {   
      if(citr->getName() == "LocalNodeInfoCharacterizer") {
        LocalNodeInfoCharacterizer<CFG,WEIGHT>* localnodeinfo = 
            new LocalNodeInfoCharacterizer<CFG,WEIGHT>(*citr,this->GetMPProblem());
        all_NodeCharacterizerMethod.push_back(localnodeinfo);
      } else if(citr->getName() == "CCExpandCharacterizer") {
        CCExpandCharacterizer<CFG,WEIGHT>* expandchar = 
            new CCExpandCharacterizer<CFG,WEIGHT>(*citr,this->GetMPProblem());
        all_NodeCharacterizerMethod.push_back(expandchar);
      } else {
        citr->warnUnrequestedAttributes();
      }
    }
    LOG_DEBUG_MSG("~MPCharacterizer::ParseXML()");
  }



  
  void PrintOptions(ostream& out_os){};
  
  NodeCharacterizerMethod<CFG,WEIGHT>* GetNodeCharacterizerMethod(string& in_strLabel) {
    typename vector<NodeCharacterizerMethod<CFG,WEIGHT>*>::iterator I;
    for(I = all_NodeCharacterizerMethod.begin(); 
      I != all_NodeCharacterizerMethod.end(); ++I) {
      if((*I)->GetLabel() == in_strLabel) {
        return (*I);
      }
    }
  }
  
  private:
    vector< NodeCharacterizerMethod<CFG,WEIGHT>* > all_NodeCharacterizerMethod;
    //vector<EdgeChar...>
    //vector<RegionChar...>

};

#endif

#ifndef MPCharacterizer_h
#define MPCharacterizer_h

class MPProblem;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MapEvaluators
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
template<typename CFG,typename WEIGHT>
class NodeCharacterizerMethod : public MPBaseObject {
  public:
  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
    NodeCharacterizerMethod(XMLNode& _inNode, MPProblem* _inpProblem) :
      MPBaseObject(_inNode,_inpProblem) { };
    NodeCharacterizerMethod() {};
    virtual ~NodeCharacterizerMethod() {}
    virtual void ParseXML(XMLNode& _inNode)=0;
    virtual void Characterize()=0;
    virtual void Characterize(VID) {};
    virtual void Print(ostream& _out_os) const {}
};


////////////////////////////////////////////////////////////////////////////////
/// @ingroup MapEvaluators
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
template<typename CFG,typename WEIGHT>
class CCExpandCharacterizer : public NodeCharacterizerMethod<CFG,WEIGHT> {
  public:

    typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;

    CCExpandCharacterizer(shared_ptr<DistanceMetricMethod> _dm) : dm(_dm) {}

    CCExpandCharacterizer(XMLNode& _inNode, MPProblem* _inpProblem) :
      NodeCharacterizerMethod<CFG,WEIGHT>(_inNode,_inpProblem) {
      ParseXML(_inNode);
      string dm_label =_inNode.Read(string("dm_method"), false,
                                    string("default"), string("Distance Metric Method"));
      m_lp = _inNode.Read(string("lp_method"), false, string("default"), string("Local Planner"));
      dm = _inpProblem->GetDistanceMetric()->GetMethod(dm_label);
    };

    virtual void ParseXML(XMLNode& _inNode) { };

    virtual void Characterize() {
      cout << "CCExpandCharacterizer::Characterize() Not implemented" << endl;
      exit(-1);
    };

    virtual void Characterize(VID _inVid) {
      Roadmap<CFG,WEIGHT>* pRoadmap = this->GetMPProblem()->GetRoadmap();
      RoadmapGraph<CFG,WEIGHT>* pGraph = pRoadmap->m_pRoadmap;
      LocalPlanners < CFG, WEIGHT > * lp = this->GetMPProblem()->GetMPStrategy()->GetLocalPlanners();
      LPOutput< CFG, WEIGHT > lp_output;
      Environment* env = this->GetMPProblem()->GetEnvironment();
      double pos_res = this->GetMPProblem()->GetEnvironment()->GetPositionRes();
      double ori_res = this->GetMPProblem()->GetEnvironment()->GetOrientationRes();
      StatClass Stats;

      vector<VID> neighbors;
      if(pGraph->get_successors(_inVid, neighbors) > 1){
         cout << "Pls sort me first since your not using CHECKIF SAME CC" << endl; exit(-1);
      }
      //Next find neighbor's neighbors
      vector<VID> neighbor_neighbor;
      pGraph->get_successors(neighbors[0],neighbor_neighbor);
      bool is_expansion = true;
      for(typename vector<VID>::iterator i_n = neighbor_neighbor.begin(); i_n !=neighbor_neighbor.end(); ++i_n) {  //test connection to each;
        if(!(lp->GetMethod(m_lp)->
               IsConnected(env, Stats, dm,
                           (*(pGraph->find_vertex(_inVid))).property(),
                           (*(pGraph->find_vertex(*i_n))).property(),
                           &lp_output, pos_res, ori_res, true))) {
          is_expansion = false; // cannot connect _inVid to neighbor_neighbor
        }
      }
      if(is_expansion)
         ((*(pGraph->find_vertex(_inVid))).property()).SetLabel("CCEXPAND",true);
      else
         (*(pGraph->find_vertex(_inVid))).property().SetLabel("CCOVERSAMPLE",true);

    };
    virtual void Print(ostream& out_os) const {};
  private:
    shared_ptr<DistanceMetricMethod> dm;
    string m_lp;
};



////////////////////////////////////////////////////////////////////////////////
/// @ingroup MapEvaluators
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
template<typename CFG,typename WEIGHT>
class LocalNodeInfoCharacterizer : public NodeCharacterizerMethod<CFG,WEIGHT> {
  public:
    typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
    LocalNodeInfoCharacterizer(string _dm, double dRadius) : NodeCharacterizerMethod<CFG,WEIGHT>(), m_dmLabel(_dm), m_dRadius(dRadius)  {}
    LocalNodeInfoCharacterizer(XMLNode& _inNode, MPProblem* _inpProblem) :
      NodeCharacterizerMethod<CFG,WEIGHT>(_inNode,_inpProblem) {

      m_dRadius = 0;
      ParseXML(_inNode);
      m_dmLabel =_inNode.Read(string("dm_method"), false,
                                    string("default"), string("Distance Metric Method"));
    };

    virtual void ParseXML(XMLNode& _inNode) {

      m_dRadius = _inNode.Read(string("radius"),true,double(0.5),
                                          double(0.0),double(1000.0),
                                          string("Radius Value"));

    };

    virtual void Characterize() {
      cout << "*********LocalNodeInfoCharacterizer:: m_dRadius = " << m_dRadius << endl;
      NeighborhoodFinderMethod* rnf = new RadiusNF(m_dmLabel, m_dRadius, "", this->GetMPProblem());
      Roadmap<CFG,WEIGHT>* pRoadmap = this->GetMPProblem()->GetRoadmap();
      Roadmap<CFG,WEIGHT>* pColRoadmap = this->GetMPProblem()->GetColRoadmap();
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
      for(itr = vids.begin(); itr != vids.end(); ++itr) {
        vector<VID> vid;//s = dm->RangeQuery(pRoadmap,*itr,m_dRadius);
        vector<VID> col_vids;// = dm->RangeQuery(pColRoadmap,(*(pGraph->find_vertex(*itr))).property(),m_dRadius);
        rnf->KClosest(pRoadmap, *itr, 1, back_inserter(vid));
        rnf->KClosest(pColRoadmap, (*(pGraph->find_vertex(*itr))).property(), 1, back_inserter(col_vids));
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
      delete rnf;
    };

    virtual void Print(ostream& out_os) const {};
  private:
    bool IsBridgeLike(CFG free_cfg,vector<CFG> vec_col) {
      DistanceMetric::DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric()->GetMethod(m_dmLabel);
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
    string m_dmLabel;
};



////////////////////////////////////////////////////////////////////////////////
/// @ingroup MapEvaluators
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
template<typename CFG, typename WEIGHT>
class MPCharacterizer : public MPBaseObject {
public:
  MPCharacterizer(vector< NodeCharacterizerMethod<CFG,WEIGHT>* > all) : all_NodeCharacterizerMethod(all) {};
  MPCharacterizer(XMLNode& _inNode, MPProblem* _inpProblem) :
      MPBaseObject(_inNode, _inpProblem) {
    ParseXML(_inNode);
  }

  void ParseXML(XMLNode& _inNode) {

    XMLNode::childiterator citr;
    for(citr = _inNode.children_begin(); citr!= _inNode.children_end(); ++citr) {
      if(citr->getName() == "LocalNodeInfoCharacterizer") {
        LocalNodeInfoCharacterizer<CFG,WEIGHT>* localnodeinfo =
            new LocalNodeInfoCharacterizer<CFG,WEIGHT>(*citr,this->GetMPProblem());
        all_NodeCharacterizerMethod.push_back(localnodeinfo);
      }
      else if(citr->getName() == "CCExpandCharacterizer") {
        CCExpandCharacterizer<CFG,WEIGHT>* expandchar =
            new CCExpandCharacterizer<CFG,WEIGHT>(*citr,this->GetMPProblem());
        all_NodeCharacterizerMethod.push_back(expandchar);
      }
      else {
        citr->warnUnrequestedAttributes();
      }
    }
  }




  void Print(ostream& out_os) const {};

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

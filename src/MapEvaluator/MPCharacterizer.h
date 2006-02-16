#ifndef MPCharacterizer_h
#define MPCharacterizer_h

#include "util.h"
#include "MPProblem.h"


template<typename CFG,typename WEIGHT>
class NodeCharacterizerMethod : public MPBaseObject 
{
  public:
    NodeCharacterizerMethod(TiXmlNode* in_pNode, MPProblem* in_pProblem) : 
      MPBaseObject(in_pNode,in_pProblem) { };
    virtual void ParseXML(TiXmlNode* in_pNode)=0;
    virtual void Characterize(MPRegion<CFG,WEIGHT>*)=0;
    virtual void Characterize(MPRegion<CFG,WEIGHT>*, VID) {};
    virtual void PrintOptions(ostream& out_os)=0;
  private:
  
};


template<typename CFG,typename WEIGHT>
class CCExpandCharacterizer : public NodeCharacterizerMethod<CFG,WEIGHT>
{
  public: 
    CCExpandCharacterizer(TiXmlNode* in_pNode, MPProblem* in_pProblem) : 
      NodeCharacterizerMethod<CFG,WEIGHT>(in_pNode,in_pProblem) {
      LOG_DEBUG_MSG("CCExpandCharacterizer::LocalNodeInfoCharacterizer()");
      ParseXML(in_pNode);    
      LOG_DEBUG_MSG("~CCExpandCharacterizer::LocalNodeInfoCharacterizer()");
    };
    
    virtual void ParseXML(TiXmlNode* in_pNode) {
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
      LocalPlanners < CFG, WEIGHT > * lp = GetMPProblem()->GetMPStrategy()->GetLocalPlanners();
      LPOutput< CFG, WEIGHT > lp_output; 
      Environment * env = GetMPProblem()->GetEnvironment();
      CollisionDetection * cd = GetMPProblem()->GetCollisionDetection();
      DistanceMetric * dm = GetMPProblem()->GetDistanceMetric();
      double pos_res = GetMPProblem()->GetEnvironment()->GetPositionRes();
      double ori_res = GetMPProblem()->GetEnvironment()->GetOrientationRes();
      Stat_Class Stats;

      vector<VID> neighbors;
      if(pGraph->GetSuccessors(in_vid, neighbors) > 1)
      {
         cout << "Pls sort me first since your not using CHECKIF SMAE CC" << endl; exit(-1);
      }
      //Next find neighbor's neighbors
      vector<VID> neighbor_neighbor;
      pGraph->GetSuccessors(neighbors[0],neighbor_neighbor);
      bool is_expansion = true;
      for(vector<VID>::iterator i_n = neighbor_neighbor.begin(); i_n !=neighbor_neighbor.end(); ++i_n)
      {  //test connection to each;
        if(!(lp->IsConnected(env, Stats, cd, dm, 
                             pGraph->GetData(in_vid),
                             pGraph->GetData(*i_n),
                             &lp_output, pos_res, ori_res, true))) {
          is_expansion = false; // cannot connect in_vid to neighbor_neighbor
        }
      }
      if(is_expansion) 
         pGraph->GetReferenceofData(in_vid)->SetLabel("CCEXPAND",true);
      else
         pGraph->GetReferenceofData(in_vid)->SetLabel("CCOVERSAMPLE",true);

      LOG_DEBUG_MSG("~CCExpandCharacterizer::Characterize()");
    };
    virtual void PrintOptions(ostream& out_os) {};
  private:
};



template<typename CFG,typename WEIGHT>
class LocalNodeInfoCharacterizer : public NodeCharacterizerMethod<CFG,WEIGHT>
{
  public: 
    LocalNodeInfoCharacterizer(TiXmlNode* in_pNode, MPProblem* in_pProblem) : 
      NodeCharacterizerMethod<CFG,WEIGHT>(in_pNode,in_pProblem) {
      
      LOG_DEBUG_MSG("LocalNodeInfoCharacterizer::LocalNodeInfoCharacterizer()");
      m_dRadius = 0;
      ParseXML(in_pNode);    
      LOG_DEBUG_MSG("~LocalNodeInfoCharacterizer::LocalNodeInfoCharacterizer()");
    };
    
    virtual void ParseXML(TiXmlNode* in_pNode) {
      LOG_DEBUG_MSG("LocalNodeInfoCharacterizer::ParseXML()");
      
      double radius;
      int query = in_pNode->ToElement()->QueryDoubleAttribute("radius", &radius);
      if(query == TIXML_SUCCESS ) {
        m_dRadius = radius;
      }
      else {
          LOG_WARNING_MSG("LocalNodeInfoCharacterizer::  I don't know: "<< endl << *in_pNode);
      }
     
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
        vector<VID> vids = this->GetMPProblem()->GetDistanceMetric()->RangeQuery(pRoadmap,*itr,m_dRadius);
        vector<VID> col_vids = this->GetMPProblem()->GetDistanceMetric()->RangeQuery(pColRoadmap,pGraph->GetData(*itr),m_dRadius);
        //cout << "VID = " << *itr << " has " << vids.size() 
        //<< "nodes in radius in Free Map and " 
        //<< col_vids.size() << " in Col Map " << endl;
        if(col_vids.size() >= 1) {
       //   cout << "Gauss-like node found" << endl;
          pGraph->GetReferenceofData(*itr)->SetLabel("GaussLike",true);
        }
        if(col_vids.size() >= 2) {
          vector<CFG> tmpCFG;
          for(int z=0;z<col_vids.size(); ++z)
            tmpCFG.push_back(pColGraph->GetData(col_vids[z]));
          if(IsBridgeLike(pGraph->GetData(*itr),tmpCFG)) {
          cout << "Bridge-like node found" << endl;
          pGraph->GetReferenceofData(*itr)->SetLabel("BridgeLike",true);
          }
        }
        if(col_vids.size() > 2) {
         // cout << "Better than bridge node found" << endl;
          pGraph->GetReferenceofData(*itr)->SetLabel("BetterThanBridge",true);
        }
      }
      LOG_DEBUG_MSG("~LocalNodeInfoCharacterizer::Characterize()");
    };
    
    virtual void PrintOptions(ostream& out_os) {};
  private:
    
    bool IsBridgeLike(CFG free_cfg,vector<CFG> vec_col) {
      typename vector<CFG>::iterator I,J;
      for(I=vec_col.begin(); I!=vec_col.end(); ++I) {
        for(J=I; J!=vec_col.end(); ++J) {
          if(*I == *J)
            continue;
          double df1,df2,dc;
          DistanceMetric* dm = this->GetMPProblem()->GetDistanceMetric();
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
  MPCharacterizer(TiXmlNode* in_pNode, MPProblem* in_pProblem) :
      MPBaseObject(in_pNode, in_pProblem) {
    LOG_DEBUG_MSG( "MPCharacterizer::MPCharacterizer()");
    ParseXML(in_pNode);
    LOG_DEBUG_MSG( "~MPCharacterizer::MPCharacterizer()");
  }
  
  void ParseXML(TiXmlNode* in_pNode) {
    LOG_DEBUG_MSG("MPCharacterizer::ParseXML()");
      
    for( TiXmlNode* pChild = in_pNode->FirstChild(); pChild !=0; 
      pChild = pChild->NextSibling()) {
      
      if(string(pChild->Value()) == "LocalNodeInfoCharacterizer") {
        LocalNodeInfoCharacterizer<CFG,WEIGHT>* localnodeinfo = 
            new LocalNodeInfoCharacterizer<CFG,WEIGHT>(pChild,this->GetMPProblem());
        all_NodeCharacterizerMethod.push_back(localnodeinfo);
      } else if(string(pChild->Value()) == "CCExpandCharacterizer") {
        CCExpandCharacterizer<CFG,WEIGHT>* expandchar = 
            new CCExpandCharacterizer<CFG,WEIGHT>(pChild,GetMPProblem());
        all_NodeCharacterizerMethod.push_back(expandchar);
      } else {
        LOG_WARNING_MSG("MPCharacterizer::  I don't know: "<< endl << *pChild);
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

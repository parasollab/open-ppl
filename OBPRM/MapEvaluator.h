#ifndef _MAP_EVALUATOR_H
#define _MAP_EVALUATOR_H
#include "Roadmap.h"

template <class CFG, class WEIGTH> 
class MapEvaluationMethod;

template <class CFG, class WEIGHT>
class MapEvaluator {
 public:
  vector<MapEvaluationMethod<CFG,WEIGHT>*> evaluators;

  MapEvaluator() {}
  MapEvaluator(const vector<MapEvaluationMethod<CFG,WEIGHT>*>& e) : 
    evaluators(e) {}
  ~MapEvaluator() {}

  void AddEvaluator(MapEvaluationMethod<CFG,WEIGHT>* e) {
    evaluators.push_back(e);
  }

  bool operator()(Roadmap<CFG,WEIGHT>* rmap) {
    typename vector<MapEvaluationMethod<CFG,WEIGHT>*>::iterator E;
    for(E = evaluators.begin(); E != evaluators.end(); ++E)
      if(!(*E)->evaluate(rmap))
	return false;
    return true;
  }
};

template <class CFG, class WEIGHT>
class MapEvaluationMethod {
 public:
  virtual bool evaluate(Roadmap<CFG,WEIGHT>* rmap) = 0;
};


////////////////////////////////////////
//TEST
//for testing the framework only... will be deleted later
template <class CFG, class WEIGHT>
class TestEvaluation : public MapEvaluationMethod<CFG,WEIGHT> {
 public:
  int size;
  TestEvaluation(int s = 400) : size(s) {}
  ~TestEvaluation() {}

  virtual bool evaluate(Roadmap<CFG,WEIGHT>* rmap) {
    return (rmap->m_pRoadmap->GetVertexCount() > size);
  }
};



/////////////////////////
// Max Flow Evaluators
#include "GraphAlgo.h"

template <class CFG, class WEIGHT, class CAPACITY>
class MaxFlowEvaluation : public MapEvaluationMethod<CFG,WEIGHT> {
 public:
  VID source, sink;
  double flow_size;

  virtual bool evaluate(Roadmap<CFG,WEIGHT>* rmap) {
    CAPACITY capacity(*(rmap->m_pRoadmap));
    double flow = max_flow(*(rmap->m_pRoadmap), source, sink, capacity);
    return (flow >= flow_size);
  }
};

template <class InputIterator, class EqualityComparable>
InputIterator
find_first(InputIterator first, InputIterator last,
	   const EqualityComparable& value) {
  while((first != last) && !(value == first->first)) 
    first++;
  return first; 
};

template <class GRAPH>
struct ConstantCapacity {
  typedef typename GRAPH::EI EI;
  vector<pair<EI, double> > capacity_map;
  
  ConstantCapacity(GRAPH& G) {
    vector<VID> vids;
    G.GetVerticesVID(vids);
    typename GRAPH::VI v;
    EI e;
    for(typename vector<VID>::const_iterator V = vids.begin(); V != vids.end(); ++V) {
      G.IsVertex(*V, &v);
      for(e = v->edgelist.begin(); e != v->edgelist.end(); ++e)
        capacity_map.push_back(make_pair(e, 1));
    }
  }
  ~ConstantCapacity() {}
  
  void SetCapacity(EI e, double c) {
    if(c <= 0) {
      typename vector<pair<EI, double> >::iterator CM =
        find_first(capacity_map.begin(), capacity_map.end(), e);
      if(CM != capacity_map.end())
	CM->second = 0;
    }
  }
  
  double GetCapacity(EI e) const {
    typename vector<pair<EI, double> >::const_iterator CM = 
      find_first(capacity_map.begin(), capacity_map.end(), e);
    if(CM != capacity_map.end())
      return CM->second;
  }
};

template <class GRAPH>
struct InverseWeightCapacity {
  typedef typename GRAPH::EI EI;
  vector<pair<EI, double> > capacity_map;
  
  InverseWeightCapacity(GRAPH& G) {
    vector<VID> vids;
    G.GetVerticesVID(vids);
    typename GRAPH::VI v;
    EI e;
    for(typename vector<VID>::const_iterator V = vids.begin(); V != vids.end(); ++V) {
      G.IsVertex(*V, &v);
      for(e = v->edgelist.begin(); e != v->edgelist.end(); ++e)
        capacity_map.push_back(make_pair(e, 1/e->weight.Weight()));
    }
  }
  ~InverseWeightCapacity() {}
  
  void SetCapacity(EI e, double c) {
    typename vector<pair<EI, double> >::iterator CM = 
      find_first(capacity_map.begin(), capacity_map.end(), e);
    if(CM != capacity_map.end())
      if(c <= 0)
	CM->second = 0;
      else
	CM->second = c;
  }
  
  double GetCapacity(EI e) const {
    typename vector<pair<EI, double> >::const_iterator CM = 
      find_first(capacity_map.begin(), capacity_map.end(), e);
    if(CM != capacity_map.end())
      return CM->second;
  }
};



/////////////////////////
// evaluate the coverage of a given roadmap
#include "NodeGenerationMethod.h"
 
template <class CFG, class WEIGHT>
class CoverageEvaluation : public MapEvaluationMethod<CFG,WEIGHT> 
{
    typedef NodeGenerationMethod<CFG> NGM;
    typedef ConnectMap<CFG,WEIGHT>    CM;

public:

    CoverageEvaluation
    (int nsize, float t, NGM* gn, CM* cm, 
     LocalPlanners<CFG, WEIGHT>* lp,
     CollisionDetection* cd, DistanceMetric* dm)
     :m_gn(gn), m_cm(cm), m_lp(lp), m_cd(cd), m_dm(dm)
    {
        m_nodesize=nsize;
        m_threshold=t;
    }

    virtual bool evaluate(Roadmap<CFG,WEIGHT>* rmap) 
    {
        if(m_nodesize==0) return true; //nothing to evaluate.

        Stat_Class my_stats;
        Environment * p_env=rmap->GetEnvironment();
        RoadmapGraph<CFG,WEIGHT> * rmapG=rmap->m_pRoadmap;

        //generate n nodes, and put into test_nodes
        vector<CFG> test_nodes;
        int nsize_backup=m_gn->numNodes.GetValue();
        int exact_backup=m_gn->exactNodes.GetValue();
        m_gn->numNodes.PutValue(m_nodesize);
        m_gn->exactNodes.PutValue(1);
        m_gn->GenerateNodes
            (rmap->GetEnvironment(),my_stats,m_cd,m_dm,test_nodes);
        m_gn->numNodes.PutValue(nsize_backup);
        m_gn->exactNodes.PutValue(exact_backup);

        if(test_nodes.size()<m_nodesize)
            //should print warning here
            m_nodesize=test_nodes.size();

        //connect n nodes to the map
        int connected=0;
		VID backupVID=rmapG->getVertIDs();
        vector< vector<CFG> > vv(2,vector<CFG>());
        rmapG->GetVerticesData(vv[1]); //get nodes in the roadmap
        for( int i=0;i<m_nodesize;i++ ){ //for each node
            vv[0].clear(); //empty the vector
            vv[0].push_back(test_nodes[i]);
            //add the node to the map
            VID added_ID=rmapG->AddVertex(test_nodes[i]);
            //connect the node to the rest of the map
            m_cm->ConnectComponents
                (rmap,my_stats,m_cd,m_dm,m_lp,false,false,vv);
            //check if the node has connections
            if(rmapG->GetVertexOutDegree(added_ID)>0)
                connected++; // this node has been connected to the map
            //remove the node from the map
            rmapG->DeleteVertex(added_ID);
        }
		rmapG->setVertIDs(backupVID); 
		//done    
        return (((float)connected)/m_nodesize)>m_threshold;
    }

private:

    int m_nodesize;     // number of nodes to generate for testing
    float m_threshold; // success coverage rate

    //planner stuff
    NGM* m_gn;  //used to generate map nodes
    CM * m_cm;  //used to connect map nodes
    LocalPlanners<CFG, WEIGHT>* m_lp;  //local planner
    CollisionDetection* m_cd;
    DistanceMetric* m_dm;
};


/////////////////////////////////////////
//query evaluation
#include "Query.h"

template <class CFG, class WEIGHT>
class QueryEvaluation : public MapEvaluationMethod<CFG,WEIGHT> {

  public:

  QueryEvaluation(char * queryFileName, 
                 Stat_Class& Stats, CollisionDetection* cd , 
		 ConnectMap<CFG, WEIGHT> *cm, DistanceMetric * dm,
		 LocalPlanners<CFG,WEIGHT>* lp);
  QueryEvaluation(CFG _start, CFG _goal,
                 Stat_Class& Stats, CollisionDetection* cd , 
		 ConnectMap<CFG, WEIGHT> *cm, DistanceMetric * dm,
		 LocalPlanners<CFG,WEIGHT>* lp);
  ~QueryEvaluation();

  virtual bool evaluate(Roadmap<CFG,WEIGHT>* rmap);


  ///////////////////////////
  //
  //  DATA MEMBERS
  //
  ///////////////////////////

  private:

  Query<CFG, WEIGHT> query;
  Stat_Class mystats;
  CollisionDetection* mycd;
  ConnectMap<CFG, WEIGHT> *mycm;
  DistanceMetric * mydm;
  LocalPlanners<CFG,WEIGHT>* mylp;


};  



template <class CFG, class WEIGHT>
QueryEvaluation<CFG, WEIGHT>::
QueryEvaluation(char * queryFileName, 
                 Stat_Class& Stats, CollisionDetection* cd , 
		 ConnectMap<CFG, WEIGHT> *cm, DistanceMetric * dm,
		 LocalPlanners<CFG,WEIGHT>* lp) : query(queryFileName) , mycd(cd) , mycm(cm) , mydm(dm) , mylp(lp) {

}


template <class CFG, class WEIGHT>
QueryEvaluation<CFG, WEIGHT>::
QueryEvaluation(CFG _start, CFG _goal,
                 Stat_Class& Stats, CollisionDetection* cd , 
		 ConnectMap<CFG, WEIGHT> *cm, DistanceMetric * dm,
		 LocalPlanners<CFG,WEIGHT>* lp) : query(_start, _goal) , mycd(cd) , mycm(cm) , mydm(dm) , mylp(lp) {
}


template <class CFG, class WEIGHT>
QueryEvaluation<CFG, WEIGHT>::
~QueryEvaluation(){}


template <class CFG, class WEIGHT>
bool
QueryEvaluation<CFG, WEIGHT>::
evaluate(Roadmap<CFG,WEIGHT>* rmap) {

  bool queryResult;
  int oriVertID = rmap->m_pRoadmap->getVertIDs();

  queryResult = query.PerformQuery(rmap, mystats,mycd,mycm,mylp,mydm);
  
  //remove nodes in query.query from rmap
  for(int i = 0; i< query.query.size(); i++){
    rmap->m_pRoadmap->DeleteVertex(query.query[i]);  
  }

  rmap->m_pRoadmap->setVertIDs(oriVertID);

  return queryResult;

}
#endif

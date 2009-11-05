



#ifndef _ST_SPILL_TREE_CLASS_H_
#define _ST_SPILL_TREE_CLASS_H_

 


#include "Graph.h"
#include <list>
#include <vector>
#include "CGAL/Cartesian_d.h"
#include <CGAL/constructions_d.h>
using namespace std;


class Input;
class Environment;


class vertexDistance{
public:
  VID vertex;
  double distance;

  vertexDistance(VID _vertex, double _distance){
    vertex=_vertex;
    distance=_distance;
  }
};

template<typename CFGTYPE, typename WEIGHT>
class spillTreeNode{

 public:
  list<VID> *verticies;
  list<VID> verticiesObj;
  double maxDistance;  
  CFGTYPE center;
  VID mostDistantVertex1;
  VID mostDistantVertex2;
  double median;
  bool isSpillTreeNode;
  double distanceToMidpoint;

  spillTreeNode(){
    isSpillTreeNode=true;
  }


  spillTreeNode(list<VID> &_verticies){
    verticies=&_verticies;
    isSpillTreeNode=true;
  }


  spillTreeNode(list<VID> *_verticies){
    verticiesObj=*_verticies;
    verticies=&verticiesObj;
    isSpillTreeNode=true;
  }
};


template<typename CFGTYPE, typename WEIGHT> 
class spillTree{
  
 public:
  typedef stapl::Graph<stapl::DIRECTED,stapl::NONMULTIEDGES,stapl::WEIGHTED,  spillTreeNode<CFGTYPE, WEIGHT>* > graphType;
  int maxLeafSize;
  double overlapDistance;
  double maxPerBranch;
  bool divideAtMedian;
  Roadmap<CFGTYPE,WEIGHT>* rmap;
  DistanceMetricMethod* dmm;
  graphType *tree;
  VID root;
  Environment *env;
  bool includeQueryCFG;
  bool isMetricTree;

  spillTree(){}

  spillTree(int _maxLeafSize, double _overlapDistance, double _maxPerBranch, bool _divideAtMedian, bool  _includeQueryCFG, bool _isMetricTree){
    //cout<<"in construct"<<endl;
    maxLeafSize=_maxLeafSize;
    overlapDistance=_overlapDistance;
    maxPerBranch=_maxPerBranch;
    divideAtMedian=_divideAtMedian;
    tree = new graphType();
    includeQueryCFG = _includeQueryCFG;
    isMetricTree=_isMetricTree;
  }

  ~spillTree(){
      tree->graphType::~Graph();
  }
  
  bool isLeaf(VID node){
    return (tree->GetVertexOutDegree(node)==0);
  }

  VID getLChild(VID node){
    vector<VID> successors;
    tree->GetSuccessors(node, successors);
    return successors.front();
  }

  VID getRChild(VID node){
    vector<VID> successors;
    tree->GetSuccessors(node, successors);
    return successors.back();
  }

  spillTreeNode<CFGTYPE,WEIGHT>* getNodeDataStructure(VID node){
    return tree->GetData(node);
  }

  VID createChild(VID parent, list<VID> *vertexList){
    spillTreeNode<CFGTYPE,WEIGHT> *newNode = new spillTreeNode<CFGTYPE, WEIGHT>(vertexList);
    VID newNodeVID = tree->AddVertex(newNode);
    tree->AddEdge(parent, newNodeVID);
    return newNodeVID;
  }

  

  void splitNode(VID node){
    std::list<VID> rSublist;
    std::list<VID> lSublist;
    divideList(getNodeDataStructure(node),lSublist,rSublist);
    createChild(node,lSublist);
    createChild(node,rSublist);
  }
  

  list<vertexDistance>* addIfClosest(const CFGTYPE &queryCFG, VID v,  list<vertexDistance> *closest, int k){
    RoadmapGraph<CFGTYPE,WEIGHT>* pMap = rmap->m_pRoadmap;
    if(pMap->GetData(v)==queryCFG && !includeQueryCFG)
      return closest;
    vertexDistance distance_v=vertexDistance(v, distance(v,queryCFG));
    for(std::list<vertexDistance>::iterator iter = closest->begin(); iter != closest->end(); iter++){
      if(v==(*iter).vertex)
	return closest;  
      if(distance_v.distance<(*iter).distance){
        closest->insert(iter,distance_v);
        if(closest->size()>k)
          closest->pop_back();
        return closest;
      }
    }
    if(closest->size()<k)
      closest->push_back(distance_v);
    return closest;
  }

  //replace with existing method
  void findClosest(const CFGTYPE &queryCFG, int k, VID nodeVID, list<vertexDistance> *closest){
    spillTreeNode<CFGTYPE,WEIGHT>* node = getNodeDataStructure(nodeVID);
    for(std::list<VID>::iterator iter = node->verticies->begin(); iter != node->verticies->end(); iter++){
      closest=addIfClosest(queryCFG, *iter, closest, k);
    }
  }
  
  bool canPrune(const CFGTYPE &queryCFG, int k, VID node, const list<vertexDistance> *closest){
    return ((closest->back().distance)<(distance(getNodeDataStructure(node)->center,queryCFG)-getNodeDataStructure(node)->maxDistance));
  }

  void query(const CFGTYPE &queryCFG, int k, VID node, list<vertexDistance> *closest){
    //cout<<"in query"<<endl;
    if(isLeaf(node)){
      list<VID> *v2=getNodeDataStructure(node)->verticies;
      findClosest(queryCFG, k, node, closest);   
    }else{
      if(isToLeftOfPartition(queryCFG, node)){
        query(queryCFG, k, getLChild(node), closest);
	//cout<<"query left"<<endl;
        if(closest->size()<k || (!(getNodeDataStructure(node)->isSpillTreeNode) && !canPrune(queryCFG,k, getRChild(node), closest))){
	  query(queryCFG, k, getRChild(node), closest);
	  //cout<<"not pruined"<<endl;
	}
      }else{
        query(queryCFG, k, getRChild(node), closest);
        //cout<<"query right"<<endl;
        if(closest->size()<k || (!(getNodeDataStructure(node)->isSpillTreeNode) && !canPrune(queryCFG,k, getLChild(node), closest))){
	  //cout<<"not pruined"<<endl;
          query(queryCFG, k, getLChild(node), closest);
        }
      }
    }
  }

  vector<VID>* query(const CFGTYPE &v, int k){
    list<vertexDistance> *closestDistances = new list<vertexDistance>();
    query(v, k, root, closestDistances);
    vector<VID> *closest = new vector<VID>();
    for(std::list<vertexDistance>::iterator iter = closestDistances->begin(); iter != closestDistances->end(); iter++)
      closest->push_back((*iter).vertex);
    return closest;
  }

  vector<VID>* query(VID v, int k){
    RoadmapGraph<CFGTYPE,WEIGHT>* pMap = rmap->m_pRoadmap;
    return query(pMap->GetData(v), k);
  }

  

  
  double project(VID v1, VID v2, VID v3){
    RoadmapGraph<CFGTYPE,WEIGHT>* pMap = rmap->m_pRoadmap;
    CFGTYPE cfg1 = pMap->GetData(v1);
    CFGTYPE cfg2 = pMap->GetData(v2);
    CFGTYPE cfg3 = pMap->GetData(v3);
    return project(cfg1, cfg2, cfg3);
  }
    
  
  double project(const CFGTYPE &cfg, VID v2, VID v3){
    RoadmapGraph<CFGTYPE,WEIGHT>* pMap = rmap->m_pRoadmap;
    CFGTYPE cfg2 = pMap->GetData(v2);
    CFGTYPE cfg3 = pMap->GetData(v3);
    return project(cfg, cfg2, cfg3);
  }
      
  CGAL::Point_d< CGAL::Cartesian_d<double> > toCgalPoint(VID v){
    RoadmapGraph<CFGTYPE,WEIGHT>* pMap = rmap->m_pRoadmap;
    CFGTYPE cfg = pMap->GetData(v);
    return toCgalPoint(cfg);
  }

  CGAL::Point_d< CGAL::Cartesian_d<double> > toCgalPoint(CFGTYPE cfg){
    CGAL::Point_d< CGAL::Cartesian_d<double> > p(cfg.GetData().size(), cfg.GetData().begin(),cfg.GetData().end());
    return p;
  }


  double project(const CFGTYPE &cfg1, const CFGTYPE &cfg2, const CFGTYPE &cfg3){
    CGAL::Point_d< CGAL::Cartesian_d<double> > p1(cfg1.GetData().size(), cfg1.GetData().begin(),cfg1.GetData().end());
    CGAL::Point_d< CGAL::Cartesian_d<double> > p2(cfg2.GetData().size(), cfg2.GetData().begin(),cfg2.GetData().end());
    CGAL::Point_d< CGAL::Cartesian_d<double> > p3(cfg3.GetData().size(), cfg3.GetData().begin(),cfg3.GetData().end());
    CGAL::Line_d< CGAL::Cartesian_d<double> > l(p2,p3);
    CGAL::Point_d< CGAL::Cartesian_d<double> > projection = l.projection(p1);
    //double e = sqrt(CGAL::squared_distance(p1, projection))/sqrt(CGAL::squared_distance(p1, p2));
    //return sqrt(CGAL::squared_distance(p2, projection));
    double distToP2= sqrt(CGAL::squared_distance(p2, projection));
    double distToP3= sqrt(CGAL::squared_distance(p3, projection));
    if(distToP2<distToP3){
      double distP2ToP3= sqrt(CGAL::squared_distance(p2, p3));
      if(distToP3>distP2ToP3)
        return -distToP2;
    }
    return distToP2;
  }
  
  
  

  //does not work, need to configure to use accual projection
  void divideListAtMidpoint(spillTreeNode<CFGTYPE, WEIGHT> *node, list<VID>& _lSublist, list<VID>& _rSublist, double _overlapDistance){
    //cout<<"in divideListAtMidpoint"<<endl;
    list<VID> rSublist;
    list<VID> lSublist;
    
    findMostDistantPair(node->verticies, node->mostDistantVertex1, node->mostDistantVertex2);
    node->distanceToMidpoint=sqrt(CGAL::squared_distance(toCgalPoint(node->mostDistantVertex1), toCgalPoint(node->mostDistantVertex2)));
    node->distanceToMidpoint/=2;
    for(std::list<VID>::iterator iter = node->verticies->begin();iter != node->verticies->end(); iter++){
      double projection=project(*iter, node->mostDistantVertex1, node->mostDistantVertex2);
      //cout<<" projection="<<projection<<" dist to mpt ="<< node->distanceToMidpoint<<endl;
      if(projection<=node->distanceToMidpoint+_overlapDistance){
        lSublist.push_back(*iter);
	//cout<<"in left"<<endl;
      }
      if(projection>=node->distanceToMidpoint-_overlapDistance){
        rSublist.push_back(*iter);
        //cout<<"in right"<<endl;
      }
    }
   
    _lSublist=lSublist;
    _rSublist=rSublist;
  }


  
  list<double>* getProjections(list<VID>* verticies, VID mdv1, VID mdv2){
    list<double>* projections = new list<double>;
    for(std::list<VID>::iterator iter = verticies->begin(); iter != verticies->end(); iter++){
      projections->push_back(project(*iter, mdv1, mdv2));
    }
    return projections;
  }


  void divideListAtMedian(spillTreeNode<CFGTYPE, WEIGHT> *node, list<VID>& _lSublist, list<VID>& _rSublist, double overlapDistance){
    //cout<<"in divideListAtMedian"<<endl;
    list<VID> rSublist;
    list<VID> lSublist;
    findMostDistantPair(node->verticies, node->mostDistantVertex1, node->mostDistantVertex2);
    //cout<<"Most Distant Pair found"<<endl;
    list<double> *projections= getProjections(node->verticies, node->mostDistantVertex1, node->mostDistantVertex2);
    list<double> sortedProjections = *projections;
    //cout<<"Got Projections found"<<endl;
    sortedProjections.sort();
    std::list<double>::iterator iter = sortedProjections.begin(); 
    for(int i=0; i<((double)sortedProjections.size())/2; i++){
      iter++;
    }
    node->median=*iter;
 
    std::list<double>::iterator projection=projections->begin();
    for(std::list<VID>::iterator iter = node->verticies->begin(); iter != node->verticies->end(); iter++){
      //cout<<"dividing based on projection"<<endl;
      //double projection=project(*iter, node->mostDistantVertex1, node->mostDistantVertex2);
      if(*projection <= node->median+overlapDistance){
        lSublist.push_back(*iter);
      }
      if(*projection >= node->median-overlapDistance){
        rSublist.push_back(*iter);
      }
      projection++;
    }
    //cout<<"divided by median lsize="<<lSublist.size()<<" rsize="<<rSublist.size()<<endl;
    _lSublist=lSublist;
    _rSublist=rSublist;
  }


  void divideList(spillTreeNode<CFGTYPE, WEIGHT> *node, list<VID>& lSublist, list<VID>& rSublist){
    //cout<<"in divideList"<<endl;
    if(divideAtMedian){
      if(isMetricTree){
        divideListAtMedian(node, lSublist, rSublist, 0);
	node->isSpillTreeNode=false;
      }else{
        divideListAtMedian(node, lSublist, rSublist, overlapDistance);
        if(lSublist.size() > maxPerBranch * node->verticies->size() || rSublist.size() > maxPerBranch * node->verticies->size()){
          divideListAtMedian(node, lSublist, rSublist, 0);
	  node->isSpillTreeNode=false;
        }
      }
    }else{
      if(isMetricTree){
	divideListAtMidpoint(node, lSublist, rSublist, 0);
        node->isSpillTreeNode=false;
      }else{
        divideListAtMidpoint(node, lSublist, rSublist, overlapDistance);
        if(lSublist.size() > maxPerBranch * node->verticies->size() || rSublist.size() > maxPerBranch*node->verticies->size()){
	  divideListAtMidpoint(node, lSublist, rSublist, 0);
          node->isSpillTreeNode=false;
	}
      }
    }
  }


  void setDistanceMetric(DistanceMetricMethod* _dmm){
    dmm=_dmm;
  }

  void setEnvironment(Environment* _env){
    env=_env;
  }

  double distance(VID v1, VID v2){
    RoadmapGraph<CFGTYPE,WEIGHT>* pMap = rmap->m_pRoadmap;
    CFGTYPE cfg1 = pMap->GetData(v1);
    CFGTYPE cfg2 = pMap->GetData(v2);
    return dmm->Distance(rmap->GetEnvironment(), cfg1, cfg2);
  }

  double distance(const CFGTYPE &cfg1, const CFGTYPE &cfg2){
    return dmm->Distance(rmap->GetEnvironment(), cfg1, cfg2);
  }


  double distance(VID v1, const CFGTYPE &cfg2){
    RoadmapGraph<CFGTYPE,WEIGHT>* pMap = rmap->m_pRoadmap;
    CFGTYPE cfg1 = pMap->GetData(v1);
    return dmm->Distance(rmap->GetEnvironment(), cfg1, cfg2);
  }

  double euclideanDistance(VID v1, VID v2){
    RoadmapGraph<CFGTYPE,WEIGHT>* pMap = rmap->m_pRoadmap;
    CFGTYPE cfg1 = pMap->GetData(v1);
    CFGTYPE cfg2 = pMap->GetData(v2);
    CGAL::Point_d< CGAL::Cartesian_d<double> > p1(cfg1.GetData().size(), cfg1.GetData().begin(),cfg1.GetData().end());
    CGAL::Point_d< CGAL::Cartesian_d<double> > p2(cfg2.GetData().size(), cfg2.GetData().begin(),cfg2.GetData().end());
    return CGAL::squared_distance(p1, p2);
  }

  VID findMostDistant(VID v, list<VID> *verticies){
    double maxDistance=-1;
    VID maxVertex=verticies->front();
    for(std::list<VID>::iterator list_iter = verticies->begin(); list_iter != verticies->end();list_iter++){
      //cout<<"in findMostDistant loop, iter="<<*list_iter<<endl;
      double dist = euclideanDistance(v, *list_iter);
      if(dist>maxDistance){
	maxDistance = dist;
	maxVertex=*list_iter;
      }
    }
    return maxVertex;
  }

  //change to use existing method if one exists
  void findMostDistantPair(list<VID> *verticies, VID& _v1, VID& _v2){
    /*   
    double maxDist=0;
    for(std::list<VID>::iterator list_iter = verticies->begin(); list_iter != verticies->end();list_iter++){
      cout<<"in findMostDistantPair loop"<<endl;
      VID mdv = findMostDistant(*list_iter, verticies);
      if(distance(*list_iter,mdv)>maxDist){
	maxDist= distance(*list_iter,mdv);
	_v1=*list_iter;
	_v2=mdv;
      }
    } 
    */
     			
    VID v1=findMostDistant(verticies->front(), verticies);
    VID v2=findMostDistant(v1, verticies);
    _v1=v1;
    _v2=v2;
    			
  }

  void setRoadmap(Roadmap<CFGTYPE,WEIGHT>* _rmp){
    rmap=_rmp;
  }


 
  double getMaxDistance(const CFGTYPE &center, list<VID> verticies){
    double max=0;
    for(std::list<VID>::iterator iter = verticies.begin(); iter != verticies.end(); ++iter) {
      double dist = distance(*iter, center);
      max = (max>dist) ? max : dist;
    }
    return max;
  }

  
  CFGTYPE getCenter(list<VID>& verticies){ 
    RoadmapGraph<CFGTYPE,WEIGHT>* pMap = rmap->m_pRoadmap;
    CFGTYPE center;
    for(std::list<VID>::iterator iter = verticies.begin(); iter != verticies.end(); ++iter) {
      center.add(center,pMap->GetData(*iter));
    }
    center.divide(center, verticies.size());
    return center;
  }
  

  void create(list<VID> &verticies){
    //cout<<"in createm list size ="<<verticies.size()<<endl;
    //cout<<"max list size ="<<maxLeafSize<<endl;
   spillTreeNode<CFGTYPE, WEIGHT> *temp= new spillTreeNode<CFGTYPE, WEIGHT>(&verticies);
   //list<VID> *v3=temp->verticies;
   //cout<<"getting center"<<endl;
   temp->center=getCenter(verticies);
   //cout<<"getting max dist"<<endl;
   temp->maxDistance=getMaxDistance(temp->center, verticies);
   VID _root = tree->AddVertex(temp);
   if(verticies.size()>maxLeafSize){
     //cout<<"in if stmt"<<endl;
      list<VID> lSublist;
      list<VID> rSublist;
      divideList(temp,lSublist,rSublist);
      //cout<<"divided lsize="<<lSublist.size()<<" rsize="<<rSublist.size()<<endl;
      create(lSublist);
      VID lChild=root;
      //list<VID> *v2=getNodeDataStructure(lChild)->verticies;
      create(rSublist);
      VID rChild=root;
      tree->AddEdge(_root, lChild);
      tree->AddEdge(_root, rChild);
    }    
    root = _root;
    //list<VID> *v4=getNodeDataStructure(_root)->verticies;
  }



  
  //need to change midpoint one
  bool isToLeftOfPartition(const CFGTYPE &cfg, VID node){
    if(divideAtMedian){
      return (project(cfg,getNodeDataStructure(node)->mostDistantVertex1,getNodeDataStructure(node)->mostDistantVertex2)<getNodeDataStructure(node)->median);
    }else{
      return (project(cfg,getNodeDataStructure(node)->mostDistantVertex1,getNodeDataStructure(node)->mostDistantVertex2)<getNodeDataStructure(node)->distanceToMidpoint);
    }
  }

  bool isToLeftOfPartition(VID vertex, VID node){
    RoadmapGraph<CFGTYPE,WEIGHT>* pMap = rmap->m_pRoadmap;
    CFGTYPE cfg = pMap->GetData(vertex);
    return isToLeftOfPartition(cfg, node);
  }
  
  void addVertexToLeaf(VID newVertex, VID leaf){
    getNodeDataStructure(leaf)->verticies->push_back(newVertex);
    if(getNodeDataStructure(leaf)->verticies->size()>maxLeafSize){
      list<VID> lSublist;
      list<VID> rSublist;
      divideList(getNodeDataStructure(leaf),lSublist,rSublist);
      //cout<<"divided lsize="<<lSublist.size()<<" rsize="<<rSublist.size()<<endl;
      createChild(leaf, &lSublist);
      createChild(leaf, &rSublist);
    }
  }

    
  void addVertex(VID newVertex, VID node){
    getNodeDataStructure(node)->maxDistance = max(distance(newVertex, getNodeDataStructure(node)->center), getNodeDataStructure(node)->maxDistance);
    if(isLeaf(node)){
      addVertexToLeaf(newVertex, node);
    }else{
      if(isToLeftOfPartition(newVertex, node)){
	addVertex(newVertex, getLChild(node));
      }else{
        addVertex(newVertex, getRChild(node));
      }
    }
    cout<<"done adding vertex"<<endl;
  }
    


  void addVertex(VID newVertex){
    //cout<<"in add vertex";
    addVertex(newVertex, root);
  }

  double Euclidean(VID v1, VID v2){
    RoadmapGraph<CFGTYPE,WEIGHT>* pMap = rmap->m_pRoadmap;
    CFGTYPE cfg1 = pMap->GetData(v1);
    CFGTYPE cfg2 = pMap->GetData(v2);
    return Euclidean(cfg1,cfg2);
  }

  
  double Euclidean(const CFGTYPE &cfg1, const CFGTYPE &cfg2){
    return Euclidean(cfg1.GetData(),cfg2.GetData());
  }
  
 
  double Euclidean( const vector<double>& v1, const vector<double>& v2){
    double diff, dist = 0.0;
    int d;
    if(v1.size() != v2.size() ){
      cout << "Dimensons of points are not consistent when calculating Euclidean Distance"
           << endl;
      return -1;
    }else{
      d = v1.size();
      for(int i = 0; i < d; ++i){
	diff = v1[i] - v2[i];
	dist += diff * diff;
      }
      //return .5;
      return sqrt(dist);
    } 
  }
  
};

#endif 

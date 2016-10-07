#ifndef _ST_SPILL_TREE_CLASS_H_
#define _ST_SPILL_TREE_CLASS_H_

#include "Graph.h"
#include <vector>
#include "CGAL/Cartesian_d.h"
#include <CGAL/constructions_d.h>
using namespace std;

class Input;
class Environment;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup NeighborhoodFinderUtils
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
template<typename CFGTYPE, typename WEIGHT>
class vertexDistance{
public:
  typedef typename RoadmapGraph<CFGTYPE, WEIGHT>::VID VID;

  VID vertex;
  double distance;

  vertexDistance(VID _vertex, double _distance){
    vertex=_vertex;
    distance=_distance;
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup NeighborhoodFinderUtils
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
template<typename CFGTYPE, typename WEIGHT>
class spillTreeNode{

 public:
  typedef typename RoadmapGraph<CFGTYPE, WEIGHT>::VID VID;
  vector<VID> *verticies;
  vector<VID> verticiesObj;
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


  spillTreeNode(vector<VID> &_verticies){
    verticies=&_verticies;
    isSpillTreeNode=true;
  }


  spillTreeNode(vector<VID> *_verticies){
    verticiesObj=*_verticies;
    verticies=&verticiesObj;
    isSpillTreeNode=true;
  }
};


////////////////////////////////////////////////////////////////////////////////
/// @ingroup NeighborhoodFinderUtils
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
template<typename CFGTYPE, typename WEIGHT>
class spillTree{

 public:
  typedef stapl::sequential::graph<stapl::DIRECTED, stapl::NONMULTIEDGES, spillTreeNode<CFGTYPE, WEIGHT>* > graphType;
  typedef typename RoadmapGraph<CFGTYPE, WEIGHT>::VID VID;

  int maxLeafSize;
  double overlapDistance;
  double maxPerBranch;
  bool divideAtMedian;
  Roadmap<CFGTYPE,WEIGHT>* rmap;
  shared_ptr<DistanceMetricMethod> dmm;
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


  bool isLeaf(VID node){
    //typename RoadmapGraph<CFGTYPE, WEIGHT>::vertex_iterator vi= tree->find_vertex(node);
    //return ((*vi).size()==0);
   return((*(tree->find_vertex(node))).size()==0);
    //return true;
  }

  VID getLChild(VID node){
    vector<VID> successors;
    tree->get_successors(node, successors);
    return successors.front();
  }

  VID getRChild(VID node){
    vector<VID> successors;
    tree->get_successors(node, successors);
    return successors.back();
  }

  spillTreeNode<CFGTYPE,WEIGHT>* getNodeDataStructure(VID node){
    return (*(tree->find_vertex(node))).property();
  }

  VID createChild(VID parent, vector<VID> *vertexList){
    spillTreeNode<CFGTYPE,WEIGHT> *newNode = new spillTreeNode<CFGTYPE, WEIGHT>(vertexList);
    VID newNodeVID = tree->add_vertex(newNode);
    tree->add_edge(parent, newNodeVID);
    return newNodeVID;
  }



  void splitNode(VID node){
    std::vector<VID> rSublist;
    std::vector<VID> lSublist;
    rSublist.reserve(getNodeDataStructure(node)->verticies->size());
    lSublist.reserve(getNodeDataStructure(node)->verticies->size());
    divideList(getNodeDataStructure(node),lSublist,rSublist);
    createChild(node,lSublist);
    createChild(node,rSublist);
  }



  vector<vertexDistance<CFGTYPE, WEIGHT> >* addIfClosest(const CFGTYPE &queryCFG, VID v,  vector<vertexDistance<CFGTYPE, WEIGHT> > *closest, int k){
    RoadmapGraph<CFGTYPE,WEIGHT>* pMap = rmap->m_pRoadmap;
    if((*(pMap->find_vertex(v))).property() == queryCFG && !includeQueryCFG){
      return closest;
    }
    //vertexDistance<CFGTYPE, WEIGHT> distance_v(v, distance(v,queryCFG));
    //typename std::vector<vertexDistance<CFGTYPE, WEIGHT> >::iterator iter;
    vertexDistance<CFGTYPE, WEIGHT> distance_v(v, distance(v,queryCFG));
    for(typename std::vector<vertexDistance<CFGTYPE, WEIGHT> >::iterator iter = closest->begin(); iter != closest->end(); iter++){
      if(v==(*iter).vertex)
        return closest;
      if(distance_v.distance<(*iter).distance){
        closest->insert(iter,distance_v);
        if((int)closest->size()>k)
          closest->pop_back();
        return closest;
      }
    }
    if((int)closest->size()<k)
      closest->push_back(distance_v);
    return closest;
  }

  //replace with existing method
  void findClosest(const CFGTYPE &queryCFG, int k, VID nodeVID, vector<vertexDistance<CFGTYPE, WEIGHT> > *closest){
    spillTreeNode<CFGTYPE,WEIGHT>* node = getNodeDataStructure(nodeVID);
    for(typename std::vector<VID>::iterator iter = node->verticies->begin(); iter != node->verticies->end(); iter++){
      closest=addIfClosest(queryCFG, *iter, closest, k);
    }
  }

  bool canPrune(const CFGTYPE &queryCFG, int k, VID node, const vector<vertexDistance<CFGTYPE, WEIGHT> > *closest){
    return ((closest->back().distance)<(distance(getNodeDataStructure(node)->center,queryCFG)-getNodeDataStructure(node)->maxDistance));
  }

  void query(const CFGTYPE &queryCFG, int k, VID node, vector<vertexDistance<CFGTYPE, WEIGHT> > *closest){
    if(isLeaf(node)){
      //vector<VID> *v2=getNodeDataStructure(node)->verticies;
      //cout<<"finding closest"<<endl;
      findClosest(queryCFG, k, node, closest);
    }else{
      if(isToLeftOfPartition(queryCFG, node)){
        //cout<<"query left"<<endl;
        query(queryCFG, k, getLChild(node), closest);
        //cout<<"done query left"<<endl;
        if((int)closest->size()<k || (!(getNodeDataStructure(node)->isSpillTreeNode) && !canPrune(queryCFG,k, getRChild(node), closest))){
   query(queryCFG, k, getRChild(node), closest);
   //cout<<"not pruined"<<endl;
 }
      }else{
        //cout<<"query right"<<endl;
        query(queryCFG, k, getRChild(node), closest);
        //cout<<"done query right"<<endl;
        if((int)closest->size()<k || (!(getNodeDataStructure(node)->isSpillTreeNode) && !canPrune(queryCFG,k, getLChild(node), closest))){
   //cout<<"not pruined"<<endl;
          query(queryCFG, k, getLChild(node), closest);
        }
      }
    }
  }

  vector<VID>* query(const CFGTYPE &v, int k){
    vector<vertexDistance<CFGTYPE,WEIGHT> > *closestDistances = new vector<vertexDistance<CFGTYPE,WEIGHT> >();
    closestDistances->reserve(k);
    query(v, k, root, closestDistances);
    vector<VID> *closest = new vector<VID>();
    closest->reserve(k);
    for(typename std::vector<vertexDistance<CFGTYPE,WEIGHT> >::iterator iter = closestDistances->begin(); iter != closestDistances->end(); iter++)
      closest->push_back((*iter).vertex);
    return closest;
  }

  vector<VID>* query(VID v, int k){
    RoadmapGraph<CFGTYPE,WEIGHT>* pMap = rmap->m_pRoadmap;
    return query((*(pMap->find_vertex(v))).property(), k);
  }




  double project(VID v1, VID v2, VID v3){
    RoadmapGraph<CFGTYPE,WEIGHT>* pMap = rmap->m_pRoadmap;
    CFGTYPE cfg1 = (*(pMap->find_vertex(v1))).property();
    CFGTYPE cfg2 = (*(pMap->find_vertex(v2))).property();
    CFGTYPE cfg3 = (*(pMap->find_vertex(v3))).property();
    return project(cfg1, cfg2, cfg3);
  }


  double project(const CFGTYPE &cfg, VID v2, VID v3){
    RoadmapGraph<CFGTYPE,WEIGHT>* pMap = rmap->m_pRoadmap;
    CFGTYPE cfg2 = (*(pMap->find_vertex(v2))).property();
    CFGTYPE cfg3 = (*(pMap->find_vertex(v3))).property();
    return project(cfg, cfg2, cfg3);
  }

  CGAL::Point_d< CGAL::Cartesian_d<double> > toCgalPoint(VID v){
    RoadmapGraph<CFGTYPE,WEIGHT>* pMap = rmap->m_pRoadmap;
    CFGTYPE cfg = (*(pMap->find_vertex(v))).property();
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
  void divideListAtMidpoint(spillTreeNode<CFGTYPE, WEIGHT> *node, vector<VID>& _lSublist, vector<VID>& _rSublist, double _overlapDistance){
    //cout<<"in divideListAtMidpoint"<<endl;
    vector<VID> rSublist;
    vector<VID> lSublist;
    rSublist.reserve(node->verticies->size());
    lSublist.reserve(node->verticies->size());
    findMostDistantPair(node->verticies, node->mostDistantVertex1, node->mostDistantVertex2);
    node->distanceToMidpoint=sqrt(CGAL::squared_distance(toCgalPoint(node->mostDistantVertex1), toCgalPoint(node->mostDistantVertex2)));
    node->distanceToMidpoint/=2;
    for(typename std::vector<VID>::iterator iter = node->verticies->begin();iter != node->verticies->end(); iter++){
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



  vector<double>* getProjections(vector<VID>* verticies, VID mdv1, VID mdv2){
    vector<double>* projections = new vector<double>;
    projections->reserve(verticies->size());
    for(typename std::vector<VID>::iterator iter = verticies->begin(); iter != verticies->end(); iter++){
      projections->push_back(project(*iter, mdv1, mdv2));
    }
    return projections;
  }


  void divideListAtMedian(spillTreeNode<CFGTYPE, WEIGHT> *node, vector<VID>& _lSublist, vector<VID>& _rSublist, double overlapDistance){
    //cout<<"in divideListAtMedian"<<endl;
    vector<VID> rSublist;
    vector<VID> lSublist;
    rSublist.reserve(node->verticies->size());
    lSublist.reserve(node->verticies->size());
    findMostDistantPair(node->verticies, node->mostDistantVertex1, node->mostDistantVertex2);
    //cout<<"Most Distant Pair found"<<endl;
    vector<double> *projections= getProjections(node->verticies, node->mostDistantVertex1, node->mostDistantVertex2);
    vector<double> sortedProjections = *projections;
    //cout<<"Got Projections found"<<endl;
    sort(sortedProjections.begin(),sortedProjections.end());
    std::vector<double>::iterator iter = sortedProjections.begin();
    for(int i=0; i<((double)sortedProjections.size())/2; i++){
      iter++;
    }
    node->median=*iter;

    std::vector<double>::iterator projection=projections->begin();
    for(typename std::vector<VID>::iterator iter = node->verticies->begin(); iter != node->verticies->end(); iter++){
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


  void divideListByDMM(spillTreeNode<CFGTYPE, WEIGHT> *node, vector<VID>& lSublist, vector<VID>& rSublist, double od){
    //vector<VID> rSublist;
    //vector<VID> lSublist;
    rSublist.clear();
    lSublist.clear();
    rSublist.reserve(node->verticies->size());
    lSublist.reserve(node->verticies->size());
    findMostDistantPair(node->verticies, node->mostDistantVertex1, node->mostDistantVertex2);
    for(typename std::vector<VID>::iterator iter = node->verticies->begin();iter != node->verticies->end(); iter++){
      //CFGTYPE projection=project(*iter, node->mostDistantVertex1, node->mostDistantVertex2);
      //cout<<" projection="<<projection<<" dist to mpt ="<< node->distanceToMidpoint<<endl;
      if(distance(*iter, node->mostDistantVertex1) <= distance(*iter, node->mostDistantVertex2) + 2*od){
        lSublist.push_back(*iter);
        //cout<<"in left"<<endl;
      }
      if(distance(*iter, node->mostDistantVertex2) <= distance(*iter, node->mostDistantVertex1) + 2*od){
        rSublist.push_back(*iter);
        //cout<<"in right"<<endl;
      }
    }
    //_lSublist=lSublist;
    //_rSublist=rSublist;
  }


  void divideList(spillTreeNode<CFGTYPE, WEIGHT> *node, vector<VID>& lSublist, vector<VID>& rSublist){
    //cout<<"in divideList"<<endl;
    /*
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
    */
    if(isMetricTree){
      divideListByDMM(node, lSublist, rSublist, 0);
      node->isSpillTreeNode=false;
    }else{
      divideListByDMM(node, lSublist, rSublist, overlapDistance);
      if(lSublist.size() > maxPerBranch * node->verticies->size() || rSublist.size() > maxPerBranch*node->verticies->size()){
        divideListByDMM(node, lSublist, rSublist, 0);
        node->isSpillTreeNode=false;
      }
    }
  }


  void setDistanceMetric(shared_ptr<DistanceMetricMethod> _dmm){
    dmm=_dmm;
  }

  void setEnvironment(Environment* _env){
    env=_env;
  }

  double distance(VID v1, VID v2){
    RoadmapGraph<CFGTYPE,WEIGHT>* pMap = rmap->m_pRoadmap;
    CFGTYPE cfg1 = (*(pMap->find_vertex(v1))).property();
    CFGTYPE cfg2 = (*(pMap->find_vertex(v2))).property();
    return dmm->Distance(rmap->GetEnvironment(), cfg1, cfg2);
  }

  double distance(const CFGTYPE &cfg1, const CFGTYPE &cfg2){
    return dmm->Distance(rmap->GetEnvironment(), cfg1, cfg2);
  }


  double distance(VID v1, const CFGTYPE &cfg2){
    RoadmapGraph<CFGTYPE,WEIGHT>* pMap = rmap->m_pRoadmap;
    CFGTYPE cfg1 = (*(pMap->find_vertex(v1))).property();
    return dmm->Distance(rmap->GetEnvironment(), cfg1, cfg2);
  }

  double euclideanDistance(VID v1, VID v2){
    RoadmapGraph<CFGTYPE,WEIGHT>* pMap = rmap->m_pRoadmap;
    CFGTYPE cfg1 = (*(pMap->find_vertex(v1))).property();
    CFGTYPE cfg2 = (*(pMap->find_vertex(v2))).property();
    CGAL::Point_d< CGAL::Cartesian_d<double> > p1(cfg1.GetData().size(), cfg1.GetData().begin(),cfg1.GetData().end());
    CGAL::Point_d< CGAL::Cartesian_d<double> > p2(cfg2.GetData().size(), cfg2.GetData().begin(),cfg2.GetData().end());
    return CGAL::squared_distance(p1, p2);
  }

  VID findMostDistant(VID v, vector<VID> *verticies){
    double maxDistance=-1;
    VID maxVertex=verticies->front();
    for(typename std::vector<VID>::iterator list_iter = verticies->begin(); list_iter != verticies->end();list_iter++){
      //cout<<"in findMostDistant loop, iter="<<*list_iter<<endl;
      double dist = distance(v, *list_iter);
      if(dist>maxDistance){
 maxDistance = dist;
 maxVertex=*list_iter;
      }
    }
    return maxVertex;
  }

  //change to use existing method if one exists
  void findMostDistantPair(vector<VID> *verticies, VID& _v1, VID& _v2){
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



  double getMaxDistance(const CFGTYPE &center, vector<VID> verticies){
    double max=0;
    for(typename std::vector<VID>::iterator iter = verticies.begin(); iter != verticies.end(); ++iter) {
      double dist = distance(*iter, center);
      max = (max>dist) ? max : dist;
    }
    return max;
  }


  CFGTYPE getCenter(vector<VID>& verticies){
    RoadmapGraph<CFGTYPE,WEIGHT>* pMap = rmap->m_pRoadmap;
    CFGTYPE center;
    for(typename std::vector<VID>::iterator iter = verticies.begin(); iter != verticies.end(); ++iter) {
      CFGTYPE tmp = (*(pMap->find_vertex(*iter))).property();
      center.add(center,tmp);
    }
    center.divide(center, verticies.size());
    return center;
  }


  void create(vector<VID> &verticies){
    //cout<<"in createm list size ="<<verticies.size()<<endl;
    //cout<<"max list size ="<<maxLeafSize<<endl;
   spillTreeNode<CFGTYPE, WEIGHT> *temp= new spillTreeNode<CFGTYPE, WEIGHT>(&verticies);
   //list<VID> *v3=temp->verticies;
   //cout<<"getting center"<<endl;
   temp->center=getCenter(verticies);
   //cout<<"getting max dist"<<endl;
   temp->maxDistance=getMaxDistance(temp->center, verticies);
   VID _root = tree->add_vertex(temp);
   if((int)verticies.size()>maxLeafSize){
     //cout<<"in if stmt"<<endl;
      vector<VID> lSublist;
      vector<VID> rSublist;
      rSublist.reserve(verticies.size());
      lSublist.reserve(verticies.size());
      divideList(temp,lSublist,rSublist);
      //cout<<"divided lsize="<<lSublist.size()<<" rsize="<<rSublist.size()<<endl;
      create(lSublist);
      VID lChild=root;
      //list<VID> *v2=getNodeDataStructure(lChild)->verticies;
      create(rSublist);
      VID rChild=root;
      tree->add_edge(_root, lChild);
      tree->add_edge(_root, rChild);
    }
    root = _root;
    //list<VID> *v4=getNodeDataStructure(_root)->verticies;
  }




  //need to change midpoint one
  bool isToLeftOfPartition(const CFGTYPE &cfg, VID node){
    if(divideAtMedian){
      return (project(cfg,getNodeDataStructure(node)->mostDistantVertex1,getNodeDataStructure(node)->mostDistantVertex2)<getNodeDataStructure(node)->median);
    }else{
      return (distance(getNodeDataStructure(node)->mostDistantVertex1, cfg)<=distance(getNodeDataStructure(node)->mostDistantVertex2, cfg));
      //return (project(cfg,getNodeDataStructure(node)->mostDistantVertex1,getNodeDataStructure(node)->mostDistantVertex2)<getNodeDataStructure(node)->distanceToMidpoint);
    }
  }

  bool isToLeftOfPartition(VID vertex, VID node){
    RoadmapGraph<CFGTYPE,WEIGHT>* pMap = rmap->m_pRoadmap;
    CFGTYPE cfg = (*(pMap->find_vertex(vertex))).property();
    return isToLeftOfPartition(cfg, node);
  }

  void addVertexToLeaf(VID newVertex, VID leaf){
    getNodeDataStructure(leaf)->verticies->push_back(newVertex);
    if((int)getNodeDataStructure(leaf)->verticies->size()>maxLeafSize){
      vector<VID> lSublist;
      vector<VID> rSublist;
      rSublist.reserve(getNodeDataStructure(leaf)->verticies->size());
      lSublist.reserve(getNodeDataStructure(leaf)->verticies->size());
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
    //cout<<"done adding vertex"<<endl;
  }



  void addVertex(VID newVertex){
    //cout<<"in add vertex";
    addVertex(newVertex, root);
  }

  double Euclidean(VID v1, VID v2){
    RoadmapGraph<CFGTYPE,WEIGHT>* pMap = rmap->m_pRoadmap;
    CFGTYPE cfg1 = (*(pMap->find_vertex(v1))).property();
    CFGTYPE cfg2 = (*(pMap->find_vertex(v2))).property();
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

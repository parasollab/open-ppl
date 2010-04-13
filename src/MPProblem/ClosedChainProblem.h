#ifndef ClosedChainProblem_h
#define ClosedChainProblem_h

/*
#include "SwitchDefines.h"

#include "OBPRMDef.h"
#include "Roadmap.h"
//#include "Input.h"
#include "Clock_Class.h"
#include "Stat_Class.h"
#include "CollisionDetection.h"
#include "ConnectMap.h"
#include "DistanceMetrics.h"
#include "Weight.h"
#include "LocalPlanners.h"
#include "MapGenerator.h"
#include "CfgTypes.h"
#include "boost/lambda/lambda.hpp"
*/
#include "MPProblem.h"
#include "Environment.h"
#include "boost/lambda/lambda.hpp"
#include "MPProblem/ClosedChainProblem.h"
#include "VirtualLink.h"

//Check for stl equivalent
template <typename T, typename U, typename V>
struct triple
{
  triple() {}
  triple(const T& t, const U& u, const V& v) : first(t), second(u), third(v) {}
  ~triple() {}

  T first;
  U second;
  V third;
};

template<typename F, typename T>
struct first_is : public unary_function<T, bool>
{
  first_is(const F& f) : first(f) {}
  ~first_is() {}
  bool operator()(const T& t) const 
  {
    return first == t.first;
  }

  F first;
};

class ClosedChainProblem:public MPProblem{
 public:
  //todo: make private
  vector<Link *> g_baseLinks;
  vector<Link *> g_loopRoots;
  vector<vector<int> > g_loopIDs;
  vector<vector<Link *> > g_loopLinks;
  vector<pair<Link *, Link *> > g_cfgJoints;
  vector<triple<int,int,int> > g_earJoints;
  vector<Link*> g_ear_roots;
  vector<int> g_ear_rootIDs;
  vector<vector<int> > g_ears;
  vector<vector<int> > g_non_ears;
  /*
  vector<Link *> baseLinks;
  vector<Link *> loopRoots;
  vector<vector<int> > loopIDs;
  vector<vector<Link *> > loopLinks;
  vector<pair<Link *, Link *> > cfgJoints;
  vector<triple<int,int,int> > earJoints;
  vector<Link*> ear_roots;
  vector<int> ear_rootIDs;
  vector<vector<int> > ears;
  vector<vector<int> > non_ears;
  */
  //Input input;
  //Stat_Class Stats;

  /*
  XMLNodeReader& stripClosedChainNodes(XMLNodeReader& in_Node){
    return in_Node;
  }
  */

  ClosedChainProblem(XMLNodeReader& in_Node):MPProblem(in_Node){ParseXML(in_Node);}
  void ParseXML(XMLNodeReader& in_Node);
  bool ParseRealLink(ifstream &fin);
  bool ParseVirtualLink(ifstream &fin);
  bool ParseLoop(ifstream &fin);
  bool ParseCfgJoints(ifstream &fin);
  bool ParseEarJoints(ifstream &fin);
  //void seen_both_children(Link* link, set<Link*>& seen, set<Link*>& implied);
  //void partition_loop(Link* link, set<Link*>& links_seen, vector<int>& seen, vector<int>& unseen);
  //void get_actual_links_of(Link* link, vector<int>& ids);
  //Link* get_parent_of(Link* link, const vector<int>& ear);  use function in link class instead
  //void  ConfigEar(Environment* env, Link* ear_root, Link* loop_root, double base_link_angle);
  double MyCalculateJointAngle(Environment* env, Link* link1, Link* link2);
  bool ParseLinksFile(const char* linksFileName);
  void PrintConfiguration(Environment* env, ostream & ofPath);
  //void ConfigBase(Environment* env, const vector<double>& v = vector<double>(6, 0));
  void ConfigBase(Environment* env, const vector<double>& v);
  void ConfigBase(Environment* env){ConfigBase(env, vector<double>(6, 0));}
  void ConfigEnvironment(Environment* env);
  void ConfigEar(Environment* env, Link* ear_root, vector<int>& actual_ear_links, int base_link_id, double base_link_angle);
  void ConfigEar(Environment* env, Link* ear_root, Link* loop_root);
  void ConfigEar(Environment* env, Link* ear_root, Link* loop_root, double base_link_angle);
  void PrintEarJointCoords(Environment* env, Link* ear_root);
  /*
  friend void seen_both_children(Link* link, set<Link*>& seen, set<Link*>& implied);
  friend void partition_loop(Link* link, set<Link*>& links_seen, vector<int>& seen, vector<int>& unseen);
  friend void get_actual_links_of(Link* link, vector<int>& ids);
  friend Link* get_parent_of(Link* link, const vector<int>& ear);
  */
};

#endif

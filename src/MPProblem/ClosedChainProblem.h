#ifndef ClosedChainProblem_h
#define ClosedChainProblem_h

#include "MPProblem.h"
#include "Environment.h"
#include "boost/lambda/lambda.hpp"
#include "MPProblem/ClosedChainProblem.h"
#include "VirtualLink.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningUniverse
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
class ClosedChainProblem:public MPProblem{
 public:
  vector<Link *> g_baseLinks;
  vector<Link *> g_loopRoots;
  vector<pair<Link *, Link *> > g_cfgJoints;
  vector<triple<int,int,int> > g_earJoints;
  vector<Link*> g_ear_roots;
  vector<vector<int> > g_ears;
  vector<vector<int> > g_non_ears;

  ClosedChainProblem(XMLNode& in_Node);
  ~ClosedChainProblem();

  void ParseXML(XMLNode& in_Node);
  bool ParseRealLink(ifstream &fin);
  bool ParseVirtualLink(ifstream &fin);
  bool ParseLoop(ifstream &fin);
  bool ParseCfgJoints(ifstream &fin);
  bool ParseEarJoints(ifstream &fin);
  bool ParseLinksFile(const char* linksFileName);

  double MyCalculateJointAngle(Environment* env, Link* link1, Link* link2);

  void PrintConfiguration(Environment* env, ostream & ofPath);
  void PrintEarJointCoords(Environment* env, Link* ear_root);

  void ConfigBase(Environment* env, const vector<double>& v);
  void ConfigBase(Environment* env){ConfigBase(env, vector<double>(6, 0));}
  void ConfigureRobot(Environment* env);
  void ConfigEar(Environment* env, Link* ear_root, vector<int>& actual_ear_links, int base_link_id, double base_link_angle);
  void ConfigEar(Environment* env, Link* ear_root, Link* loop_root);
  void ConfigEar(Environment* env, Link* ear_root, Link* loop_root, double base_link_angle);
};

#endif

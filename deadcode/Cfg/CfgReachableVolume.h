#ifndef CfgReachableVolume_h
#define CfgReachableVolume_h


#include "Cfg.h"

typedef Vector3d Vector3D;

class ReachableVolumeRobot;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ReachableCfgs
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
class CfgReachableVolume : public Cfg {
 public:
  CfgReachableVolume();
  CfgReachableVolume(const Cfg&c);
  CfgReachableVolume(const CfgReachableVolume&c);
  ~CfgReachableVolume(){};
  virtual const string GetName() const;


  void loadTreeFiles();
  virtual void GetRandomCfg(double R, double rStep){}
  virtual void GetRandomCfg(Environment* _env,shared_ptr<Boundary> _bb);
  virtual void GetRandomCfg(Environment* _env);
  //vector<Robot> GetRobots(int _numJoints);
  //vector<Robot> GetRobots(vector<Robot> &_robots, const Environment* _env);
   template<class DistanceMetricPointer>
     void GetRandomRay(double _incr, DistanceMetricPointer _dm, bool _norm=true){}

 private:
  shared_ptr<vector<shared_ptr<ReachableVolumeRobot> > >  m_reachableVolumeRobots;  //an instance of m_reachableVolumeRobot for each robot
  double m_numOfJoints;
};

#endif

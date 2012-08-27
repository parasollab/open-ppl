#ifndef MINKOWSKIDISTANCE_H_
#define MINKOWSKIDISTANCE_H_

#include "DistanceMetricMethod.h"
//#include "boost/utility/enable_if.hpp"
#include "IsClosedChain.h"
#include "Cfg_free_tree.h"
#include "Cfg_fixed_tree.h"

class MinkowskiDistance : public DistanceMetricMethod {
  public:
    MinkowskiDistance(double _r1 = 3, double _r2 = 3, double _r3 = 1.0/3, bool _normalize = false);
    MinkowskiDistance(XMLNodeReader& _node, MPProblem* _problem, bool _warn = true, bool _parse = true);
    virtual ~MinkowskiDistance();
    virtual void PrintOptions(ostream& _os) const;
    virtual double Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2);
    virtual void ScaleCfg(Environment* _env, double _length, Cfg& _o, Cfg& _c, bool _normalizeOrientation = true);

  protected:
    //default implementation
    template<typename Enable>
      Cfg* DifferenceCfg(const Cfg& _c1, const Cfg& _c2,
          typename boost::disable_if<IsClosedChain<Enable> >::type* _dummy = 0) {
        Cfg* c = _c1.CreateNewCfg();
        c->subtract(_c1, _c2);
        return c;
      }
    //reachable distance implementation
    template<typename Enable>
      Cfg* DifferenceCfg(const Cfg& _c1, const Cfg& _c2,
          typename boost::enable_if<IsClosedChain<Enable> >::type* _dummy = 0) {
        vector<double> _v1 = _c1.GetData();
        vector<double> _v2 = _c2.GetData();
        if(_v1.size() != _v2.size()) { 
          cout << "ERROR in MinkowskiDistance::DifferenceCfg, _c1 dofs (" << _v1.size() << ") != _c2 dofs (" << _v2.size() << ")\n"; 
          exit(-1);
        }
        if(_v1.size() == CfgType::GetNumOfJoints()) {
          Cfg_fixed_tree c1Linkage;
          c1Linkage.SetData(_v1);
          Cfg_fixed_tree c2Linkage;
          c2Linkage.SetData(_v2);
          Cfg* c = c1Linkage.CreateNewCfg();
          c->subtract(c1Linkage, c2Linkage);
          return c;
        } else {
          Cfg_free_tree c1Linkage;
          c1Linkage.SetData(_v1);
          Cfg_free_tree c2Linkage;
          c2Linkage.SetData(_v2);
          Cfg* c = c1Linkage.CreateNewCfg();
          c->subtract(c1Linkage, c2Linkage);
          return c;
        }
      }

    double PositionDistance(Environment* _env, const Cfg& _c);
    double OrientationDistance(const Cfg& _c);

    /**Power factors for Minkowski Distance **/
    double m_r1; ///<For position part.
    double m_r2; ///<For rotation part.
    double m_r3; ///<For calculating root.
    bool m_normalize;
};

#endif

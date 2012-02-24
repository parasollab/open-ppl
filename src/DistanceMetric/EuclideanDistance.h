#ifndef EUCLIDEANDISTANCE_H_
#define EUCLIDEANDISTANCE_H_

#include "DistanceMetricMethod.h"
#include "CfgTypes.h"
#include "Cfg_free_tree.h"
#include "MPProblem.h"
#include "boost/utility/enable_if.hpp"

/**This computes the euclidean distance between two cfgs.  This class is 
  *derived off of DistanceMetricMethod.
  */
class EuclideanDistance : public DistanceMetricMethod {
  public:
    EuclideanDistance();
    EuclideanDistance(XMLNodeReader& _node, MPProblem* _problem, bool _warn = true);
    virtual ~EuclideanDistance();
    virtual void PrintOptions(ostream& _os) const;
    virtual double Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2);
    virtual void ScaleCfg(Environment* _env, double _length, Cfg& _o, Cfg& _c, bool _norm=true);

  protected:
    //default implementation
    template<typename Enable>
      double ScaledDistance(Environment* _env, const Cfg& _c1, const Cfg& _c2, double _sValue,
          typename boost::disable_if<IsClosedChain<Enable> >::type* _dummy = 0){
        CfgType tmp;
        return ScaledDistanceImpl(_env, _c1, _c2, _sValue, tmp);
      }

    //reachable distance implementation
    template<typename Enable>
      double ScaledDistance(Environment* _env, const Cfg& _c1, const Cfg& _c2, double _sValue,
          typename boost::enable_if<IsClosedChain<Enable> >::type* _dummy = 0){
        vector<double> _v1 = _c1.GetData();
        if((int)_v1.size() == CfgType::GetNumOfJoints()) 
          _v1.insert(_v1.begin(), 6, 0);
        vector<double> _v2 = _c2.GetData();
        if((int)_v2.size() == CfgType::GetNumOfJoints()) 
          _v2.insert(_v2.begin(), 6, 0);
        Cfg_free_tree c1Linkage(_v1);
        Cfg_free_tree c2Linkage(_v2);
        Cfg_free_tree tmp;
        return ScaledDistanceImpl(_env, c1Linkage, c2Linkage, _sValue, tmp);
      }

  private:
    double ScaledDistanceImpl(Environment* _env, const Cfg& _c1, const Cfg& _c2, double _sValue, Cfg& tmp);
};

#endif

#ifndef RMSDDISTANCE_H_
#define RMSDDISTANCE_H_

#include "DistanceMetricMethod.h"

class RMSDDistance : public DistanceMetricMethod {
  public:
    RMSDDistance();
    RMSDDistance(XMLNodeReader& _node, MPProblem* _problem, bool _warn = true);
    virtual ~RMSDDistance();

    virtual double Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2);

  protected:
    virtual vector<Vector3D> GetCoordinatesForRMSD(const Cfg& _c, Environment* _env);
    double RMSD(vector<Vector3D> _x, vector<Vector3D> _y, int _dim);
};

#endif

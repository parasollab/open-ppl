#ifndef RSMDDISTANCE_H_
#define RSMDDISTANCE_H_

#include "EuclideanDistance.h"


class RSMDDistance : public EuclideanDistance {
  public:
    RSMDDistance();
    RSMDDistance(XMLNodeReader& _node, MPProblem* _problem, bool _warn = true);
    virtual ~RSMDDistance();
    virtual double Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2);
    virtual vector<Vector3D> GetCoordinatesForRMSD(const Cfg& _c, Environment* _env);
    double RMSD(vector<Vector3D> _x, vector<Vector3D> _y, int _dim);
  
};

#endif

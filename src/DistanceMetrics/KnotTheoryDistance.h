#ifndef KNOTTHEORYDISTANCE_H_
#define KNOTTHEORYDISTANCE_H_

#include "DistanceMetricMethod.h"

/**This computes the knot theory distance between two cfgs.  Taking the topological information into perspective.
  */
template<class MPTraits>
class KnotTheoryDistance : public DistanceMetricMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;

    KnotTheoryDistance();
    KnotTheoryDistance(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node);
    virtual ~KnotTheoryDistance();
    
    virtual double Distance(Environment* _env, const CfgType& _c1, const CfgType& _c2);
    
  protected:
    virtual vector<Vector3D> GetCoordinatesForKnot(const CfgType& _c, Environment* _env);
    double Knot(vector<Vector3D>& _c1, vector<Vector3D>& _c2);
};

template<class MPTraits>
KnotTheoryDistance<MPTraits>::KnotTheoryDistance() : DistanceMetricMethod<MPTraits>() {
  this->m_name = "KnotTheory";  
}

template<class MPTraits>
KnotTheoryDistance<MPTraits>::
KnotTheoryDistance(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node) : DistanceMetricMethod<MPTraits>(_problem, _node) {
  this->m_name = "KnotTheory"; 
}

template<class MPTraits>
KnotTheoryDistance<MPTraits>::~KnotTheoryDistance(){
}

template<class MPTraits>
double KnotTheoryDistance<MPTraits>::Distance(Environment* _env, const CfgType& _c1, const CfgType& _c2) {
  vector<Vector3D> c1 = GetCoordinatesForKnot(_c1, _env);
  vector<Vector3D> c2 = GetCoordinatesForKnot(_c2, _env);
  return Knot(c1, c2);
}

template<class MPTraits>
vector<Vector3D> 
KnotTheoryDistance<MPTraits>::GetCoordinatesForKnot(const CfgType& _c, Environment* _env) {
  _c.ConfigEnvironment(_env);
  vector<Vector3D> coordinates;
  for(int i=0; i< _env->GetMultiBody(_c.GetRobotIndex())->GetFreeBodyCount(); ++i)
    coordinates.push_back(_env->GetMultiBody(_c.GetRobotIndex())->GetFreeBody(i)->WorldTransformation().m_position); 
  return coordinates;
}

template<class MPTraits>
double 
KnotTheoryDistance<MPTraits>::Knot(vector<Vector3D>& _c1, vector<Vector3D>& _c2){

  if (_c1.empty() || _c1.size()<2) {
    cerr << "\n\nError in KnotTheoryDistance::Distance(), _c1 has too few links, exiting.\n";
    exit(1);
  }

  vector<Vector3D> unitVect(_c1.size()), unitVect2(_c2.size());
  double sum = 0.0, sign = 0, signsum=0;
  for(size_t i = 0; i < _c1.size()-1; ++i) {
    if((_c1[i+1]-_c1[i]).magnitude() != 0)
      unitVect[i] = (_c1[i+1]+_c1[i])/(_c1[i+1]-_c1[i]).magnitude();
    else
      unitVect[i] = 0;
    if((_c2[i+1]-_c2[i]).magnitude() !=0)
      unitVect2[i]= (_c2[i+1]+_c2[i])/(_c2[i+1]-_c2[i]).magnitude();
    else 
      unitVect2[i]=0;
    double area = unitVect[i].crossProduct(unitVect2[i]).magnitude();
    sign = (unitVect2[i].dotProduct(unitVect[i]));
    sum += area;  
    signsum += sign;
  }
  
  return signsum;
}

#endif

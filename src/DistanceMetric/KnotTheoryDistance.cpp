#include "KnotTheoryDistance.h"


KnotTheoryDistance::KnotTheoryDistance() : EuclideanDistance(){
  m_name = "KnotTheory";  
}

KnotTheoryDistance::
KnotTheoryDistance(XMLNodeReader& _node, MPProblem* _problem, bool _warn): EuclideanDistance(_node, _problem, _warn){
  m_name = "KnotTheory";  
}

KnotTheoryDistance::~KnotTheoryDistance(){
}

double KnotTheoryDistance ::Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2){
  double sum =0.0, sign = 0,sign2 =0, temp1,dist,nn;
  dist =EuclideanDistance::Distance(_env, _c1,_c2);
  Vector3D n1,n2;
  vector <Vector3D> unitVect(3000), unitVect2(3000), c11, c21;
  vector<Vector3D> c1=_c1.PolyApprox(_env);
  vector<Vector3D> c2=_c2.PolyApprox(_env);
  if (c1.empty()|| c1.size()<2){
    cerr << "\n\nError in KnotTheoryDistance::Distance(), c1 has too few links, exiting.\n";
    exit(-1);
  }
  for(size_t i =0; i < c1.size()-1;i++){
    if((c1[i+1]-c1[i]).magnitude() != 0)
      unitVect[i] = (c1[i+1]+c1[i]);
    else
      unitVect[i] = 0;
    if((c2[i+1]-c2[i]).magnitude() !=0)
      unitVect2[i]= (c2[i+1]+c2[i]);
    else 
      unitVect2[i]=0;
    if((unitVect[i].crossProduct(unitVect2[i])).magnitude() != 0)
      n1= (unitVect[i].crossProduct(unitVect2[i]))/(unitVect[i].crossProduct(unitVect2[i])).magnitude();
    else
      n1 = 0.0;
      nn =unitVect[i].crossProduct(unitVect2[i]).magnitude();
    if((unitVect2[i].crossProduct(unitVect[i])).magnitude() != 0)
      n2 =  (unitVect2[i].crossProduct(unitVect[i]))/(unitVect2[i].crossProduct(unitVect[i])).magnitude();
    else
      n2 = 0.0;
    if(n1.dotProduct(n2)>1)
      temp1 = 1.0;
    else
      temp1 = n1.dotProduct(n2);
    sign =(n1.crossProduct(n2)).dotProduct(unitVect[i]); 
    sum += nn; 
  }
 sign2 = sign/fabs(sign);
  return  sum/(4.0); 
}

void KnotTheoryDistance::PrintOptions(ostream& _os) const {
  _os << "\n" << GetName() << " ";
  _os << endl;
}


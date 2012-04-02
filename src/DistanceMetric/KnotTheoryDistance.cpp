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
  double sum =0.0, sign = 0,sign2 =0,dist,area;
  dist =EuclideanDistance::Distance(_env, _c1,_c2);
  vector <Vector3D> unitVect(3000),unitVect2(3000);
  vector<Vector3D> c1=_c1.PolyApprox(_env);
  vector<Vector3D> c2=_c2.PolyApprox(_env);
  
   if (c1.empty()|| c1.size()<2){
    cerr << "\n\nError in KnotTheoryDistance::Distance(), c1 has too few links, exiting.\n";
    exit(-1);
  }
  for(size_t i =0; i < c1.size()-1;i++){
    if((c1[i+1]-c1[i]).magnitude() != 0)
      unitVect[i] = (c1[i+1]+c1[i])/(c1[i+1]-c1[i]).magnitude();
    else
      unitVect[i] = 0;
    if((c2[i+1]-c2[i]).magnitude() !=0)
      unitVect2[i]= (c2[i+1]+c2[i])/(c2[i+1]-c2[i]).magnitude();
    else 
      unitVect2[i]=0;
    
     area = unitVect[i].crossProduct(unitVect2[i]).magnitude();
  
    sign =(unitVect2[i].dotProduct(unitVect[i]));
    sum += area; 
  }
    sign2 = sign/fabs(sign);
  return  sum/(4.0) * sign2; 
}

void KnotTheoryDistance::PrintOptions(ostream& _os) const {
  _os << "\n" << GetName() << " ";
  _os << endl;
}


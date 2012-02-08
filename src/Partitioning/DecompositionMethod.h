#ifndef DECOMPOSITIONMETHOD_H_
#define DECOMPOSITIONMETHOD_H_
//other classes(workspace decomposition, clustering decomposition) will extend the class, and use their own functions
class DecompositionMethod : public MPBaseObject
{
 
 public:
  DecompositionMethod();
  ~DecompositionMethod();

  virtual RegionGraph Decompose (shared_ptr<BoundingBox> = NULL, vector<VID> _v = vector <VID> () ) =0 ; 
  

};

#endif


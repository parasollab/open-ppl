//////////////////////////////////////////////////////////////////////////////////////////
//  class DecompositionMethod
//
//  This class defines the base class for decomposition methods. All must define
//  Decompose for their workspace/cspace decomposition, or roadmap decomposition
//  methods.
//////////////////////////////////////////////////////////////////////////////////////////

#ifndef DECOMPOSITIONMETHOD_H_
#define DECOMPOSITIONMETHOD_H_

class DecompositionMethod : public MPBaseObject {
  public:
    DecompositionMethod(){}
    ~DecompositionMethod(){}

    virtual RegionGraph Decompose(shared_ptr<BoundingBox> _bbx = NULL, 
        vector<VID> _v = vector<VID>()) = 0; 
};

#endif


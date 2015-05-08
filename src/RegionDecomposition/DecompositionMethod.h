#ifndef DECOMPOSITIONMETHOD_H_
#define DECOMPOSITIONMETHOD_H_

////////////////////////////////////////////////////////////////////////////////
/// @ingroup RegionDecomposition
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class DecompositionMethod : public MPBaseObject {
  public:
    DecompositionMethod(){}
    ~DecompositionMethod(){}

    virtual RegionGraph Decompose(shared_ptr<Boundary> _bbx = NULL,
        vector<VID> _v = vector<VID>()) = 0;
};

#endif


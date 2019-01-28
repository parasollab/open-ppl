#ifndef GRID_FILLING_H_
#define GRID_FILLING_H_

#include "PlacementMethod.h"

#include "MPLibrary/MPTools/InteractionTemplate.h"
#include "Utilities/XMLNode.h"

class GridFilling : public PlacementMethod {

  public:

    ///@name Construction
    ///@{

    GridFilling(MPProblem* _problem);

    GridFilling(MPProblem* _problem, XMLNode& _node);

    ~GridFilling() = default;

    ///@}
    ///@name Interface
    ///@{

    void virtual PlaceIT(InteractionTemplate* _it, MPSolution* _solution, MPLibrary* _library, Coordinator* _coordinator) override;

    ///@}
  protected:

    void CreateGrid(/*Space _spaceType*/);
    void TranslateCfg(const Cfg& _centerCfg, Cfg& _relativeCfg);

  private:

    Environment* m_environment;
    const Boundary* m_boundary;
    vector<Cfg> m_locations;
    int m_squaresX, m_squaresY; //in how many squares/parts we want the env. divided



};

#endif

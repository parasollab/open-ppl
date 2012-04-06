#ifndef _Connector_h_
#define _Connector_h_

// Standard Headers
#include <sstream>

// Roadmap Headers
#include "Roadmap.h"
#include "MPRegion.h"
#include "MPProblem.h"

// Connector Headers
#include "ConnectionMethod.h"
#include "ConnectCCs.h"
#include "NeighborhoodConnection.h"
#include "PreferentialAttachment.h"
#include "OptimalConnection.h"
#include "OptimalRewire.h"
//#include "ClosestVE.h"
//#include "RRTcomponents.h"
//#include "RayTracer.h"

// Utility Headers
#include "MetricUtils.h"

// Namespaces
namespace pmpl_detail { //hide NeighborhoodFinde_rmethodList in pmpl_detail namespace
  typedef boost::mpl::list< NeighborhoodConnection<CfgType,WeightType>, 
          ConnectCCs<CfgType,WeightType>,
          PreferentialAttachment<CfgType,WeightType>,
          OptimalConnection<CfgType,WeightType>,
          OptimalRewire<CfgType,WeightType>
            //ClosestVE<CfgType,WeightType>
            > ConnectorMethodList;
}

//#############################################################################
// A collection of connection methods
//#############################################################################
template <class CFG, class WEIGHT>
class Connector :  private ElementSet< ConnectionMethod<CFG,WEIGHT> >,
  public MPBaseObject{

    private:
      typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;

    public:
      typedef typename ElementSet<ConnectionMethod<CFG,WEIGHT> >::MethodPointer ConnectionPointer;

      ////////////////////////////////////////////////////////////////////////////////////
      /* CONSTRUCTORS AND DESTRUCTORS */
      ////////////////////////////////////////////////////////////////////////////////////
      Connector() : ElementSet< ConnectionMethod<CFG,WEIGHT> >(pmpl_detail::ConnectorMethodList()){}

      Connector(XMLNodeReader& inNode, MPProblem* in_pProblem) 
        : ElementSet< ConnectionMethod<CFG,WEIGHT> >(pmpl_detail::ConnectorMethodList()), MPBaseObject(inNode, in_pProblem){
          ParseXML(inNode);
        }

      ~Connector(){}

      ////////////////////////////////////////////////////////////////////////////////////
      /* ACCESS METHODS */
      ////////////////////////////////////////////////////////////////////////////////////
      ConnectionPointer GetMethod(const string& _label){
        ConnectionPointer toReturn = ElementSet<ConnectionMethod<CFG,WEIGHT> >::GetElement(_label);
        if(toReturn.get() == NULL){
          cerr << "Connector::GetMethod(..) - Error: " << _label << " not found!" << endl;
          exit(-1);
        }
        return toReturn;
      }

      ////////////////////////////////////////////////////////////////////////////////////
      /* DEBUG METHODS */
      ////////////////////////////////////////////////////////////////////////////////////
      void PrintOptions(ostream& _os);

      ////////////////////////////////////////////////////////////////////////////////////
      /* CONNECTOR METHODS */
      ////////////////////////////////////////////////////////////////////////////////////
      void Connect( ConnectionPointer _selected,
          Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats);

      template<typename OutputIterator>
        void Connect( ConnectionPointer _selected,
            Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats,
            OutputIterator _collision);

      template<typename InputIterator>
        void Connect( ConnectionPointer _selected, 
            Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats,
            InputIterator _itr1First, InputIterator _itr1Last);

      template<typename InputIterator>
        void Connect( ConnectionPointer _selected, 
            Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats,
            InputIterator _itr1First, InputIterator _itr1Last,
            InputIterator _itr2First, InputIterator _itr2Last);

      template<typename InputIterator, typename OutputIterator>
        void Connect( ConnectionPointer _selected, 
            Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats,
            InputIterator _itr1First, InputIterator _itr1Last, 
            OutputIterator _collision);

      template<typename InputIterator, typename OutputIterator>
        void Connect( ConnectionPointer _selected, 
            Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats,
            InputIterator _itr1First, InputIterator _itr1Last,
            InputIterator _itr2First, InputIterator _itr2Last, 
            OutputIterator _collision);

    protected:
      void ParseXML(XMLNodeReader& _node);
  };

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
void Connector<CFG,WEIGHT>::PrintOptions(ostream& _os){
  _os << "  Connection Methods" << endl;
  typename map<string, ConnectionPointer >::const_iterator Conn;
  for(  Conn = ElementSet<ConnectionMethod<CFG,WEIGHT> > ::ElementsBegin(); 
      Conn != ElementSet<ConnectionMethod<CFG,WEIGHT> >::ElementsEnd(); 
      ++Conn){
    _os << "  " << Conn->first << "::\t";
    Conn->second->PrintOptions(_os);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
void Connector<CFG,WEIGHT>::ParseXML(XMLNodeReader& inNode) {
  XMLNodeReader::childiterator citr;
  //Iterate over child nodes
  for(citr = inNode.children_begin(); citr!= inNode.children_end(); ++citr) {
    if (!ElementSet<ConnectionMethod<CFG,WEIGHT> >::AddElement(citr->getName(), *citr, GetMPProblem())){
      citr->warnUnknownNode();
      exit(-1);
    }
  }
  PrintOptions(cout);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
void Connector<CFG,WEIGHT>::Connect(ConnectionPointer _selected,
    Roadmap<CFG,WEIGHT>* _rm, StatClass& _stats){
  vector<CFG> collision;
  Connect(_selected, _rm, _stats, back_inserter(collision));
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
template<typename OutputIterator>
void Connector<CFG,WEIGHT>::Connect(ConnectionPointer _selected, 
    Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats,
    OutputIterator _collision) {
  vector<VID> verts;
  _rm->m_pRoadmap->GetVerticesVID(verts);
  Connect(_selected, _rm, _stats, verts.begin(), verts.end(), verts.begin(), verts.end(), _collision);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
template<typename InputIterator, typename OutputIterator>
void Connector<CFG,WEIGHT>::Connect(ConnectionPointer _selected, 
    Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats,
    InputIterator _itr1First, InputIterator _itr1Last, 
    OutputIterator _collision) {
  Connect(_selected, _rm, _stats, _itr1First, _itr1Last, _itr1First, _itr1Last, _collision);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
template<typename InputIterator>
void Connector<CFG,WEIGHT>::Connect(ConnectionPointer _selected, 
    Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats,
    InputIterator _itr1First, InputIterator _itr1Last){
  vector<CFG> collision;
  Connect(_selected, _rm, _stats, _itr1First, _itr1Last, back_inserter(collision));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
template<typename InputIterator, typename OutputIterator>
void Connector<CFG,WEIGHT>::Connect(ConnectionPointer _selected, 
    Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats,
    InputIterator _itr1First, InputIterator _itr1Last,
    InputIterator _itr2First, InputIterator _itr2Last, 
    OutputIterator _collision) {
  string name = _selected.get()->GetName();

  if(name == "NeighborhoodConnection"){
    dynamic_cast<NeighborhoodConnection<CFG,WEIGHT>* >(_selected.get())->
      Connect(_rm, _stats, _itr1First, _itr1Last, _itr2First, _itr2Last, _collision);
  }
  else if(name == "ConnectCCs"){
    dynamic_cast<ConnectCCs<CFG,WEIGHT>* >(_selected.get())->
      Connect(_rm, _stats, _itr1First, _itr1Last, _itr2First, _itr2Last, _collision);
  }
  else if(name == "PreferentialAttachment"){
    dynamic_cast<PreferentialAttachment<CFG,WEIGHT>* >(_selected.get())->
      Connect(_rm, _stats, _itr1First, _itr1Last, _itr2First, _itr2Last, _collision);
  }
  else if(name == "OptimalConnection"){
    dynamic_cast<OptimalConnection<CFG,WEIGHT>* >(_selected.get())->
      Connect(_rm, _stats, _itr1First, _itr1Last, _itr2First, _itr2Last, _collision);
  }
  else if(name == "OptimalRewire"){
    dynamic_cast<OptimalRewire<CFG,WEIGHT>* >(_selected.get())->
      Connect(_rm, _stats, _itr1First, _itr1Last, _itr2First, _itr2Last, _collision);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
template<typename InputIterator>
void Connector<CFG,WEIGHT>::Connect(ConnectionPointer _selected, 
    Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats,
    InputIterator _itr1First, InputIterator _itr1Last,
    InputIterator _itr2First, InputIterator _itr2Last){
  vector<CFG> collision;
  Connect(_selected, _rm, _stats, _itr1First, _itr1Last, _itr2First, _itr2Last, back_inserter(collision));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///
///
/// End new Connect interface
///
///

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#endif /*_Connector_h_*/


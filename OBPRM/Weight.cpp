// Weight.cpp: implementation of the Weight class.
//
//////////////////////////////////////////////////////////////////////

#include "Weight.h"
#include "OBPRM.h"

bool IWeight::equal( const IWeight * tmp ) const 
{ 
	return ((lp==(tmp)->lp)&&(weight==(tmp)->weight)); 
}

void IWeight::Input( istream& in )
{ 
	in>> lp >> weight; 
}

void IWeight::Output( ostream& out ) const 
{ 
	out<< lp << " " << weight; 
}

double& IWeight::Weight() { return weight; }
int& IWeight::LP() { return lp; }

/////////////////////////////////////////////////////////////
//
//	IntWeight : pub IWeight
//
/////////////////////////////////////////////////////////////

IntWeight::IntWeight(){ lp = INVALID_LP; weight=1; }

IWeight* IntWeight::clone() const{
	IntWeight *pTmp=new IntWeight();
	pTmp->Weight()=weight;
	pTmp->LP()=lp;
	return pTmp;
}

/////////////////////////////////////////////////////////////
//
//	DblWeight : pub IWeight
//
/////////////////////////////////////////////////////////////

/**Represention for edge weights in roadmap graph.
*"special" edges, for example, for protein folding.
*/
DblWeight::DblWeight(){ lp=INVALID_LP; weight = INVALID_DBL; }

IWeight* DblWeight::clone() const{
	DblWeight *pTmp=new DblWeight();
	pTmp->Weight()=weight;
	pTmp->LP()=lp;
	return pTmp;
}

/////////////////////////////////////////////////////////////
//
//	WeightFactory
//
/////////////////////////////////////////////////////////////
bool DefaulWeightFactory::Create( IWeight ** ppIWeight /*in/out*/ ) const
{
	//check input
	if( ppIWeight==NULL ) return false;
	*ppIWeight = new IntWeight();
	if( *ppIWeight==NULL ) return false; //not enough memory
	return true;
}

/////////////////////////////////////////////////////////////
//
//	WeightObject
//
/////////////////////////////////////////////////////////////

DefaulWeightFactory * WeightObject::m_pFactory = new DefaulWeightFactory();
double WeightObject::WO_MAX_WEIGHT=MAX_DBL;
int    WeightObject::WO_INVALID_LP=INVALID_LP;

/////////////////////////////////////////////////////////////
//
//	Constructors and Destructor
//	These functions create default IWeight if on clients
//	speicifed their own IWeight.
//
/////////////////////////////////////////////////////////////

WeightObject::WeightObject(){
	assert(m_pFactory!=NULL);
	bool bResult = m_pFactory->Create( &m_pIWeight );
	assert(bResult);
}

WeightObject::WeightObject( int lpID ){
	
	assert(m_pFactory!=NULL);
	bool bResult = m_pFactory->Create( &m_pIWeight );
	assert(bResult);
	m_pIWeight->LP() = lpID;
}

WeightObject::WeightObject( int lpID, double weight ){
	
	assert(m_pFactory!=NULL);
	bool bResult = m_pFactory->Create( &m_pIWeight );
	assert(bResult);
	m_pIWeight->LP() = lpID;
	m_pIWeight->Weight() = weight;	
}

WeightObject::WeightObject( const WeightObject & WeightObj ){

	*this=WeightObj;
}

WeightObject::WeightObject( IWeight * pIWeight ){
	m_pIWeight=pIWeight;
}

WeightObject::~WeightObject(){ 
	delete m_pIWeight; 
}

/////////////////////////////////////////////////////////////
//
//	Graph.h Interface
//
/////////////////////////////////////////////////////////////

//A static function returns invalid weight
WeightObject WeightObject::InvalidWeight() { return WeightObject((IWeight *)NULL); }
double & WeightObject::MaxWeight() { return WO_MAX_WEIGHT; }; ///< For Dijkstra's Alg

///Copy values from antoher given IntWeight instance.
bool WeightObject::operator==(const WeightObject &tmp) const
{
	//if both are invalid weight
	if( tmp.m_pIWeight==NULL && m_pIWeight==NULL )
		return true;
	//if one of them are invlid
	if( tmp.m_pIWeight==NULL || m_pIWeight==NULL )
		return false;
	return m_pIWeight->equal(tmp.m_pIWeight);
}

const WeightObject & WeightObject::operator=(const WeightObject & WeightObj)
{
	if( WeightObj.m_pIWeight==NULL){
		m_pIWeight=NULL;
	}
	else{
		m_pIWeight=WeightObj.m_pIWeight->clone();
	}

	return *this;
}

/////////////////////////////////////////////////////////////
//
//	Access Methods
//
/////////////////////////////////////////////////////////////

IWeight * WeightObject::GetIWeight(){ return m_pIWeight; }
const IWeight * WeightObject::GetIWeight() const{ return m_pIWeight; }

const DefaulWeightFactory * 
WeightObject::GetWeightFactory(){
	return m_pFactory;
}

void WeightObject::SetWeightFactory(DefaulWeightFactory * fact)
{
	if( m_pFactory!=NULL )
		delete m_pFactory;
	m_pFactory = fact;
}

/////////////////////////////////////////////////////////////
//
//	Global Methods
//
/////////////////////////////////////////////////////////////

ostream& operator<< (ostream& _os, const WeightObject& w) {
	if( w.m_pIWeight==NULL) return _os;
	w.m_pIWeight->Output(_os);
	return _os;
}

istream& operator>> (istream& _is, WeightObject& w) {
	if( w.m_pIWeight==NULL) return _is;
	w.m_pIWeight->Input(_is);
	return _is;
}
